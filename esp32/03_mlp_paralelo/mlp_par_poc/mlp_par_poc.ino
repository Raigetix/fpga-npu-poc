// mlp_par_poc.ino -- ESP32-S3 como maestro SPI de la version PARALELA (8
// carriles MAC) de la red 130->128->64->32->5, implementada en
// fpga_NPU_poc/top_mlp_par.v. A diferencia de mlp_poc.ino: los pesos se
// reparten en 8 bancos (uno por carril, neurona%8) y se cargan en RAFAGAS de
// 5 bytes por transaccion SPI en vez de 1, porque con 8 multiplicadores en
// paralelo el cuello de botella deja de ser el computo y pasa a ser el I/O.
//
// IMPORTANTE: este sketch asume que la FPGA tiene cargado 'top_mlp_par'.
// Conexionado: igual que los otros sketches (ver npu_poc.ino).

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP      0x00
#define CMD_LOAD_W   0x03
#define CMD_LOAD_B   0x02
#define CMD_START    0x04
#define CMD_DBG_RD   0x06
#define CMD_SET_WTGT 0x07
#define CMD_SET_ITGT 0x0A
#define CMD_IBURST5  0x0B
// sel=5 en CMD_DBG_RD: lee weight_bank[w_lane][w_ptr], apuntados por el
// CMD_SET_WTGT mas reciente (solo para lectura de debug, no para escribir).
//
// La carga de PESOS es 1 transaccion SPI = 1 peso (CMD_LOAD_W), no rafaga:
// la carga en rafaga (8 bancos, secuenciador de varios ciclos en la FPGA)
// resulto ser una fuente de corrupcion intermitente e irreproducible entre
// resets, sin causa raiz identificada pese a probar distintas frecuencias.
// Como la carga de pesos es un costo unico al arrancar (no se repite en el
// loop de inferencia que se compara contra el ESP32), no vale la pena el
// riesgo -- ver top_mlp_par.v para el detalle. input_mem si sigue en
// rafaga porque su tiempo SI cuenta en cada inferencia, y viene
// funcionando perfecto (130/130 en todas las pruebas).

#define NLANES 8

// ---- Dimensiones de la red (deben coincidir EXACTO con mlp_engine_par.v) ----
#define IN_COUNT  130
#define H1_COUNT  128
#define H2_COUNT  64
#define H3_COUNT  32
#define OUT_COUNT 5

// ---- Direcciones "planas" originales (para el calculo de referencia en
// software, igual que en mlp_poc.ino) ----
#define W1_BASE 0
#define W1_SIZE (IN_COUNT * H1_COUNT)
#define W2_BASE (W1_BASE + W1_SIZE)
#define W2_SIZE (H1_COUNT * H2_COUNT)
#define W3_BASE (W2_BASE + W2_SIZE)
#define W3_SIZE (H2_COUNT * H3_COUNT)
#define W4_BASE (W3_BASE + W3_SIZE)
#define W4_SIZE (H3_COUNT * OUT_COUNT)
#define W_TOTAL (W4_BASE + W4_SIZE)

#define B1_BASE 0
#define B2_BASE (B1_BASE + H1_COUNT)
#define B3_BASE (B2_BASE + H2_COUNT)
#define B4_BASE (B3_BASE + H3_COUNT)
#define B_TOTAL (B4_BASE + OUT_COUNT)

// ---- Mapa de bancos por carril (direccion LOCAL, misma numeracion que el
// comentario de mlp_engine_par.v) ----
struct LayerCfg { int in_count, out_count, waves, flat_base, lane_base; };
LayerCfg LAYERS[4] = {
  {IN_COUNT, H1_COUNT, 16, W1_BASE, 0},
  {H1_COUNT, H2_COUNT, 8,  W2_BASE, 2080},
  {H2_COUNT, H3_COUNT, 4,  W3_BASE, 3104},
  {H3_COUNT, OUT_COUNT, 1, W4_BASE, 3360},
};
#define LANE_BANK_SIZE 3392 // direcciones locales realmente usadas por carril (0..3391)

SPIClass fpga_spi(FSPI);

int8_t weights[W_TOTAL];
int8_t biases[B_TOTAL];
int8_t h1[H1_COUNT], h2[H2_COUNT], h3[H3_COUNT];

#define SPI_HZ 8000000
void spi_burst_begin() { fpga_spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0)); }
void spi_burst_end()   { fpga_spi.endTransaction(); }

void npu_xfer(uint8_t cmd, uint16_t a, uint16_t b_raw, uint8_t rx[6]) {
  uint8_t tx[6] = {
    cmd, (uint8_t)(a >> 8), (uint8_t)(a & 0xFF),
    (uint8_t)(b_raw >> 8), (uint8_t)(b_raw & 0xFF), 0x00
  };
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}

// Frame de rafaga: 5 bytes de carga util (sin nocion de direccion, el
// puntero interno de la FPGA ya sabe donde escribir y auto-incrementa).
void npu_xfer_burst(uint8_t cmd, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t rx[6]) {
  uint8_t tx[6] = {cmd, b0, b1, b2, b3, b4};
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}

// bit1 del status = pll_lock (ver top_mlp_par.v: tx_snapshot[..]=
// {6'd0,pll_lock,busy,...}). El carril 0 (primer carril cargado, justo al
// arrancar el sketch) mostro muchas mas fallas de carga que el resto y sin
// converger con reintentos -- sospecha de que el PLL todavia no estaba
// bloqueado en los primerisimos frames SPI. Se chequea explicitamente antes
// de mandar nada.
void wait_pll_lock() {
  uint8_t rx[6];
  uint32_t t0 = micros();
  bool locked = false;
  spi_burst_begin();
  for (int i = 0; i < 1000 && !locked; i++) {
    npu_xfer(CMD_NOP, 0, 0, rx);
    locked = (rx[0] & 0x02) != 0;
    if (!locked) delayMicroseconds(100);
  }
  spi_burst_end();
  uint32_t dt = micros() - t0;
  Serial.printf("PLL lock: %s (tardo %lu us en confirmarse)\n", locked ? "OK" : "NUNCA SE VIO EN 1000 INTENTOS", (unsigned long)dt);
}

void npu_send(uint8_t cmd, uint16_t a, uint16_t b_raw) {
  uint8_t rx[6];
  spi_burst_begin();
  npu_xfer(cmd, a, b_raw, rx);
  spi_burst_end();
}

int8_t npu_dbg_read(uint8_t sel, uint8_t addr) {
  uint8_t rx[6];
  uint16_t a = ((uint16_t)sel << 8) | addr;
  spi_burst_begin();
  npu_xfer(CMD_DBG_RD, a, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  return (int8_t)rx[5];
}

void npu_wait_done(int8_t out[OUT_COUNT], int *polls_out) {
  uint8_t rx[6];
  int polls = 0;
  spi_burst_begin();
  bool busy = true;
  while (busy) {
    npu_xfer(CMD_NOP, 0, 0, rx);
    busy = (rx[0] & 0x01) != 0;
    polls++;
  }
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  out[0] = (int8_t)rx[1]; out[1] = (int8_t)rx[2]; out[2] = (int8_t)rx[3];
  out[3] = (int8_t)rx[4]; out[4] = (int8_t)rx[5];
  *polls_out = polls;
}

// Mismo post-procesamiento que el hardware: ReLU + saturacion a [0,127].
int8_t relu_sat(int32_t v) {
  if (v < 0) return 0;
  if (v > 127) return 127;
  return (int8_t)v;
}

void mlp_forward_sw(const int8_t *input, int8_t *output) {
  for (int n = 0; n < H1_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < IN_COUNT; i++) acc += (int32_t)weights[W1_BASE + n * IN_COUNT + i] * (int32_t)input[i];
    acc += biases[B1_BASE + n];
    h1[n] = relu_sat(acc);
  }
  for (int n = 0; n < H2_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < H1_COUNT; i++) acc += (int32_t)weights[W2_BASE + n * H1_COUNT + i] * (int32_t)h1[i];
    acc += biases[B2_BASE + n];
    h2[n] = relu_sat(acc);
  }
  for (int n = 0; n < H3_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < H2_COUNT; i++) acc += (int32_t)weights[W3_BASE + n * H2_COUNT + i] * (int32_t)h2[i];
    acc += biases[B3_BASE + n];
    h3[n] = relu_sat(acc);
  }
  for (int n = 0; n < OUT_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < H3_COUNT; i++) acc += (int32_t)weights[W4_BASE + n * H3_COUNT + i] * (int32_t)h3[i];
    acc += biases[B4_BASE + n];
    output[n] = relu_sat(acc);
  }
}

// ---- Carga de los pesos de UN carril, un peso por transaccion SPI ----
// Recorre las 4 capas en el orden en que estan concatenadas dentro del banco
// de este carril (ver LAYERS[].lane_base), tomando de 'weights[]' (layout
// plano original) el valor que le corresponde a la neurona (ola*8+lane).
// CMD_LOAD_W manda {carril, direccion local, valor} en una sola transaccion
// (sin puntero/secuenciador del lado FPGA) -- ver comentario de cabecera del
// porque no se usa rafaga aca.
void send_lane_weights(int lane) {
  uint8_t rx[6];
  spi_burst_begin();
  for (int L = 0; L < 4; L++) {
    LayerCfg &c = LAYERS[L];
    for (int w = 0; w < c.waves; w++) {
      int neuron = w * NLANES + lane;
      for (int i = 0; i < c.in_count; i++) {
        int8_t val = (neuron < c.out_count) ? weights[c.flat_base + neuron * c.in_count + i] : 0;
        int local_addr = c.lane_base + w * c.in_count + i;
        uint16_t a = ((uint16_t)lane << 12) | (uint16_t)local_addr;
        npu_xfer(CMD_LOAD_W, a, (uint16_t)(uint8_t)val, rx);
      }
    }
  }
  spi_burst_end();
}

// Recorre las 4 capas de la misma forma que send_lane_weights() para
// encontrar a que valor esperado corresponde una direccion LOCAL dada
// (necesario para poder reintentar UNA direccion puntual sin recorrer todo
// de nuevo).
int8_t weight_at_local_addr(int lane, int local_addr) {
  for (int L = 0; L < 4; L++) {
    LayerCfg &c = LAYERS[L];
    int span = c.waves * c.in_count;
    if (local_addr >= c.lane_base && local_addr < c.lane_base + span) {
      int rel = local_addr - c.lane_base;
      int w = rel / c.in_count;
      int i = rel % c.in_count;
      int neuron = w * NLANES + lane;
      return (neuron < c.out_count) ? weights[c.flat_base + neuron * c.in_count + i] : 0;
    }
  }
  return 0;
}

// Lee de vuelta las LANE_BANK_SIZE direcciones reales del carril y compara
// contra lo que deberian ser. Llena mismatch_addr[] (hasta max_mismatch)
// con las direcciones que fallaron y devuelve cuantas fallaron en total.
int verify_lane_weights(int lane, int *mismatch_addr, int max_mismatch) {
  int count = 0;
  for (int addr = 0; addr < LANE_BANK_SIZE; addr++) {
    npu_send(CMD_SET_WTGT, ((uint16_t)lane << 12) | (uint16_t)addr, 0);
    int8_t got = npu_dbg_read(5, 0);
    int8_t expected = weight_at_local_addr(lane, addr);
    if (got != expected) {
      if (count < max_mismatch) mismatch_addr[count] = addr;
      count++;
    }
  }
  return count;
}

// La carga de pesos por SPI muestra corrupcion intermitente en un puñado de
// direcciones (causa raiz no identificada, ver README de 03_mlp_paralelo).
// Como el mecanismo de lectura de verificacion (CMD_SET_WTGT+CMD_DBG_RD
// sel=5) resulto confiable, en vez de perseguir la causa exacta: cargar,
// leer de vuelta, y reintentar SOLO las direcciones que fallaron, hasta que
// quede todo bien (o se agoten los reintentos).
#define MAX_WEIGHT_MISMATCH 256
void send_lane_weights_with_retry(int lane, int max_retries = 8) {
  send_lane_weights(lane);
  int mismatch_addr[MAX_WEIGHT_MISMATCH];
  for (int attempt = 1; attempt <= max_retries; attempt++) {
    int n = verify_lane_weights(lane, mismatch_addr, MAX_WEIGHT_MISMATCH);
    if (n == 0) {
      if (attempt > 1) Serial.printf("  carril %d: OK despues de %d reintento(s)\n", lane, attempt - 1);
      return;
    }
    Serial.printf("  carril %d: %d direccion(es) mal (intento %d/%d), reintentando...\n", lane, n, attempt, max_retries);
    int fix_count = (n < MAX_WEIGHT_MISMATCH) ? n : MAX_WEIGHT_MISMATCH;
    uint8_t rx[6];
    spi_burst_begin();
    for (int k = 0; k < fix_count; k++) {
      int addr = mismatch_addr[k];
      int8_t val = weight_at_local_addr(lane, addr);
      uint16_t a = ((uint16_t)lane << 12) | (uint16_t)addr;
      npu_xfer(CMD_LOAD_W, a, (uint16_t)(uint8_t)val, rx);
    }
    spi_burst_end();
  }
  Serial.printf("  carril %d: SIGUE MAL despues de %d reintentos!\n", lane, max_retries);
}

void load_weights_and_biases() {
  Serial.println("Generando pesos/bias aleatorios (rango -2..2)...");
  randomSeed(42);
  for (int i = 0; i < W_TOTAL; i++) weights[i] = (int8_t)random(-2, 3);
  for (int i = 0; i < B_TOTAL; i++) biases[i]  = (int8_t)random(-2, 3);

  Serial.println("Cargando pesos en la FPGA (con verificacion y reintento por carril)...");
  uint32_t t0 = micros();

  for (int lane = 0; lane < NLANES; lane++) send_lane_weights_with_retry(lane);

  uint8_t rx[6];
  spi_burst_begin();
  for (int i = 0; i < B_TOTAL; i++)
    npu_xfer(CMD_LOAD_B, (uint16_t)i, (uint16_t)(uint8_t)biases[i], rx);
  spi_burst_end();

  uint32_t t1 = micros();
  Serial.printf("Carga completa en %.2f ms\n\n", (t1 - t0) / 1000.0);
}

// Test aislado: carga un patron conocido en input_mem por rafaga, y lo lee
// de vuelta con CMD_DBG_RD (sel=4) para confirmar si la carga en si funciona,
// sin meter la inferencia en el medio.
void test_input_burst_loading() {
  Serial.println("=== Test aislado: carga en rafaga de input_mem ===");
  int8_t pattern[IN_COUNT];
  for (int i = 0; i < IN_COUNT; i++) pattern[i] = (int8_t)((i % 5) - 2);

  uint8_t rx[6];
  npu_send(CMD_SET_ITGT, 0, 0);
  uint8_t buf[5];
  int bufcount = 0;
  spi_burst_begin();
  for (int i = 0; i < IN_COUNT; i++) {
    buf[bufcount++] = (uint8_t)pattern[i];
    if (bufcount == 5) {
      npu_xfer_burst(CMD_IBURST5, buf[0], buf[1], buf[2], buf[3], buf[4], rx);
      bufcount = 0;
    }
  }
  spi_burst_end();

  int mism = 0;
  for (int i = 0; i < IN_COUNT; i++) {
    int8_t got = npu_dbg_read(4, i);
    if (got != pattern[i]) {
      mism++;
      if (mism <= 10) Serial.printf("  input_mem[%3d]: esperado=%4d  fpga=%4d\n", i, pattern[i], got);
    }
  }
  Serial.printf("input_mem: %d/%d coinciden\n\n", IN_COUNT - mism, IN_COUNT);
}

// Lee bias_mem de vuelta (CMD_DBG_RD sel=6, direccion plana 0..228) y lo
// compara contra 'biases[]'. Bias usa el patron de escritura mas simple
// posible (1 memoria compartida, sin decodificador de carril) -- si esto
// TAMBIEN sale corrupto, el problema no es especifico del camino de pesos.
void test_bias_readback() {
  Serial.println("=== Test aislado: bias_mem (lectura) ===");
  int mism = 0;
  for (int i = 0; i < B_TOTAL; i++) {
    int8_t got = npu_dbg_read(6, i);
    if (got != biases[i]) {
      mism++;
      if (mism <= 12) Serial.printf("  bias[%3d]: esperado=%4d  fpga=%4d\n", i, biases[i], got);
    }
  }
  Serial.printf("bias_mem: %d/%d coinciden\n\n", B_TOTAL - mism, B_TOTAL);
}

// El carril 0 muestra ~90 direcciones mal de forma persistente, sin
// converger con reintentos (a diferencia del resto de los carriles, que
// convergen en 2-4 intentos). Para distinguir "el DATO guardado esta mal"
// de "la LECTURA de verificacion en si es inestable para este carril": lee
// las mismas 3392 direcciones DOS VECES seguidas, sin reescribir nada en el
// medio, y compara una lectura contra la otra (no contra lo esperado). Si
// las dos lecturas coinciden entre si (aunque difieran de lo esperado), el
// dato esta mal guardado. Si difieren entre si, la lectura misma es la que
// falla.
void test_lane0_read_stability() {
  Serial.println("=== Test: estabilidad de lectura del carril 0 (2 pasadas, sin reescribir) ===");
  static int8_t pass1[LANE_BANK_SIZE];
  static int8_t pass2[LANE_BANK_SIZE];
  for (int addr = 0; addr < LANE_BANK_SIZE; addr++) {
    npu_send(CMD_SET_WTGT, ((uint16_t)0 << 12) | (uint16_t)addr, 0);
    pass1[addr] = npu_dbg_read(5, 0);
  }
  for (int addr = 0; addr < LANE_BANK_SIZE; addr++) {
    npu_send(CMD_SET_WTGT, ((uint16_t)0 << 12) | (uint16_t)addr, 0);
    pass2[addr] = npu_dbg_read(5, 0);
  }
  int diff_between_passes = 0, diff_pass1_expected = 0;
  for (int addr = 0; addr < LANE_BANK_SIZE; addr++) {
    int8_t expected = weight_at_local_addr(0, addr);
    if (pass1[addr] != pass2[addr]) {
      diff_between_passes++;
      if (diff_between_passes <= 10) Serial.printf("  addr %4d: pasada1=%4d  pasada2=%4d  (esperado=%4d)\n", addr, pass1[addr], pass2[addr], expected);
    }
    if (pass1[addr] != expected) diff_pass1_expected++;
  }
  Serial.printf("Pasada1 vs Pasada2 (misma lectura repetida): %d/%d difieren\n", diff_between_passes, LANE_BANK_SIZE);
  Serial.printf("Pasada1 vs esperado: %d/%d difieren\n\n", diff_pass1_expected, LANE_BANK_SIZE);
}

// Lee weight_bank[lane][addr] de vuelta (via CMD_SET_WTGT + CMD_DBG_RD sel=5)
// y lo compara contra el valor que send_lane_weights() calculo para esa
// misma direccion local, para el carril 'lane' completo (las 3392
// direcciones reales). Exhaustivo pero solo para UN carril (representativo);
// si el bug es sistemico (offset, orden de bytes, etc.) alcanza para verlo.
void test_weight_bank_readback(int lane) {
  Serial.printf("=== Test aislado: weight_bank del carril %d (rafaga + lectura) ===\n", lane);
  int mism = 0, checked = 0;
  for (int L = 0; L < 4; L++) {
    LayerCfg &c = LAYERS[L];
    int neuron = -1000; // se recalcula por ola
    for (int w = 0; w < c.waves; w++) {
      int nrn = w * NLANES + lane;
      for (int i = 0; i < c.in_count; i++) {
        int local_addr = c.lane_base + w * c.in_count + i;
        int8_t expected = (nrn < c.out_count) ? weights[c.flat_base + nrn * c.in_count + i] : 0;
        npu_send(CMD_SET_WTGT, ((uint16_t)lane << 12) | (uint16_t)local_addr, 0);
        int8_t got = npu_dbg_read(5, 0);
        checked++;
        if (got != expected) {
          mism++;
          if (mism <= 12) Serial.printf("  L%d w=%d i=%d local_addr=%4d: esperado=%4d  fpga=%4d\n", L, w, i, local_addr, expected, got);
        }
      }
    }
  }
  Serial.printf("weight_bank[carril %d]: %d/%d coinciden\n\n", lane, checked - mism, checked);
}

// Lee h1/h2/h3 de la FPGA (con busy en 0) y los compara contra los que
// quedaron en las globales h1/h2/h3 del ULTIMO mlp_forward_sw().
void debug_compare_layers() {
  Serial.println("  Comparando capas ocultas h1/h2/h3 (FPGA vs software)...");
  int mism1 = 0, mism2 = 0, mism3 = 0;
  for (int i = 0; i < H1_COUNT; i++) {
    int8_t got = npu_dbg_read(0, i);
    if (got != h1[i]) { mism1++; if (mism1 <= 6) Serial.printf("    h1[%3d]: esperado=%4d  fpga=%4d\n", i, h1[i], got); }
  }
  for (int i = 0; i < H2_COUNT; i++) {
    int8_t got = npu_dbg_read(1, i);
    if (got != h2[i]) { mism2++; if (mism2 <= 6) Serial.printf("    h2[%3d]: esperado=%4d  fpga=%4d\n", i, h2[i], got); }
  }
  for (int i = 0; i < H3_COUNT; i++) {
    int8_t got = npu_dbg_read(2, i);
    if (got != h3[i]) { mism3++; if (mism3 <= 6) Serial.printf("    h3[%3d]: esperado=%4d  fpga=%4d\n", i, h3[i], got); }
  }
  Serial.printf("  Diferencias: h1=%d/%d  h2=%d/%d  h3=%d/%d\n\n", mism1, H1_COUNT, mism2, H2_COUNT, mism3, H3_COUNT);
}

void run_inference_once(const int8_t *mfcc, int8_t *got, uint32_t *t_load, uint32_t *t_compute, int *polls_out) {
  uint8_t rx[6];
  uint32_t t0 = micros();

  npu_send(CMD_SET_ITGT, 0, 0);
  uint8_t buf[5];
  int bufcount = 0;
  spi_burst_begin();
  for (int i = 0; i < IN_COUNT; i++) {
    buf[bufcount++] = (uint8_t)mfcc[i];
    if (bufcount == 5) {
      npu_xfer_burst(CMD_IBURST5, buf[0], buf[1], buf[2], buf[3], buf[4], rx);
      bufcount = 0;
    }
  }
  npu_xfer(CMD_START, 0, 0, rx);
  spi_burst_end();
  uint32_t t_loaded = micros();

  npu_wait_done(got, polls_out);
  uint32_t t1 = micros();

  *t_load    = t_loaded - t0;
  *t_compute = t1 - t_loaded;
}

uint32_t total_runs = 0, total_fails = 0;
uint32_t compute_us_min = 0xFFFFFFFF, compute_us_max = 0;
uint64_t compute_us_sum = 0;

void run_inference_test(int iter) {
  int8_t mfcc[IN_COUNT];
  for (int i = 0; i < IN_COUNT; i++) mfcc[i] = (int8_t)random(-2, 3);

  int8_t expected[OUT_COUNT];
  uint32_t sw_t0 = micros();
  mlp_forward_sw(mfcc, expected);
  uint32_t sw_t1 = micros();

  int8_t got[OUT_COUNT];
  uint32_t t_load, t_compute;
  int polls;
  run_inference_once(mfcc, got, &t_load, &t_compute, &polls);

  bool match = true;
  for (int i = 0; i < OUT_COUNT; i++) if (expected[i] != got[i]) match = false;
  total_runs++;
  if (!match) total_fails++;
  if (t_compute < compute_us_min) compute_us_min = t_compute;
  if (t_compute > compute_us_max) compute_us_max = t_compute;
  compute_us_sum += t_compute;

  Serial.printf("--- Iteracion %d ---  [fallo acumulado: %lu/%lu = %.1f%%]\n",
                iter, (unsigned long)total_fails, (unsigned long)total_runs, 100.0f * total_fails / total_runs);
  Serial.printf("ESP32 (software): %6.3f ms -> [%4d %4d %4d %4d %4d]\n",
                (sw_t1 - sw_t0) / 1000.0, expected[0], expected[1], expected[2], expected[3], expected[4]);
  Serial.printf("FPGA (SPI+infer): %6.3f ms -> [%4d %4d %4d %4d %4d]  (carga entrada: %.3f ms, computo+polling: %.3f ms, %d polls)\n",
                (t_load + t_compute) / 1000.0, got[0], got[1], got[2], got[3], got[4],
                t_load / 1000.0, t_compute / 1000.0, polls);
  Serial.println(match ? ">> OK: coinciden" : ">> ERROR: no coinciden");
  if (!match && iter < 3) debug_compare_layers();
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: inferencia PARALELA (8 carriles) via SPI ===\n");
  wait_pll_lock();
  load_weights_and_biases();
  test_input_burst_loading();
  test_bias_readback();
  test_weight_bank_readback(0);
  test_lane0_read_stability();
}

// Barrido de frecuencia de clk_sys: correr exactamente N_TEST_ITERS
// inferencias y despues quedarse quieto con un resumen, en vez de un loop
// infinito -- asi cada paso de frecuencia (27/54/108/216/432 MHz) da un
// resultado limpio y autocontenido sin tener que contar lineas a mano.
#define N_TEST_ITERS 50
int iteration = 0;
bool test_done = false;

void loop() {
  if (!test_done) {
    run_inference_test(iteration++);
    if (iteration >= N_TEST_ITERS) {
      test_done = true;
      Serial.println("========================================");
      Serial.printf("RESUMEN: %lu/%lu OK  (%lu fallos, %.1f%%)\n",
                     (unsigned long)(total_runs - total_fails), (unsigned long)total_runs,
                     (unsigned long)total_fails, 100.0f * total_fails / total_runs);
      Serial.printf("computo+polling: min=%.3fms max=%.3fms prom=%.3fms\n",
                     compute_us_min / 1000.0, compute_us_max / 1000.0,
                     (compute_us_sum / (double)total_runs) / 1000.0);
      Serial.println("========================================");
    } else {
      delay(200);
    }
  }
}
