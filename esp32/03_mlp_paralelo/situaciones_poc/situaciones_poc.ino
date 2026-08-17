// situaciones_poc.ino -- ESP32-S3 como maestro SPI de las 3 "situaciones"
// de arquitectura (ver README del proyecto, seccion "3 situaciones"):
//   MODEL 1: 8->12->4      (minimo, el ESP32 le gana a la FPGA por overhead fijo de SPI)
//   MODEL 2: 130->160->96->32->5  (~39.4K pesos, la FPGA gana con margen)
//   MODEL 3: 130->176->88->36->5  (~41.7K pesos, el mas grande que entra con margen seguro de BRAM)
//
// Un solo sketch, tres bitstreams distintos (top_mlp_par_s1/s2/s3.v):
// cambiar MODEL aca abajo, volver a subir el sketch, y reprogramar la FPGA
// con el top-level correspondiente (el codigo de PC se encarga de eso).
//
// Los 3 modelos comparten el MISMO protocolo/anchos de bus (a diferencia
// del top_mlp_par.v original de 12 bits): direccion de pesos 13 bits
// (carril en addr_field[15:13], direccion en addr_field[12:0]), direccion
// de bias/debug 9 bits (sel en addr_field[11:9], direccion en addr_field[8:0]).
// Conexionado SPI: igual que los otros sketches (ver npu_poc.ino).

#define MODEL 3  // <-- CAMBIAR ESTO (1, 2 o 3) Y VOLVER A SUBIR PARA CADA SITUACION

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

#define NLANES 8

#if MODEL == 1
  #define NUM_LAYERS 2
  static const int LAYER_SIZES[NUM_LAYERS + 1] = {8, 12, 4};
  #define MODEL_NAME "Situacion 1: 8->12->4 (minimo, ESP32 deberia ganar)"
#elif MODEL == 2
  #define NUM_LAYERS 4
  static const int LAYER_SIZES[NUM_LAYERS + 1] = {130, 160, 96, 32, 5};
  #define MODEL_NAME "Situacion 2: 130->160->96->32->5 (FPGA gana con margen)"
#elif MODEL == 3
  #define NUM_LAYERS 4
  static const int LAYER_SIZES[NUM_LAYERS + 1] = {130, 176, 88, 36, 5};
  #define MODEL_NAME "Situacion 3: 130->176->88->36->5 (el mas grande que entra)"
#else
  #error "MODEL debe ser 1, 2 o 3"
#endif

#define IN_COUNT  (LAYER_SIZES[0])
#define OUT_COUNT (LAYER_SIZES[NUM_LAYERS])
#define MAX_WIDTH 256   // ancho maximo de cualquier capa en cualquiera de los 3 modelos
#define MAX_W_TOTAL 45000 // cota superior de pesos totales entre los 3 modelos

struct LayerCfg { int in_count, out_count, waves, flat_base, lane_base, bias_base; };
LayerCfg LAYERS[NUM_LAYERS];
int W_TOTAL = 0, B_TOTAL = 0, LANE_BANK_SIZE = 0;

void setup_layers() {
  int flat_acc = 0, lane_acc = 0, bias_acc = 0;
  for (int i = 0; i < NUM_LAYERS; i++) {
    LAYERS[i].in_count  = LAYER_SIZES[i];
    LAYERS[i].out_count = LAYER_SIZES[i + 1];
    LAYERS[i].waves     = (LAYERS[i].out_count + 7) / 8;
    LAYERS[i].flat_base = flat_acc;
    LAYERS[i].lane_base = lane_acc;
    LAYERS[i].bias_base = bias_acc;
    flat_acc += LAYERS[i].in_count * LAYERS[i].out_count;
    lane_acc += LAYERS[i].waves * LAYERS[i].in_count;
    bias_acc += LAYERS[i].out_count;
  }
  W_TOTAL = flat_acc;
  B_TOTAL = bias_acc;
  LANE_BANK_SIZE = lane_acc;
}

SPIClass fpga_spi(FSPI);

int8_t weights[MAX_W_TOTAL];
int8_t biases[600];
int8_t stage_act[NUM_LAYERS + 1][MAX_WIDTH]; // [0]=entrada, [NUM_LAYERS]=salida

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

void npu_xfer_burst(uint8_t cmd, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t rx[6]) {
  uint8_t tx[6] = {cmd, b0, b1, b2, b3, b4};
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}

void npu_send(uint8_t cmd, uint16_t a, uint16_t b_raw) {
  uint8_t rx[6];
  spi_burst_begin();
  npu_xfer(cmd, a, b_raw, rx);
  spi_burst_end();
}

// Los 3 modelos comparten el mismo ancho de bus: sel en bit 9, direccion de
// 9 bits (a diferencia del top_mlp_par.v original, que usaba bit 8 / 8 bits).
int8_t npu_dbg_read(uint8_t sel, uint16_t addr) {
  uint8_t rx[6];
  uint16_t a = ((uint16_t)sel << 9) | (addr & 0x1FF);
  spi_burst_begin();
  npu_xfer(CMD_DBG_RD, a, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  return (int8_t)rx[5];
}

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

void npu_wait_done(int8_t out[], int *polls_out) {
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
  int8_t raw[5] = {(int8_t)rx[1], (int8_t)rx[2], (int8_t)rx[3], (int8_t)rx[4], (int8_t)rx[5]};
  for (int i = 0; i < OUT_COUNT && i < 5; i++) out[i] = raw[i];
  *polls_out = polls;
}

int8_t relu_sat(int32_t v) {
  if (v < 0) return 0;
  if (v > 127) return 127;
  return (int8_t)v;
}

void mlp_forward_sw(const int8_t *input, int8_t *output) {
  for (int i = 0; i < IN_COUNT; i++) stage_act[0][i] = input[i];
  for (int L = 0; L < NUM_LAYERS; L++) {
    LayerCfg &c = LAYERS[L];
    for (int n = 0; n < c.out_count; n++) {
      int32_t acc = 0;
      for (int i = 0; i < c.in_count; i++)
        acc += (int32_t)weights[c.flat_base + n * c.in_count + i] * (int32_t)stage_act[L][i];
      acc += biases[c.bias_base + n];
      stage_act[L + 1][n] = relu_sat(acc);
    }
  }
  for (int i = 0; i < OUT_COUNT; i++) output[i] = stage_act[NUM_LAYERS][i];
}

// ---- Carga de los pesos de UN carril, un peso por transaccion SPI ----
// Bus de 13 bits: carril=addr_field[15:13], direccion local=addr_field[12:0].
void send_lane_weights(int lane) {
  uint8_t rx[6];
  spi_burst_begin();
  for (int L = 0; L < NUM_LAYERS; L++) {
    LayerCfg &c = LAYERS[L];
    for (int w = 0; w < c.waves; w++) {
      int neuron = w * NLANES + lane;
      for (int i = 0; i < c.in_count; i++) {
        int8_t val = (neuron < c.out_count) ? weights[c.flat_base + neuron * c.in_count + i] : 0;
        int local_addr = c.lane_base + w * c.in_count + i;
        uint16_t a = ((uint16_t)lane << 13) | (uint16_t)local_addr;
        npu_xfer(CMD_LOAD_W, a, (uint16_t)(uint8_t)val, rx);
      }
    }
  }
  spi_burst_end();
}

int8_t weight_at_local_addr(int lane, int local_addr) {
  for (int L = 0; L < NUM_LAYERS; L++) {
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

int verify_lane_weights(int lane, int *mismatch_addr, int max_mismatch) {
  int count = 0;
  for (int addr = 0; addr < LANE_BANK_SIZE; addr++) {
    npu_send(CMD_SET_WTGT, ((uint16_t)lane << 13) | (uint16_t)addr, 0);
    int8_t got = npu_dbg_read(5, 0);
    int8_t expected = weight_at_local_addr(lane, addr);
    if (got != expected) {
      if (count < max_mismatch) mismatch_addr[count] = addr;
      count++;
    }
  }
  return count;
}

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
      uint16_t a = ((uint16_t)lane << 13) | (uint16_t)addr;
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
  // relleno final: si IN_COUNT no es multiplo de 5 (ej. situacion 1, 8
  // entradas), sin esto el ultimo grupo parcial nunca se manda.
  while (bufcount != 0) {
    buf[bufcount++] = 0;
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
  while (bufcount != 0) {
    buf[bufcount++] = 0;
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
  Serial.printf("ESP32 (software): %6.3f ms -> [", (sw_t1 - sw_t0) / 1000.0);
  for (int i = 0; i < OUT_COUNT; i++) Serial.printf("%4d ", expected[i]);
  Serial.println("]");
  Serial.printf("FPGA (SPI+infer): %6.3f ms -> [", (t_load + t_compute) / 1000.0);
  for (int i = 0; i < OUT_COUNT; i++) Serial.printf("%4d ", got[i]);
  Serial.printf("]  (carga entrada: %.3f ms, computo+polling: %.3f ms, %d polls)\n",
                t_load / 1000.0, t_compute / 1000.0, polls);
  Serial.println(match ? ">> OK: coinciden" : ">> ERROR: no coinciden");
  Serial.println();
}

#define N_TEST_ITERS 50
int iteration = 0;
bool test_done = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  setup_layers();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: comparacion de arquitecturas ===");
  Serial.println(MODEL_NAME);
  Serial.printf("W_TOTAL=%d B_TOTAL=%d LANE_BANK_SIZE(direcciones/carril)=%d\n\n", W_TOTAL, B_TOTAL, LANE_BANK_SIZE);

  wait_pll_lock();
  load_weights_and_biases();
  test_input_burst_loading();
  test_bias_readback();
}

void loop() {
  if (!test_done) {
    run_inference_test(iteration++);
    if (iteration >= N_TEST_ITERS) {
      test_done = true;
      Serial.println("========================================");
      Serial.println(MODEL_NAME);
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
