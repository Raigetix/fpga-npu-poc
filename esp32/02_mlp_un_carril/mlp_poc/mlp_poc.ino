// mlp_poc.ino -- ESP32-S3 como maestro SPI de la red feedforward de 4 capas
// (130 -> 128 -> 64 -> 32 -> 5) implementada en fpga_NPU_poc/top_mlp.v.
// Genera pesos/bias "aleatorios con sentido", los carga en la FPGA por SPI,
// y despues, en un bucle: genera un vector de entrada dummy (simulando
// MFCC), calcula el resultado esperado EN SOFTWARE (mismo algoritmo que la
// FPGA: acumulador int32, bias, ReLU, saturacion a int8), mide cuanto tarda
// ese calculo, se lo manda a la FPGA, mide el tiempo de ida y vuelta, y
// compara ambos resultados y tiempos.
//
// Conexionado: igual que npu_poc.ino (mismos 4 pines + GND, ver ese sketch).
// IMPORTANTE: este sketch asume que la FPGA tiene cargado 'top_mlp_fast'
// (logica interna a 108MHz via PLL; no 'top_mlp' a 27MHz ni 'top', que es
// el demo simple de producto punto).

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP    0x00
#define CMD_LOAD_W 0x01
#define CMD_LOAD_B 0x02
#define CMD_LOAD_I 0x03
#define CMD_START  0x04
#define CMD_DBG_RD 0x06

// ---- Dimensiones de la red: deben coincidir EXACTO con mlp_engine.v ----
#define IN_COUNT  130
#define H1_COUNT  128
#define H2_COUNT  64
#define H3_COUNT  32
#define OUT_COUNT 5

// ---- Mapa de direcciones de pesos (igual al comentario de mlp_engine.v) ----
#define W1_BASE 0
#define W1_SIZE (IN_COUNT * H1_COUNT)
#define W2_BASE (W1_BASE + W1_SIZE)
#define W2_SIZE (H1_COUNT * H2_COUNT)
#define W3_BASE (W2_BASE + W2_SIZE)
#define W3_SIZE (H2_COUNT * H3_COUNT)
#define W4_BASE (W3_BASE + W3_SIZE)
#define W4_SIZE (H3_COUNT * OUT_COUNT)
#define W_TOTAL (W4_BASE + W4_SIZE)

// ---- Mapa de direcciones de bias ----
#define B1_BASE 0
#define B2_BASE (B1_BASE + H1_COUNT)
#define B3_BASE (B2_BASE + H2_COUNT)
#define B4_BASE (B3_BASE + H3_COUNT)
#define B_TOTAL (B4_BASE + OUT_COUNT)

SPIClass fpga_spi(FSPI);

// Copia local de pesos/bias: nos hace falta para poder calcular el
// "esperado" en software, ademas de para cargarlos en la FPGA.
int8_t weights[W_TOTAL];
int8_t biases[B_TOTAL];

#define SPI_HZ 8000000

// La transaccion SPI (beginTransaction/endTransaction) tiene overhead de
// mutex/reconfiguracion en el driver de Arduino -- medido en ~46us por
// llamada, mas que el propio tiempo de transferencia de bits. Por eso las
// funciones "_burst" abren la transaccion UNA vez para muchos frames
// seguidos (solo alternan CS), en vez de una vez por frame.
void spi_burst_begin() { fpga_spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0)); }
void spi_burst_end()   { fpga_spi.endTransaction(); }

// Transfiere UN frame de 6 bytes; asume que ya se llamo spi_burst_begin().
void npu_xfer(uint8_t cmd, uint16_t a, uint16_t b_raw, uint8_t rx[6]) {
  uint8_t tx[6] = {
    cmd,
    (uint8_t)(a >> 8), (uint8_t)(a & 0xFF),
    (uint8_t)(b_raw >> 8), (uint8_t)(b_raw & 0xFF),
    0x00
  };
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}

// Version "de a uno": para llamadas sueltas donde no vale la pena batchear.
void npu_send(uint8_t cmd, uint16_t a, uint16_t b_raw) {
  uint8_t rx[6];
  spi_burst_begin();
  npu_xfer(cmd, a, b_raw, rx);
  spi_burst_end();
}

// Lee una neurona de una capa oculta (sel: 0=h1 1=h2 2=h3), solo tiene
// sentido llamarlo con busy=0 (entre inferencias).
int8_t npu_dbg_read(uint8_t sel, uint8_t addr) {
  uint8_t rx[6];
  uint16_t a = ((uint16_t)sel << 8) | addr;
  spi_burst_begin();
  npu_xfer(CMD_DBG_RD, a, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx); // margen extra de latencia (mismo patron de antes)
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  return (int8_t)rx[5];
}

void npu_read_status(bool *busy, int8_t out[OUT_COUNT]) {
  uint8_t rx[6];
  spi_burst_begin();
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  *busy  = (rx[0] & 0x01) != 0;
  out[0] = (int8_t)rx[1];
  out[1] = (int8_t)rx[2];
  out[2] = (int8_t)rx[3];
  out[3] = (int8_t)rx[4];
  out[4] = (int8_t)rx[5];
}

// Version rapida del polling: mantiene la transaccion SPI abierta durante
// TODO el bucle de espera, en vez de abrir/cerrar en cada intento.
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
  npu_xfer(CMD_NOP, 0, 0, rx); // 1 lectura extra, mismo margen de siempre
  spi_burst_end();
  out[0] = (int8_t)rx[1];
  out[1] = (int8_t)rx[2];
  out[2] = (int8_t)rx[3];
  out[3] = (int8_t)rx[4];
  out[4] = (int8_t)rx[5];
  *polls_out = polls;
}

// Mismo post-procesamiento que mlp_engine.v: ReLU + saturacion a [0,127].
int8_t relu_sat(int32_t v) {
  if (v < 0) return 0;
  if (v > 127) return 127;
  return (int8_t)v;
}

// Capas ocultas del ULTIMO forward pass en software: globales (no locales)
// para poder compararlas byte a byte contra lo que devuelve la FPGA cuando
// la salida final no coincide.
int8_t h1[H1_COUNT], h2[H2_COUNT], h3[H3_COUNT];

// Recorre las 4 capas en software, con el MISMO algoritmo que la FPGA
// (acumulador int32, bias, ReLU+saturacion), para poder comparar resultados.
void mlp_forward_sw(const int8_t *input, int8_t *output) {
  for (int n = 0; n < H1_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < IN_COUNT; i++)
      acc += (int32_t)weights[W1_BASE + n * IN_COUNT + i] * (int32_t)input[i];
    acc += biases[B1_BASE + n];
    h1[n] = relu_sat(acc);
  }
  for (int n = 0; n < H2_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < H1_COUNT; i++)
      acc += (int32_t)weights[W2_BASE + n * H1_COUNT + i] * (int32_t)h1[i];
    acc += biases[B2_BASE + n];
    h2[n] = relu_sat(acc);
  }
  for (int n = 0; n < H3_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < H2_COUNT; i++)
      acc += (int32_t)weights[W3_BASE + n * H2_COUNT + i] * (int32_t)h2[i];
    acc += biases[B3_BASE + n];
    h3[n] = relu_sat(acc);
  }
  for (int n = 0; n < OUT_COUNT; n++) {
    int32_t acc = 0;
    for (int i = 0; i < H3_COUNT; i++)
      acc += (int32_t)weights[W4_BASE + n * H3_COUNT + i] * (int32_t)h3[i];
    acc += biases[B4_BASE + n];
    output[n] = relu_sat(acc);
  }
}

void load_weights_and_biases() {
  Serial.println("Generando pesos/bias aleatorios (rango -2..2)...");
  randomSeed(42); // reproducible entre corridas

  for (int i = 0; i < W_TOTAL; i++) weights[i] = (int8_t)random(-2, 3);
  for (int i = 0; i < B_TOTAL; i++) biases[i]  = (int8_t)random(-2, 3);

  Serial.printf("Cargando %d pesos y %d bias en la FPGA por SPI...\n", W_TOTAL, B_TOTAL);
  uint32_t t0 = micros();

  uint8_t rx[6];
  spi_burst_begin();
  for (int i = 0; i < W_TOTAL; i++)
    npu_xfer(CMD_LOAD_W, (uint16_t)i, (uint16_t)(uint8_t)weights[i], rx);
  for (int i = 0; i < B_TOTAL; i++)
    npu_xfer(CMD_LOAD_B, (uint16_t)i, (uint16_t)(uint8_t)biases[i], rx);
  spi_burst_end();

  uint32_t t1 = micros();
  Serial.printf("Carga completa en %.2f ms\n\n", (t1 - t0) / 1000.0);
}

// Corre UNA inferencia con la entrada dada (ya fijada por el llamador),
// devuelve el resultado de la FPGA y datos de tiempo. No genera nada random.
void run_inference_once(const int8_t *mfcc, int8_t *got, uint32_t *t_load, uint32_t *t_compute, int *polls_out) {
  uint8_t rx[6];
  uint32_t fpga_t0 = micros();
  spi_burst_begin();
  for (int i = 0; i < IN_COUNT; i++)
    npu_xfer(CMD_LOAD_I, (uint16_t)i, (uint16_t)(uint8_t)mfcc[i], rx);
  npu_xfer(CMD_START, 0, 0, rx);
  spi_burst_end();
  uint32_t fpga_t_loaded = micros();

  npu_wait_done(got, polls_out);
  uint32_t fpga_t1 = micros();

  *t_load    = fpga_t_loaded - fpga_t0;
  *t_compute = fpga_t1 - fpga_t_loaded;
}

// Lee h1/h2/h3 de la FPGA (con busy ya en 0) y los compara contra los que
// quedaron en las globales h1/h2/h3 del ULTIMO mlp_forward_sw(). Imprime la
// PRIMERA capa y neurona donde aparece la primera diferencia.
void debug_compare_layers() {
  Serial.println("  Comparando capas ocultas h1/h2/h3 (FPGA vs software)...");
  int mism1 = 0, mism2 = 0, mism3 = 0;
  int first_layer = -1, first_idx = -1;

  for (int i = 0; i < H1_COUNT; i++) {
    int8_t got = npu_dbg_read(0, i);
    if (got != h1[i]) {
      mism1++;
      if (first_layer < 0) { first_layer = 1; first_idx = i; }
      if (mism1 <= 3) Serial.printf("    h1[%3d]: esperado=%4d  fpga=%4d\n", i, h1[i], got);
    }
  }
  for (int i = 0; i < H2_COUNT; i++) {
    int8_t got = npu_dbg_read(1, i);
    if (got != h2[i]) {
      mism2++;
      if (first_layer < 0) { first_layer = 2; first_idx = i; }
      if (mism2 <= 3) Serial.printf("    h2[%3d]: esperado=%4d  fpga=%4d\n", i, h2[i], got);
    }
  }
  for (int i = 0; i < H3_COUNT; i++) {
    int8_t got = npu_dbg_read(2, i);
    if (got != h3[i]) {
      mism3++;
      if (first_layer < 0) { first_layer = 3; first_idx = i; }
      if (mism3 <= 3) Serial.printf("    h3[%3d]: esperado=%4d  fpga=%4d\n", i, h3[i], got);
    }
  }

  Serial.printf("  Diferencias: h1=%d/%d  h2=%d/%d  h3=%d/%d\n",
                mism1, H1_COUNT, mism2, H2_COUNT, mism3, H3_COUNT);
  if (first_layer >= 0)
    Serial.printf("  Primera diferencia: capa h%d, indice %d\n", first_layer, first_idx);
  else
    Serial.println("  h1/h2/h3 coinciden perfecto (la diferencia nace en la capa de salida)");
  Serial.println();
}

// Contadores acumulados de toda la sesion, para no tener que contar a mano
// scrolleando el log.
uint32_t total_runs = 0;
uint32_t total_fails = 0;

void run_inference_test(int iter) {
  int8_t mfcc[IN_COUNT];
  for (int i = 0; i < IN_COUNT; i++) mfcc[i] = (int8_t)random(-2, 3);

  int8_t expected[OUT_COUNT];
  uint32_t sw_t0 = micros();
  mlp_forward_sw(mfcc, expected);
  uint32_t sw_t1 = micros();

  uint8_t start_count_before = npu_dbg_read(3, 0);

  int8_t got[OUT_COUNT];
  uint32_t t_load, t_compute;
  int polls;
  run_inference_once(mfcc, got, &t_load, &t_compute, &polls);

  uint8_t start_count_after = npu_dbg_read(3, 0);
  uint8_t start_delta = start_count_after - start_count_before; // deberia ser SIEMPRE 1

  bool match = true;
  for (int i = 0; i < OUT_COUNT; i++) if (expected[i] != got[i]) match = false;

  total_runs++;
  if (!match) total_fails++;
  float fail_pct = 100.0f * total_fails / total_runs;

  Serial.printf("--- Iteracion %d ---  [fallo acumulado: %lu/%lu = %.1f%%]\n",
                iter, (unsigned long)total_fails, (unsigned long)total_runs, fail_pct);
  Serial.printf("ESP32 (software): %6.3f ms -> [%4d %4d %4d %4d %4d]\n",
                (sw_t1 - sw_t0) / 1000.0, expected[0], expected[1], expected[2], expected[3], expected[4]);
  Serial.printf("FPGA (SPI+infer): %6.3f ms -> [%4d %4d %4d %4d %4d]  (carga entrada: %.3f ms, computo+polling: %.3f ms, %d polls, start_count delta=%u)\n",
                (t_load + t_compute) / 1000.0, got[0], got[1], got[2], got[3], got[4],
                t_load / 1000.0, t_compute / 1000.0, polls, start_delta);
  Serial.println(match ? ">> OK: coinciden" : ">> ERROR: no coinciden");
  if (!match) debug_compare_layers();
  Serial.println();
}

// Corre la MISMA entrada fija muchas veces seguidas, para distinguir un bug
// de logica determinista (siempre falla igual) de un problema de hardware
// intermitente (a veces bien, a veces mal, y con valores distintos cada vez).
void run_repeatability_test(int n) {
  int8_t mfcc[IN_COUNT];
  for (int i = 0; i < IN_COUNT; i++) mfcc[i] = (int8_t)(((i * 37) % 5) - 2); // fija, no random

  int8_t expected[OUT_COUNT];
  mlp_forward_sw(mfcc, expected);

  Serial.println("=== Test de repetibilidad: misma entrada, muchas corridas ===");
  Serial.printf("Esperado: [%4d %4d %4d %4d %4d]\n\n", expected[0], expected[1], expected[2], expected[3], expected[4]);

  int ok_count = 0;
  for (int r = 0; r < n; r++) {
    int8_t got[OUT_COUNT];
    uint32_t t_load, t_compute;
    int polls;
    run_inference_once(mfcc, got, &t_load, &t_compute, &polls);

    bool match = true;
    for (int i = 0; i < OUT_COUNT; i++) if (expected[i] != got[i]) match = false;
    if (match) ok_count++;

    Serial.printf("corrida %2d: [%4d %4d %4d %4d %4d]  compute=%.3fms polls=%d  %s\n",
                  r, got[0], got[1], got[2], got[3], got[4], t_compute / 1000.0, polls,
                  match ? "OK" : "MISMATCH");
  }
  Serial.printf("\n%d/%d corridas correctas\n\n", ok_count, n);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: inferencia de red 130-128-64-32-5 via SPI ===\n");
  load_weights_and_biases();
}

int iteration = 0;

void loop() {
  run_inference_test(iteration++);
  delay(2000);
}
