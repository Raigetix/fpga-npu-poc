// sdram_p3_poc.ino -- Etapa 4, Fase 3+: NPU configurable por SPI de punta a
// punta (ver top_sdram_p3.v / mlp_engine_par_stream.v). Prueba de fondo:
// recorrer VARIOS modelos con formas bien distintas (cantidad de capas,
// anchos), reconfigurando la FPGA por SPI entre uno y otro SIN recompilar
// nada -- si los 5 dan resultados consistentes con el software, confirma
// que "configurable de punta a punta" es real, no solo para el modelo
// 130-128-64-32-5 de siempre.

#include <SPI.h>
#include <math.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP               0x00
#define CMD_SDRAM_SET_ADDR    0x01
#define CMD_SDRAM_WR          0x02
#define CMD_WSTREAM_SET_BASE  0x03
#define CMD_SET_NUM_LAYERS    0x04
#define CMD_SET_LAYER_SHAPE   0x05
#define CMD_SET_BIAS_ADDR     0x06
#define CMD_BIAS_WR           0x07
#define CMD_SET_QPARAM        0x08
#define CMD_START             0x09
#define CMD_DBG_RD            0x0A
#define CMD_SET_ITGT          0x0B
#define CMD_IBURST5           0x0C
#define CMD_SDRAM_RD          0x0D

#define DBG_SEL_OUT_MEM 5

#define NLANES 8
#define QSHIFT 20
// 0.02 (y no 0.001) para que las salidas queden en un rango dinamico real:
// con 0.001 todas colapsaban a 0/-1 por redondeo y el test casi no
// discriminaba -- de hecho ese regimen degenerado fue lo que enmascaro
// durante varias corridas el bug de la fusion del DSP (ver README).
#define REAL_MULTIPLIER 0.02
#define INPUT_ZP 5
static const int32_t MULT_INT = (int32_t)llround(REAL_MULTIPLIER * (double)(1LL << QSHIFT));
#define WSTREAM_SDRAM_BASE 0UL

// ---- 5 modelos con formas bien distintas: cantidad de capas y anchos
// variados, uno con entrada ancha (300, mas alla del limite de 256 que
// tienen las capas ocultas/salida) para probar esa asimetria a proposito.
#define MAX_MODEL_LAYERS 8
struct ModelDef { const char *name; int num_layers; int widths[MAX_MODEL_LAYERS + 1]; };
ModelDef MODELS[] = {
  {"Original 130-128-64-32-5", 4, {130, 128, 64, 32, 5}},
  {"Minimo 20-16-8 (2 capas)", 2, {20, 16, 8}},
  {"Medio 50-32-16-8-4",       4, {50, 32, 16, 8, 4}},
  {"Profundo 10-8-8-8-8-5 (5 capas)", 5, {10, 8, 8, 8, 8, 5}},
  {"Entrada ancha 300-200-64-10", 3, {300, 200, 64, 10}},
};
#define NUM_MODELS (sizeof(MODELS) / sizeof(MODELS[0]))

#define MAX_TOTAL_WEIGHTS 100000
#define MAX_TOTAL_BIAS    2000
#define MAX_LAYER_WIDTH   256   // limite real del hardware para capas ocultas/salida

struct LayerCfg { int in_count, out_count, waves, flat_base, lane_base, bias_base; };
LayerCfg cur_layers[MAX_MODEL_LAYERS];
int cur_num_layers, cur_w_total, cur_b_total, cur_lane_bank_size, cur_in_count, cur_out_count;

int8_t  weights[MAX_TOTAL_WEIGHTS];
int8_t  biases_raw[MAX_TOTAL_BIAS];
int32_t biases_folded[MAX_TOTAL_BIAS];

SPIClass fpga_spi(FSPI);
#define SPI_HZ 8000000
void spi_burst_begin() { fpga_spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0)); }
void spi_burst_end()   { fpga_spi.endTransaction(); }

void npu_xfer(uint8_t cmd, uint16_t a, uint16_t b_raw, uint8_t rx[6]) {
  uint8_t tx[6] = {cmd, (uint8_t)(a >> 8), (uint8_t)(a & 0xFF), (uint8_t)(b_raw >> 8), (uint8_t)(b_raw & 0xFF), 0x00};
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}
void npu_xfer_wide(uint8_t cmd, int32_t value, uint8_t tail, uint8_t rx[6]) {
  uint32_t v = (uint32_t)value;
  uint8_t tx[6] = {cmd, (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v, tail};
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
  spi_burst_begin(); npu_xfer(cmd, a, b_raw, rx); spi_burst_end();
}
void npu_send_wide(uint8_t cmd, int32_t value, uint8_t tail) {
  uint8_t rx[6];
  spi_burst_begin(); npu_xfer_wide(cmd, value, tail, rx); spi_burst_end();
}

void wait_pll_lock() {
  uint8_t rx[6];
  bool locked = false;
  spi_burst_begin();
  for (int i = 0; i < 2000 && !locked; i++) {
    npu_xfer(CMD_NOP, 0, 0, rx);
    locked = (rx[0] & 0x02) != 0;
    if (!locked) delayMicroseconds(100);
  }
  spi_burst_end();
  Serial.printf("PLL lock: %s\n", locked ? "OK" : "NUNCA");
}

void sdram_set_addr(uint32_t addr23) {
  npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(addr23 & 0xFFFF), (uint16_t)((addr23 >> 16) & 0x7F));
}

// Calcula in_count/out_count/waves/flat_base/lane_base/bias_base de cada
// capa a partir de la lista de anchos del modelo -- EQUIVALENTE a lo que
// antes eran los #define/LayerCfg fijos, ahora en runtime para cualquier
// forma.
void setup_layers(ModelDef &m) {
  cur_num_layers = m.num_layers;
  cur_in_count    = m.widths[0];
  cur_out_count   = m.widths[m.num_layers];
  int flat_base = 0, lane_base = 0, bias_base = 0;
  for (int L = 0; L < m.num_layers; L++) {
    int in_c = m.widths[L], out_c = m.widths[L + 1];
    int waves = (out_c + NLANES - 1) / NLANES;
    cur_layers[L] = {in_c, out_c, waves, flat_base, lane_base, bias_base};
    flat_base += in_c * out_c;
    lane_base += waves * in_c;
    bias_base += out_c;
  }
  cur_w_total = flat_base;
  cur_b_total = bias_base;
  cur_lane_bank_size = lane_base;
}

int8_t weight_at_local_addr(int lane, int local_addr) {
  for (int L = 0; L < cur_num_layers; L++) {
    LayerCfg &c = cur_layers[L];
    int span = c.waves * c.in_count;
    if (local_addr >= c.lane_base && local_addr < c.lane_base + span) {
      int rel = local_addr - c.lane_base;
      int w = rel / c.in_count, i = rel % c.in_count;
      int neuron = w * NLANES + lane;
      return (neuron < c.out_count) ? weights[c.flat_base + neuron * c.in_count + i] : 0;
    }
  }
  return 0;
}

// La escritura a SDRAM falla ~0.004% de los bytes (1 de cada ~27000), y UN
// solo peso corrupto arruina todas las inferencias del modelo -- por eso se
// verifica y se reintenta (converge al primer reintento). Mismo patron que
// la etapa 3 ya usaba para la carga a BRAM.
#define MAX_BAD 4096
uint32_t bad_addr[MAX_BAD];

static inline int8_t stream_byte_at(uint32_t idx) {
  return weight_at_local_addr((int)(idx % NLANES), (int)(idx / NLANES));
}

// Lectura suelta de una direccion (para confirmar sospechosos).
int8_t sdram_read_at(uint32_t addr) {
  uint8_t rx[6];
  sdram_set_addr(WSTREAM_SDRAM_BASE + addr);
  spi_burst_begin();
  npu_xfer(CMD_SDRAM_RD, 0, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  return (int8_t)rx[1];
}

// La lectura por SPI tiene ~0.15% de error por byte (la misma que se midio
// en la fase 1 y que el streaming esquiva con votacion por mayoria). Si se
// tomara cada diferencia como buena, en 27000 bytes aparecerian ~40
// "errores" que en realidad no existen. Por eso el barrido rapido solo
// junta SOSPECHOSOS y despues cada uno se confirma con relecturas.
uint32_t verify_weights(uint32_t total) {
  uint8_t rx[6];
  static uint32_t suspect[MAX_BAD];
  uint32_t n_susp = 0;

  sdram_set_addr(WSTREAM_SDRAM_BASE);
  spi_burst_begin();
  npu_xfer(CMD_SDRAM_RD, 0, 0, rx);          // pedido del byte 0
  for (uint32_t i = 0; i < total; i++) {
    npu_xfer(CMD_SDRAM_RD, 0, 0, rx);        // pide i+1, trae i
    if ((int8_t)rx[1] != stream_byte_at(i)) {
      if (n_susp < MAX_BAD) suspect[n_susp] = i;
      n_susp++;
    }
  }
  spi_burst_end();

  uint32_t bad = 0;
  uint32_t n_check = (n_susp < MAX_BAD) ? n_susp : MAX_BAD;
  for (uint32_t k = 0; k < n_check; k++) {
    uint32_t a = suspect[k];
    int8_t want = stream_byte_at(a);
    // dos relecturas: si alguna coincide, era un error de lectura, no un
    // peso realmente mal escrito
    if (sdram_read_at(a) == want || sdram_read_at(a) == want) continue;
    if (bad < MAX_BAD) bad_addr[bad] = a;
    bad++;
  }
  return bad;
}

void write_weights_to_sdram() {
  uint32_t total = (uint32_t)cur_lane_bank_size * NLANES;
  sdram_set_addr(WSTREAM_SDRAM_BASE);
  spi_burst_begin();
  uint8_t rx[6];
  for (int a = 0; a < cur_lane_bank_size; a++)
    for (int lane = 0; lane < NLANES; lane++)
      npu_xfer(CMD_SDRAM_WR, 0, (uint16_t)(uint8_t)weight_at_local_addr(lane, a), rx);
  spi_burst_end();

  for (int attempt = 1; attempt <= 6; attempt++) {
    uint32_t bad = verify_weights(total);
    if (bad == 0) {
      if (attempt > 1) Serial.printf("  (pesos OK despues de %d reintento(s))\n", attempt - 1);
      return;
    }
    Serial.printf("  intento %d: %lu byte(s) mal en SDRAM, reescribiendo...\n", attempt, (unsigned long)bad);
    uint32_t n_fix = (bad < MAX_BAD) ? bad : MAX_BAD;
    for (uint32_t k = 0; k < n_fix; k++) {
      sdram_set_addr(WSTREAM_SDRAM_BASE + bad_addr[k]);
      spi_burst_begin();
      npu_xfer(CMD_SDRAM_WR, 0, (uint16_t)(uint8_t)stream_byte_at(bad_addr[k]), rx);
      spi_burst_end();
    }
  }
  Serial.println("  AVISO: quedan bytes mal despues de 6 intentos");
}

void configure_model_shape() {
  npu_send(CMD_SET_NUM_LAYERS, cur_num_layers, 0);
  for (int L = 0; L < cur_num_layers; L++) {
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((0 << 4) | L), (uint16_t)cur_layers[L].in_count);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((1 << 4) | L), (uint16_t)cur_layers[L].out_count);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((2 << 4) | L), (uint16_t)cur_layers[L].bias_base);
  }
}

void compute_folded_biases() {
  for (int L = 0; L < cur_num_layers; L++) {
    LayerCfg &c = cur_layers[L];
    int32_t input_zp = (L == 0) ? INPUT_ZP : 0;
    for (int n = 0; n < c.out_count; n++) {
      int32_t wsum = 0;
      for (int i = 0; i < c.in_count; i++) wsum += (int32_t)weights[c.flat_base + n * c.in_count + i];
      biases_folded[c.bias_base + n] = (int32_t)biases_raw[c.bias_base + n] - input_zp * wsum;
    }
  }
}

void load_biases() {
  npu_send(CMD_SET_BIAS_ADDR, 0, 0);
  spi_burst_begin();
  uint8_t rx[6];
  for (int i = 0; i < cur_b_total; i++) npu_xfer_wide(CMD_BIAS_WR, biases_folded[i], 0, rx);
  spi_burst_end();
}

void load_qparams() {
  for (int L = 0; L < cur_num_layers; L++) {
    uint8_t layer = (uint8_t)L;
    npu_send_wide(CMD_SET_QPARAM, MULT_INT, (layer << 2) | 0);
    npu_send_wide(CMD_SET_QPARAM, 0,        (layer << 2) | 1);
    int32_t act_min = (L < cur_num_layers - 1) ? 0 : -128;
    npu_send_wide(CMD_SET_QPARAM, act_min, (layer << 2) | 2);
    npu_send_wide(CMD_SET_QPARAM, 127,     (layer << 2) | 3);
  }
}

int8_t clamp_i32(int64_t v, int32_t lo, int32_t hi) {
  if (v < lo) return (int8_t)lo;
  if (v > hi) return (int8_t)hi;
  return (int8_t)v;
}

// Forward pass generico (cantidad de capas variable), ping-pong entre dos
// buffers de activacion -- igual esquema que el hardware.
void mlp_forward_sw(const int8_t *input, int8_t *output) {
  static int8_t actA[MAX_LAYER_WIDTH], actB[MAX_LAYER_WIDTH];
  const int8_t *cur_in = input;
  for (int L = 0; L < cur_num_layers; L++) {
    LayerCfg &c = cur_layers[L];
    bool is_last = (L == cur_num_layers - 1);
    int8_t *dst = is_last ? output : ((L % 2 == 0) ? actA : actB);
    int32_t act_min = is_last ? -128 : 0;
    for (int n = 0; n < c.out_count; n++) {
      int32_t acc = 0;
      const int8_t *w = &weights[c.flat_base + n * c.in_count];
      for (int i = 0; i < c.in_count; i++) acc += (int32_t)w[i] * (int32_t)cur_in[i];
      int64_t biased = (int64_t)acc + (int64_t)biases_folded[c.bias_base + n];
      int64_t scaled = (biased * (int64_t)MULT_INT) >> QSHIFT;
      dst[n] = clamp_i32(scaled, act_min, 127);
    }
    cur_in = dst;
  }
}

int8_t npu_dbg_read(uint8_t sel, uint16_t addr) {
  uint8_t rx[6];
  uint16_t a = ((uint16_t)(sel & 0x7) << 11) | (addr & 0x7FF);
  spi_burst_begin();
  npu_xfer(CMD_DBG_RD, a, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  return (int8_t)rx[5];
}

void npu_wait_done() {
  uint8_t rx[6];
  spi_burst_begin();
  bool busy = true;
  while (busy) { npu_xfer(CMD_NOP, 0, 0, rx); busy = (rx[0] & 0x01) != 0; }
  spi_burst_end();
}

void run_inference_once(const int8_t *input, int8_t *got, uint32_t *t_load, uint32_t *t_compute) {
  uint8_t rx[6];
  uint32_t t0 = micros();
  npu_send(CMD_SET_ITGT, 0, 0);
  uint8_t buf[5]; int bufcount = 0;
  spi_burst_begin();
  for (int i = 0; i < cur_in_count; i++) {
    buf[bufcount++] = (uint8_t)input[i];
    if (bufcount == 5) { npu_xfer_burst(CMD_IBURST5, buf[0], buf[1], buf[2], buf[3], buf[4], rx); bufcount = 0; }
  }
  while (bufcount != 0) { // rafaga parcial: relleno con ceros (mismo bug/arreglo de Etapa 3)
    buf[bufcount++] = 0;
    if (bufcount == 5) { npu_xfer_burst(CMD_IBURST5, buf[0], buf[1], buf[2], buf[3], buf[4], rx); bufcount = 0; }
  }
  npu_xfer(CMD_START, 0, 0, rx);
  spi_burst_end();
  uint32_t t_loaded = micros();
  npu_wait_done();
  uint32_t t1 = micros();
  for (int i = 0; i < cur_out_count; i++) got[i] = npu_dbg_read(DBG_SEL_OUT_MEM, i);
  *t_load = t_loaded - t0;
  *t_compute = t1 - t_loaded;
}

void run_model_test(ModelDef &m, int n_iters) {
  Serial.printf("\n=== Modelo: %s ===\n", m.name);
  setup_layers(m);
  Serial.printf("  %d capas, entrada=%d salida=%d, %d pesos, %d bias\n",
                cur_num_layers, cur_in_count, cur_out_count, cur_w_total, cur_b_total);

  randomSeed(1000 + (int)(intptr_t)&m); // semilla distinta por modelo, reproducible
  for (int i = 0; i < cur_w_total; i++) weights[i]      = (int8_t)random(-40, 41);
  for (int i = 0; i < cur_b_total; i++) biases_raw[i]   = (int8_t)random(-10, 11);
  compute_folded_biases();

  uint32_t t0 = micros();
  configure_model_shape();
  write_weights_to_sdram();
  npu_send(CMD_WSTREAM_SET_BASE, (uint16_t)(WSTREAM_SDRAM_BASE & 0xFFFF), (uint16_t)((WSTREAM_SDRAM_BASE >> 16) & 0x7F));
  load_biases();
  load_qparams();
  Serial.printf("  Carga completa en %.2f ms\n", (micros() - t0) / 1000.0);

  int8_t input[1024], expected[MAX_LAYER_WIDTH], got[MAX_LAYER_WIDTH];
  int fails = 0;
  uint32_t compute_sum = 0;
  for (int iter = 0; iter < n_iters; iter++) {
    for (int i = 0; i < cur_in_count; i++) input[i] = (int8_t)random(-128, 128);
    mlp_forward_sw(input, expected);
    uint32_t t_load, t_compute;
    run_inference_once(input, got, &t_load, &t_compute);
    compute_sum += t_compute;
    bool match = true;
    for (int i = 0; i < cur_out_count; i++) if (expected[i] != got[i]) match = false;
    if (!match) {
      fails++;
      Serial.printf("  [%2d] ERROR -- sw=[", iter);
      for (int i = 0; i < cur_out_count; i++) Serial.printf("%d ", expected[i]);
      Serial.printf("] fpga=[");
      for (int i = 0; i < cur_out_count; i++) Serial.printf("%d ", got[i]);
      Serial.printf("]\n");
    }
  }
  Serial.printf("  Resultado: %d/%d OK  (computo prom: %.3fms)\n", n_iters - fails, n_iters, (compute_sum / (double)n_iters) / 1000.0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: Etapa 4 Fase 3+ -- 5 modelos distintos, misma FPGA sin recompilar ===");
  wait_pll_lock();

  for (unsigned m = 0; m < NUM_MODELS; m++) run_model_test(MODELS[m], 20);

  Serial.println("\n=== Prueba completa ===");
}

void loop() {}
