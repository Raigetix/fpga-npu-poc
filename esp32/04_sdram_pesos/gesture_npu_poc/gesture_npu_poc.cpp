// gesture_npu_poc.cpp -- Etapa 4: CLASIFICADOR DE GESTOS ("varita magica":
// mover el celular haciendo una figura en el aire) en la NPU.
//
// Mismo espiritu que kws_npu_poc.cpp/mnist_npu_poc.ino (esta etapa 4 en
// general): la NPU de la FPGA es generica (configurable por SPI), lo unico
// que cambia entre demos es COMO se arma el vector de entrada. Para
// palabras clave son MFCC de audio; para gestos es la secuencia de
// acelerometro+giroscopio del navegador (sensor de movimiento del celular)
// remuestreada a un largo fijo -- ver tools/train_gestures_npu.py, que
// hace EXACTAMENTE el mismo remuestreo del lado de Python para entrenar.
//
// El modelo (data/model.bin, formato "NPUG") y la pagina de demo
// (data/index.html) viven en LittleFS, igual que los otros demos de esta
// etapa -- se puede cambiar de modelo subiendo otro model.bin sin
// recompilar.
//
// Para subir:  pio run -t uploadfs   (los archivos)
//              pio run -t upload     (el firmware)
//
// EL SENSOR DE MOVIMIENTO NECESITA HTTPS: los navegadores lo bloquean en
// paginas http://. Chrome (PC y Android): chrome://flags -> "Insecure
// origins treated as secure" -> agregar http://<ip> -> reiniciar.

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
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
#define WSTREAM_SDRAM_BASE 0UL
#define MAX_LAYERS 8
#define MAX_GESTURES 8
#define MAX_RAW_SAMPLES 400   // graba ~90-100 a 60Hz en 1.5s -- de sobra

// ================= WiFi =================
const char *WIFI_SSID = "Fibertel WiFi311 2.4GHz";
const char *WIFI_PASS = "00417751437";
const char *AP_SSID = "NPU-GESTOS";
const char *AP_PASS = "npu12345";

SPIClass fpga_spi(FSPI);
WebServer server(80);

// ================= Modelo cargado desde flash =================
struct LayerCfg { uint16_t in_n, out_n; int32_t mult, zp_out, act_min, act_max; };

struct Model {
  uint16_t num_layers, n_features, num_outputs;
  float    input_scale;
  int32_t  input_zp, qshift;
  LayerCfg layer[MAX_LAYERS];
  int32_t *bias;      uint32_t n_bias;
  int8_t  *weights;   uint32_t n_weights;
  uint16_t n_timesteps, n_channels;
  float    accel_scale, gyro_scale;
  char     gesture[MAX_GESTURES][16];
  bool     ok;
} M;

static void *alloc_big(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (!p) p = malloc(n);
  return p;
}

template <typename T> static bool rd(File &f, T &v) {
  return f.read((uint8_t *)&v, sizeof(T)) == sizeof(T);
}

bool load_model(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) { Serial.printf("ERROR: no se encuentra %s\n", path); return false; }

  char magic[4];
  if (f.read((uint8_t *)magic, 4) != 4 || memcmp(magic, "NPUG", 4)) {
    Serial.println("ERROR: model.bin no tiene el formato esperado (NPUG)"); f.close(); return false;
  }
  rd(f, M.num_layers); rd(f, M.n_features); rd(f, M.num_outputs);
  rd(f, M.input_scale); rd(f, M.input_zp); rd(f, M.qshift);
  if (M.num_layers > MAX_LAYERS) { Serial.println("ERROR: demasiadas capas"); f.close(); return false; }
  for (int i = 0; i < M.num_layers; i++) {
    rd(f, M.layer[i].in_n);    rd(f, M.layer[i].out_n);
    rd(f, M.layer[i].mult);    rd(f, M.layer[i].zp_out);
    rd(f, M.layer[i].act_min); rd(f, M.layer[i].act_max);
  }
  rd(f, M.n_bias);
  M.bias = (int32_t *)alloc_big(M.n_bias * sizeof(int32_t));
  if (!M.bias) { Serial.println("ERROR: sin memoria para los bias"); f.close(); return false; }
  f.read((uint8_t *)M.bias, M.n_bias * sizeof(int32_t));

  rd(f, M.n_weights);
  M.weights = (int8_t *)alloc_big(M.n_weights);
  if (!M.weights) { Serial.println("ERROR: sin memoria para los pesos"); f.close(); return false; }
  for (uint32_t off = 0; off < M.n_weights; ) {
    size_t n = f.read((uint8_t *)M.weights + off, min((uint32_t)4096, M.n_weights - off));
    if (n == 0) { Serial.println("ERROR: model.bin cortado"); f.close(); return false; }
    off += n;
  }

  rd(f, M.n_timesteps); rd(f, M.n_channels); rd(f, M.accel_scale); rd(f, M.gyro_scale);

  uint16_t wlen; rd(f, wlen);
  char *blob = (char *)malloc(wlen + 1);
  f.read((uint8_t *)blob, wlen); blob[wlen] = 0;
  int wi = 0; const char *p = blob;
  while (wi < M.num_outputs && wi < MAX_GESTURES && p < blob + wlen) {
    strncpy(M.gesture[wi], p, 15); M.gesture[wi][15] = 0;
    p += strlen(p) + 1; wi++;
  }
  free(blob);
  f.close();

  Serial.printf("Modelo: %d capas (%d", M.num_layers, M.layer[0].in_n);
  for (int i = 0; i < M.num_layers; i++) Serial.printf("-%d", M.layer[i].out_n);
  Serial.printf("), %lu bytes de pesos, %d gestos, remuestreo a %d x %d canales\n",
                (unsigned long)M.n_weights, M.num_outputs, M.n_timesteps, M.n_channels);
  M.ok = true;
  return true;
}

// ================= SPI hacia la FPGA (identico al resto de la etapa 4) =================
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
  uint8_t rx[6]; spi_burst_begin(); npu_xfer(cmd, a, b_raw, rx); spi_burst_end();
}
void npu_send_wide(uint8_t cmd, int32_t value, uint8_t tail) {
  uint8_t rx[6]; spi_burst_begin(); npu_xfer_wide(cmd, value, tail, rx); spi_burst_end();
}

void wait_pll_lock() {
  uint8_t rx[6]; bool locked = false;
  spi_burst_begin();
  for (int i = 0; i < 2000 && !locked; i++) {
    npu_xfer(CMD_NOP, 0, 0, rx);
    locked = (rx[0] & 0x02) != 0;
    if (!locked) delayMicroseconds(100);
  }
  spi_burst_end();
  Serial.printf("PLL lock: %s\n", locked ? "OK" : "NUNCA");
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

// ================= Carga del modelo a la NPU =================
void configure_model() {
  npu_send(CMD_SET_NUM_LAYERS, M.num_layers, 0);
  uint16_t bb = 0;
  for (int L = 0; L < M.num_layers; L++) {
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((0 << 4) | L), M.layer[L].in_n);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((1 << 4) | L), M.layer[L].out_n);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((2 << 4) | L), bb);
    bb += M.layer[L].out_n;
  }
}

#define MAX_BAD 4096
static uint32_t bad_addr[MAX_BAD];

int8_t sdram_read_at(uint32_t addr) {
  uint8_t rx[6];
  uint32_t a = WSTREAM_SDRAM_BASE + addr;
  npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(a & 0xFFFF), (uint16_t)((a >> 16) & 0x7F));
  spi_burst_begin();
  npu_xfer(CMD_SDRAM_RD, 0, 0, rx);
  npu_xfer(CMD_NOP, 0, 0, rx);
  spi_burst_end();
  return (int8_t)rx[1];
}

uint32_t verify_weights() {
  uint8_t rx[6];
  static uint32_t suspect[MAX_BAD];
  uint32_t n_susp = 0;
  npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(WSTREAM_SDRAM_BASE & 0xFFFF), (uint16_t)((WSTREAM_SDRAM_BASE >> 16) & 0x7F));
  spi_burst_begin();
  npu_xfer(CMD_SDRAM_RD, 0, 0, rx);
  for (uint32_t i = 0; i < M.n_weights; i++) {
    npu_xfer(CMD_SDRAM_RD, 0, 0, rx);
    if ((int8_t)rx[1] != M.weights[i]) {
      if (n_susp < MAX_BAD) suspect[n_susp] = i;
      n_susp++;
    }
  }
  spi_burst_end();
  uint32_t bad = 0;
  uint32_t n_check = (n_susp < MAX_BAD) ? n_susp : MAX_BAD;
  for (uint32_t k = 0; k < n_check; k++) {
    uint32_t a = suspect[k];
    int8_t want = M.weights[a];
    if (sdram_read_at(a) == want || sdram_read_at(a) == want) continue;
    if (bad < MAX_BAD) bad_addr[bad] = a;
    bad++;
  }
  return bad;
}

void write_weights_to_sdram() {
  Serial.printf("Escribiendo %lu bytes de pesos en SDRAM...\n", (unsigned long)M.n_weights);
  uint32_t t0 = micros();
  npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(WSTREAM_SDRAM_BASE & 0xFFFF), (uint16_t)((WSTREAM_SDRAM_BASE >> 16) & 0x7F));
  spi_burst_begin();
  uint8_t rx[6];
  for (uint32_t i = 0; i < M.n_weights; i++)
    npu_xfer(CMD_SDRAM_WR, 0, (uint16_t)(uint8_t)M.weights[i], rx);
  spi_burst_end();
  Serial.printf("  escritos en %.2f s, verificando...\n", (micros() - t0) / 1e6);

  for (int attempt = 1; attempt <= 6; attempt++) {
    uint32_t bad = verify_weights();
    if (bad == 0) {
      Serial.printf("  verificado OK%s (%.2f s en total)\n",
                    attempt > 1 ? " despues de reintentos" : "", (micros() - t0) / 1e6);
      break;
    }
    Serial.printf("  intento %d: %lu byte(s) mal, reescribiendo...\n", attempt, (unsigned long)bad);
    uint32_t n_fix = (bad < MAX_BAD) ? bad : MAX_BAD;
    uint8_t rx2[6];
    for (uint32_t k = 0; k < n_fix; k++) {
      npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(bad_addr[k] & 0xFFFF), (uint16_t)((bad_addr[k] >> 16) & 0x7F));
      spi_burst_begin();
      npu_xfer(CMD_SDRAM_WR, 0, (uint16_t)(uint8_t)M.weights[bad_addr[k]], rx2);
      spi_burst_end();
    }
    if (attempt == 6) Serial.println("  AVISO: quedan bytes mal despues de 6 intentos");
  }
  npu_send(CMD_WSTREAM_SET_BASE, (uint16_t)(WSTREAM_SDRAM_BASE & 0xFFFF), (uint16_t)((WSTREAM_SDRAM_BASE >> 16) & 0x7F));
}

void load_biases_and_qparams() {
  npu_send(CMD_SET_BIAS_ADDR, 0, 0);
  spi_burst_begin();
  uint8_t rx[6];
  for (uint32_t i = 0; i < M.n_bias; i++) npu_xfer_wide(CMD_BIAS_WR, M.bias[i], 0, rx);
  spi_burst_end();
  for (int L = 0; L < M.num_layers; L++) {
    uint8_t l = (uint8_t)L;
    npu_send_wide(CMD_SET_QPARAM, M.layer[L].mult,    (l << 2) | 0);
    npu_send_wide(CMD_SET_QPARAM, M.layer[L].zp_out,  (l << 2) | 1);
    npu_send_wide(CMD_SET_QPARAM, M.layer[L].act_min, (l << 2) | 2);
    npu_send_wide(CMD_SET_QPARAM, M.layer[L].act_max, (l << 2) | 3);
  }
}

// ================= Inferencia (FPGA + software, identico al resto) =================
void fpga_infer(const int8_t *feat, int8_t *out, uint32_t *t_load, uint32_t *t_compute) {
  uint8_t rx[6];
  uint32_t t0 = micros();
  npu_send(CMD_SET_ITGT, 0, 0);
  uint8_t buf[5]; int bufcount = 0;
  spi_burst_begin();
  for (int i = 0; i < M.n_features; i++) {
    buf[bufcount++] = (uint8_t)feat[i];
    if (bufcount == 5) { npu_xfer_burst(CMD_IBURST5, buf[0], buf[1], buf[2], buf[3], buf[4], rx); bufcount = 0; }
  }
  while (bufcount != 0) {
    buf[bufcount++] = 0;
    if (bufcount == 5) { npu_xfer_burst(CMD_IBURST5, buf[0], buf[1], buf[2], buf[3], buf[4], rx); bufcount = 0; }
  }
  npu_xfer(CMD_START, 0, 0, rx);
  spi_burst_end();
  uint32_t t1 = micros();
  npu_wait_done();
  uint32_t t2 = micros();
  for (int i = 0; i < M.layer[M.num_layers - 1].out_n; i++) out[i] = npu_dbg_read(DBG_SEL_OUT_MEM, i);
  *t_load = t1 - t0;
  *t_compute = t2 - t1;
}

static uint32_t layer_stream_base[MAX_LAYERS];
void compute_stream_bases() {
  uint32_t b = 0;
  for (int L = 0; L < M.num_layers; L++) {
    layer_stream_base[L] = b;
    uint32_t waves = (M.layer[L].out_n + NLANES - 1) / NLANES;
    b += waves * M.layer[L].in_n * NLANES;
  }
}
static inline int8_t w_at(int L, int n, int i) {
  uint32_t wv = n / NLANES, ln = n % NLANES;
  return M.weights[layer_stream_base[L] + ((uint32_t)wv * M.layer[L].in_n + i) * NLANES + ln];
}

static int8_t sw_bufA[256], sw_bufB[256];
void sw_infer(const int8_t *feat, int8_t *out) {
  const int8_t *src = feat;
  int8_t *dst;
  uint32_t bias_base = 0;
  for (int L = 0; L < M.num_layers; L++) {
    bool is_last = (L == M.num_layers - 1);
    dst = is_last ? out : ((L % 2 == 0) ? sw_bufA : sw_bufB);
    for (int n = 0; n < M.layer[L].out_n; n++) {
      int32_t acc = 0;
      for (int i = 0; i < M.layer[L].in_n; i++) acc += (int32_t)w_at(L, n, i) * (int32_t)src[i];
      int64_t biased = (int64_t)acc + (int64_t)M.bias[bias_base + n];
      int64_t scaled = (biased * (int64_t)M.layer[L].mult) >> M.qshift;
      scaled += M.layer[L].zp_out;
      if (scaled < M.layer[L].act_min) scaled = M.layer[L].act_min;
      if (scaled > M.layer[L].act_max) scaled = M.layer[L].act_max;
      dst[n] = (int8_t)scaled;
    }
    bias_base += M.layer[L].out_n;
    src = dst;
  }
}

int argmax(const int8_t *v, int n) {
  int best = 0;
  for (int i = 1; i < n; i++) if (v[i] > v[best]) best = i;
  return best;
}

// ================= Remuestreo de la grabacion cruda a features =================
// Replica EXACTO el remuestreo de tools/train_gestures_npu.py
// (resample_recording): interpolacion lineal sobre el tiempo normalizado
// [0,1] a N_TIMESTEPS puntos, normalizado por canal, cuantizado a int8.
struct RawSample { float t, ax, ay, az, gx, gy, gz; };
static RawSample raw_buf[MAX_RAW_SAMPLES];

static float interp_channel(const float *t, const float *v, int n, float tq) {
  if (tq <= t[0]) return v[0];
  if (tq >= t[n - 1]) return v[n - 1];
  int lo = 0, hi = n - 1;
  while (hi - lo > 1) { int mid = (lo + hi) / 2; if (t[mid] <= tq) lo = mid; else hi = mid; }
  float span = t[hi] - t[lo];
  float frac = (span > 0) ? (tq - t[lo]) / span : 0.0f;
  return v[lo] + frac * (v[hi] - v[lo]);
}

void motion_to_features(const RawSample *s, int n, int8_t *feat_out) {
  static float t[MAX_RAW_SAMPLES];
  static float ch[6][MAX_RAW_SAMPLES];
  float t0 = s[0].t, tspan = s[n - 1].t - t0;
  if (tspan <= 0) tspan = 1.0f;
  for (int i = 0; i < n; i++) {
    t[i] = (s[i].t - t0) / tspan;
    ch[0][i] = s[i].ax; ch[1][i] = s[i].ay; ch[2][i] = s[i].az;
    ch[3][i] = s[i].gx; ch[4][i] = s[i].gy; ch[5][i] = s[i].gz;
  }
  const float scale[6] = { M.accel_scale, M.accel_scale, M.accel_scale,
                            M.gyro_scale,  M.gyro_scale,  M.gyro_scale };
  for (int k = 0; k < M.n_timesteps; k++) {
    float tq = (float)k / (float)(M.n_timesteps - 1);
    for (int c = 0; c < M.n_channels; c++) {
      float v = interp_channel(t, ch[c], n, tq) / scale[c];
      if (v < -1.0f) v = -1.0f; if (v > 1.0f) v = 1.0f;
      int q = (int)lroundf(v / M.input_scale) + M.input_zp;
      if (q < -128) q = -128; if (q > 127) q = 127;
      feat_out[k * M.n_channels + c] = (int8_t)q;
    }
  }
}

// ================= Benchmark (identico al resto: clips ya cuantizados en test.bin) =================
void run_benchmark() {
  File f = LittleFS.open("/test.bin", "r");
  if (!f) { Serial.println("\n(sin test.bin: se omite el benchmark)"); return; }
  uint16_t n_clips, n_feat;
  rd(f, n_clips); rd(f, n_feat);
  if (n_feat != M.n_features) { Serial.println("test.bin no corresponde a este modelo"); f.close(); return; }

  int8_t *feats = (int8_t *)alloc_big((size_t)n_clips * n_feat);
  uint8_t *labels = (uint8_t *)malloc(n_clips), *refs = (uint8_t *)malloc(n_clips);
  if (!feats || !labels || !refs) { Serial.println("sin memoria para el benchmark"); f.close(); return; }
  f.read((uint8_t *)feats, (size_t)n_clips * n_feat);
  f.read(labels, n_clips);
  f.read(refs, n_clips);
  f.close();

  int n_out = M.layer[M.num_layers - 1].out_n;
  int8_t fpga_out[16], sw_out[16];
  int fpga_vs_ref = 0, sw_vs_ref = 0, fpga_vs_label = 0, ref_vs_label = 0;
  uint64_t fpga_us = 0, sw_us = 0;

  Serial.printf("\n=== BENCHMARK: %d clips de prueba ===\n", n_clips);
  for (int k = 0; k < n_clips; k++) {
    const int8_t *feat = feats + (size_t)k * n_feat;
    uint32_t t_load, t_comp;
    fpga_infer(feat, fpga_out, &t_load, &t_comp);
    uint32_t s0 = micros();
    sw_infer(feat, sw_out);
    uint32_t sw_t = micros() - s0;

    int p_fpga = argmax(fpga_out, n_out), p_sw = argmax(sw_out, n_out);
    if (p_fpga == refs[k]) fpga_vs_ref++;
    if (p_sw == refs[k]) sw_vs_ref++;
    if (p_fpga == labels[k]) fpga_vs_label++;
    if (refs[k] == labels[k]) ref_vs_label++;
    fpga_us += t_comp; sw_us += sw_t;
    if (p_fpga != refs[k])
      Serial.printf("  [%3d] DIFIERE: FPGA=%s ref=%s\n", k, M.gesture[p_fpga], M.gesture[refs[k]]);
  }

  Serial.println("\n========================================");
  Serial.printf("Coincidencia con la referencia de Python:\n");
  Serial.printf("   FPGA        %d/%d\n", fpga_vs_ref, n_clips);
  Serial.printf("   ESP32 (sw)  %d/%d\n", sw_vs_ref, n_clips);
  Serial.printf("\nPrecision sobre el gesto real:\n");
  Serial.printf("   FPGA           %d/%d  (%.1f%%)\n", fpga_vs_label, n_clips, 100.0 * fpga_vs_label / n_clips);
  Serial.printf("   modelo en PC   %d/%d  (%.1f%%)  <- techo del modelo\n",
                ref_vs_label, n_clips, 100.0 * ref_vs_label / n_clips);
  Serial.printf("\nTiempo por inferencia:  FPGA %.2f ms   ESP32 %.2f ms   -> %.2fx\n",
                fpga_us / 1000.0 / n_clips, sw_us / 1000.0 / n_clips,
                (double)sw_us / (double)fpga_us);
  Serial.println("========================================");

  free(feats); free(labels); free(refs);
}

// ================= Servidor web =================
void handleRoot() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) { server.send(500, "text/plain", "falta index.html en el sistema de archivos"); return; }
  server.streamFile(f, "text/html");
  f.close();
}

// Cuerpo esperado: texto plano, una linea por muestra:
//   t,ax,ay,az,gx,gy,gz
// (mismos campos que manda la pagina de captura de datos de entrenamiento
// -- separado por comas en vez de JSON para no necesitar una libreria de
// parseo en el firmware).
void handlePredict() {
  String body = server.arg("plain");
  int n = 0;
  int lineStart = 0;
  int len = body.length();
  while (lineStart < len && n < MAX_RAW_SAMPLES) {
    int lineEnd = body.indexOf('\n', lineStart);
    if (lineEnd < 0) lineEnd = len;
    String line = body.substring(lineStart, lineEnd);
    line.trim();
    if (line.length() > 0) {
      float vals[7]; int nv = 0;
      int start = 0;
      while (nv < 7) {
        int comma = line.indexOf(',', start);
        String tok = (comma < 0) ? line.substring(start) : line.substring(start, comma);
        vals[nv++] = tok.toFloat();
        if (comma < 0) break;
        start = comma + 1;
      }
      if (nv == 7) {
        raw_buf[n] = { vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6] };
        n++;
      }
    }
    lineStart = lineEnd + 1;
  }

  if (n < 5) {
    server.send(400, "application/json", "{\"error\":\"muy pocas muestras\"}");
    return;
  }

  static int8_t feat[256];
  motion_to_features(raw_buf, n, feat);

  int n_out = M.layer[M.num_layers - 1].out_n;
  int8_t out[16], sw_out[16];
  uint32_t t_load, t_comp;
  fpga_infer(feat, out, &t_load, &t_comp);
  uint32_t s0 = micros();
  sw_infer(feat, sw_out);
  uint32_t sw_us = micros() - s0;

  int pred = argmax(out, n_out), sw_pred = argmax(sw_out, n_out);
  String json = "{\"gesto\":\"" + String(M.gesture[pred]) + "\",\"ms\":" + String(t_comp / 1000.0, 3) +
                ",\"swGesto\":\"" + String(M.gesture[sw_pred]) + "\",\"swMs\":" + String(sw_us / 1000.0, 3) +
                ",\"nMuestras\":" + String(n) +
                ",\"scores\":[";
  for (int i = 0; i < n_out; i++) { json += String((int)out[i]); if (i < n_out - 1) json += ","; }
  json += "],\"gestos\":[";
  for (int i = 0; i < n_out; i++) { json += "\"" + String(M.gesture[i]) + "\""; if (i < n_out - 1) json += ","; }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("\n=== GESTOS (varita magica) en la NPU (modelo leido desde flash) ===");
  Serial.printf("PSRAM: %lu bytes libres\n", (unsigned long)ESP.getFreePsram());

  if (!LittleFS.begin(true, "/littlefs", 10, "model")) {
    Serial.println("ERROR: no se pudo montar LittleFS.");
    Serial.println("       Subi los archivos con:  pio run -t uploadfs");
    return;
  }
  if (!load_model("/model.bin")) {
    Serial.println("       Subi los archivos con:  pio run -t uploadfs");
    return;
  }

  wait_pll_lock();
  compute_stream_bases();
  configure_model();
  write_weights_to_sdram();
  load_biases_and_qparams();
  Serial.println("Modelo cargado en la NPU.");

  run_benchmark();

  String url;
  bool connected = false;
  if (strlen(WIFI_SSID) > 0) {
    Serial.printf("\nConectando a \"%s\"", WIFI_SSID);
    WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
    Serial.println();
    connected = (WiFi.status() == WL_CONNECTED);
    if (connected) url = WiFi.localIP().toString();
    else Serial.println("No conecto, creando red propia...");
  }
  if (!connected) { WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS); url = WiFi.softAPIP().toString(); }

  server.on("/", handleRoot);
  server.on("/predict", HTTP_POST, handlePredict);
  server.begin();

  Serial.println("\n=== DEMO WEB LISTA ===");
  if (!connected) Serial.printf("Conectate a la red \"%s\" (clave %s)\n", AP_SSID, AP_PASS);
  Serial.printf("Abri:  http://%s\n", url.c_str());
  Serial.println("======================");
}

void loop() { server.handleClient(); }
