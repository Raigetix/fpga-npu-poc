// kws_npu_poc.cpp -- Etapa 4: DETECCION DE PALABRAS CLAVE en la NPU.
//
// El modelo y la pagina NO estan compilados dentro del firmware: viven en
// el sistema de archivos (LittleFS) de la flash y se leen al arrancar.
//   data/model.bin  -- capas, cuantizacion, bias, pesos y tablas del MFCC
//   data/test.bin   -- clips de prueba para el benchmark (opcional)
//   data/index.html -- la pagina de la demo
//
// Por que asi y no como header:
//   1) Compilar 1,5 MB de literales numericos tardaba varios minutos cada
//      vez que se tocaba una linea de codigo.
//   2) Se puede cambiar de modelo SIN recompilar el firmware: se sube otro
//      model.bin. Es el mismo espiritu de la NPU configurable por SPI del
//      lado de la FPGA, ahora tambien del lado del ESP32.
//
// Para subir:  pio run -t uploadfs   (los archivos)
//              pio run -t upload     (el firmware)
// Si falta el sistema de archivos, avisa por el puerto serie en vez de
// colgarse.
//
// EL MICROFONO NECESITA HTTPS: los navegadores lo bloquean en paginas
// http://. Chrome (PC y Android): chrome://flags -> "Insecure origins
// treated as secure" -> agregar http://<ip> -> reiniciar. El benchmark
// funciona igual sin microfono.

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
#define MAX_WORDS 16
#define CLIP_SAMPLES 16000

// ================= WiFi =================
// Si no logra conectarse en 15 segundos, cae automaticamente a crear su
// propia red (asi nunca te quedas sin poder entrar a la pagina).
const char *WIFI_SSID = "Fibertel WiFi311 2.4GHz";  // <-- el nombre de tu red WiFi
const char *WIFI_PASS = "00417751437";              // <-- la clave de tu red WiFi
const char *AP_SSID = "NPU-KWS";     // red propia, solo si falla lo de arriba
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
  uint16_t n_mel, n_mfcc, n_bins, n_frames, frame_length, frame_step, fft_length;
  float   *mel, *dct;
  char     word[MAX_WORDS][16];
  bool     ok;
} M;

// Reserva preferentemente en PSRAM (la placa tiene 8 MB): los pesos son
// ~229 kB y no entrarian comodos en la RAM interna.
static void *alloc_big(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (!p) p = malloc(n);      // sin PSRAM, intentar igual
  return p;
}

template <typename T> static bool rd(File &f, T &v) {
  return f.read((uint8_t *)&v, sizeof(T)) == sizeof(T);
}

bool load_model(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) { Serial.printf("ERROR: no se encuentra %s\n", path); return false; }

  char magic[4];
  if (f.read((uint8_t *)magic, 4) != 4 || memcmp(magic, "NPU1", 4)) {
    Serial.println("ERROR: model.bin no tiene el formato esperado"); f.close(); return false;
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
  // lectura por bloques: el archivo es grande y conviene no pedir todo de una
  for (uint32_t off = 0; off < M.n_weights; ) {
    size_t n = f.read((uint8_t *)M.weights + off, min((uint32_t)4096, M.n_weights - off));
    if (n == 0) { Serial.println("ERROR: model.bin cortado"); f.close(); return false; }
    off += n;
  }

  rd(f, M.n_mel); rd(f, M.n_mfcc); rd(f, M.n_bins); rd(f, M.n_frames);
  rd(f, M.frame_length); rd(f, M.frame_step); rd(f, M.fft_length);
  M.mel = (float *)alloc_big((size_t)M.n_bins * M.n_mel * sizeof(float));
  M.dct = (float *)alloc_big((size_t)M.n_mel * M.n_mfcc * sizeof(float));
  if (!M.mel || !M.dct) { Serial.println("ERROR: sin memoria para las tablas MFCC"); f.close(); return false; }
  f.read((uint8_t *)M.mel, (size_t)M.n_bins * M.n_mel * sizeof(float));
  f.read((uint8_t *)M.dct, (size_t)M.n_mel * M.n_mfcc * sizeof(float));

  uint16_t wlen; rd(f, wlen);
  char *blob = (char *)malloc(wlen + 1);
  f.read((uint8_t *)blob, wlen); blob[wlen] = 0;
  int wi = 0; const char *p = blob;
  while (wi < M.num_outputs && wi < MAX_WORDS && p < blob + wlen) {
    strncpy(M.word[wi], p, 15); M.word[wi][15] = 0;
    p += strlen(p) + 1; wi++;
  }
  free(blob);
  f.close();

  Serial.printf("Modelo: %d capas (%d", M.num_layers, M.layer[0].in_n);
  for (int i = 0; i < M.num_layers; i++) Serial.printf("-%d", M.layer[i].out_n);
  Serial.printf("), %lu bytes de pesos, %d salidas\n",
                (unsigned long)M.n_weights, M.num_outputs);
  M.ok = true;
  return true;
}

// ================= SPI hacia la FPGA =================
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

// La escritura a SDRAM falla ~0.004% de los bytes y UN peso corrupto
// arruina todas las inferencias -> verificar y reintentar. La lectura por
// SPI falla ~0.15%, asi que cada sospechoso se confirma con relecturas
// (si no, aparecerian cientos de errores inexistentes).
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

// ================= Inferencia =================
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

// ================= MFCC =================
// Replica exactamente la cadena del entrenamiento. Las matrices mel y DCT
// vienen del mismo script que entreno el modelo, asi que no hay forma de
// que las formulas difieran.
#define MAX_FFT 1024
static float fft_re[MAX_FFT], fft_im[MAX_FFT];
static float hann_w[MAX_FFT];
static uint16_t bitrev[MAX_FFT];

void mfcc_init() {
  for (int i = 0; i < M.frame_length; i++)      // hann periodica, como tf.signal
    hann_w[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / M.frame_length);
  int bits = 0; while ((1 << bits) < M.fft_length) bits++;
  for (int i = 0; i < M.fft_length; i++) {
    uint16_t r = 0;
    for (int b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
    bitrev[i] = r;
  }
}

void fft_run() {
  const int N = M.fft_length;
  for (int i = 0; i < N; i++) {
    int j = bitrev[i];
    if (j > i) { float t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;
                 t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t; }
  }
  for (int len = 2; len <= N; len <<= 1) {
    float ang = -2.0f * (float)M_PI / len;
    for (int i = 0; i < N; i += len) {
      for (int k = 0; k < len / 2; k++) {
        float wr = cosf(ang * k), wi = sinf(ang * k);
        int a = i + k, b = i + k + len / 2;
        float tr = fft_re[b] * wr - fft_im[b] * wi;
        float ti = fft_re[b] * wi + fft_im[b] * wr;
        fft_re[b] = fft_re[a] - tr; fft_im[b] = fft_im[a] - ti;
        fft_re[a] += tr;            fft_im[a] += ti;
      }
    }
  }
}

void audio_to_features(const float *audio, int8_t *feat_out) {
  static float mag[MAX_FFT / 2 + 1];
  static float logmel[64];
  for (int f = 0; f < M.n_frames; f++) {
    const float *src = audio + (size_t)f * M.frame_step;
    for (int i = 0; i < M.frame_length; i++) { fft_re[i] = src[i] * hann_w[i]; fft_im[i] = 0.0f; }
    for (int i = M.frame_length; i < M.fft_length; i++) { fft_re[i] = 0.0f; fft_im[i] = 0.0f; }
    fft_run();
    for (int k = 0; k < M.n_bins; k++) mag[k] = sqrtf(fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]);
    for (int m = 0; m < M.n_mel; m++) {
      float s = 0.0f;
      for (int k = 0; k < M.n_bins; k++) s += mag[k] * M.mel[(size_t)k * M.n_mel + m];
      logmel[m] = logf(s + 1e-6f);
    }
    for (int c = 0; c < M.n_mfcc; c++) {
      float s = 0.0f;
      for (int m = 0; m < M.n_mel; m++) s += logmel[m] * M.dct[(size_t)m * M.n_mfcc + c];
      int q = (int)lroundf(s / M.input_scale) + M.input_zp;
      if (q < -128) q = -128;
      if (q > 127) q = 127;
      feat_out[f * M.n_mfcc + c] = (int8_t)q;
    }
  }
}

// ================= Benchmark =================
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

  Serial.printf("\n=== BENCHMARK: %d clips reales ===\n", n_clips);
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
      Serial.printf("  [%3d] DIFIERE: FPGA=%s ref=%s\n", k, M.word[p_fpga], M.word[refs[k]]);
    if ((k + 1) % 50 == 0) Serial.printf("  ...%d/%d\n", k + 1, n_clips);
  }

  Serial.println("\n========================================");
  Serial.printf("Coincidencia con la referencia de la PC (mide si el hardware es exacto):\n");
  Serial.printf("   FPGA        %d/%d\n", fpga_vs_ref, n_clips);
  Serial.printf("   ESP32 (sw)  %d/%d\n", sw_vs_ref, n_clips);
  Serial.printf("\nPrecision sobre la palabra real:\n");
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

static int8_t b64val(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

static float *audio_buf = nullptr;

void handlePredict() {
  String body = server.arg("plain");
  int n = 0, acc = 0, bits = 0;
  uint8_t lo = 0; bool have_lo = false;
  for (unsigned int i = 0; i < body.length() && n < CLIP_SAMPLES; i++) {
    int8_t v = b64val(body[i]);
    if (v < 0) continue;
    acc = (acc << 6) | v; bits += 6;
    if (bits >= 8) {
      bits -= 8;
      uint8_t byte = (acc >> bits) & 0xFF;
      if (!have_lo) { lo = byte; have_lo = true; }
      else { int16_t s = (int16_t)((uint16_t)byte << 8 | lo);
             audio_buf[n++] = s / 32768.0f; have_lo = false; }
    }
  }
  while (n < CLIP_SAMPLES) audio_buf[n++] = 0.0f;

  static int8_t feat[1024];
  uint32_t m0 = micros();
  audio_to_features(audio_buf, feat);
  uint32_t mfcc_us = micros() - m0;

  // Diagnostico: estadisticas del vector de caracteristicas que realmente
  // ve la red, para comparar contra lo esperado (calculado en Python sobre
  // grabaciones reales) sin tener que instrumentar mas.
  int8_t feat_min = 127, feat_max = -128;
  long feat_sum = 0;
  int sat_lo = 0, sat_hi = 0;
  float raw_min = 1e9f, raw_max = -1e9f, raw_sum = 0.0f;
  for (int i = 0; i < CLIP_SAMPLES; i++) {
    if (audio_buf[i] < raw_min) raw_min = audio_buf[i];
    if (audio_buf[i] > raw_max) raw_max = audio_buf[i];
    raw_sum += fabsf(audio_buf[i]);
  }
  for (int i = 0; i < M.n_features; i++) {
    if (feat[i] < feat_min) feat_min = feat[i];
    if (feat[i] > feat_max) feat_max = feat[i];
    feat_sum += feat[i];
    if (feat[i] <= -128) sat_lo++;
    if (feat[i] >= 127) sat_hi++;
  }
  float feat_mean = (float)feat_sum / M.n_features;

  // Diagnostico completo por serie: el vector de 490 caracteristicas tal
  // cual lo ve la red, para comparar numero por numero contra Python.
  Serial.print("FEAT:");
  for (int i = 0; i < M.n_features; i++) { Serial.print((int)feat[i]); if (i < M.n_features - 1) Serial.print(','); }
  Serial.println();

  int n_out = M.layer[M.num_layers - 1].out_n;
  int8_t out[16], sw_out[16];
  uint32_t t_load, t_comp;
  fpga_infer(feat, out, &t_load, &t_comp);
  uint32_t s0 = micros();
  sw_infer(feat, sw_out);
  uint32_t sw_us = micros() - s0;

  int pred = argmax(out, n_out), sw_pred = argmax(sw_out, n_out);
  String json = "{\"word\":\"" + String(M.word[pred]) + "\",\"ms\":" + String(t_comp / 1000.0, 3) +
                ",\"swWord\":\"" + String(M.word[sw_pred]) + "\",\"swMs\":" + String(sw_us / 1000.0, 3) +
                ",\"mfccMs\":" + String(mfcc_us / 1000.0, 2) +
                ",\"featMin\":" + String((int)feat_min) + ",\"featMax\":" + String((int)feat_max) +
                ",\"featMean\":" + String(feat_mean, 1) +
                ",\"satLo\":" + String(sat_lo) + ",\"satHi\":" + String(sat_hi) +
                ",\"rawMin\":" + String(raw_min, 3) + ",\"rawMax\":" + String(raw_max, 3) +
                ",\"rawAvgAbs\":" + String(raw_sum / CLIP_SAMPLES, 4) +
                ",\"zpIn\":" + String((int)M.input_zp) +
                ",\"scores\":[";
  for (int i = 0; i < n_out; i++) { json += String((int)out[i]); if (i < n_out - 1) json += ","; }
  json += "],\"words\":[";
  for (int i = 0; i < n_out; i++) { json += "\"" + String(M.word[i]) + "\""; if (i < n_out - 1) json += ","; }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("\n=== PALABRAS CLAVE en la NPU (modelo leido desde flash) ===");
  Serial.printf("PSRAM: %lu bytes libres\n", (unsigned long)ESP.getFreePsram());

  if (!LittleFS.begin(false, "/littlefs", 10, "model")) {
    Serial.println("ERROR: no se pudo montar LittleFS.");
    Serial.println("       Subi los archivos con:  pio run -t uploadfs");
    return;
  }
  if (!load_model("/model.bin")) {
    Serial.println("       Subi los archivos con:  pio run -t uploadfs");
    return;
  }

  audio_buf = (float *)alloc_big(CLIP_SAMPLES * sizeof(float));
  if (!audio_buf) { Serial.println("ERROR: sin memoria para el audio"); return; }

  wait_pll_lock();
  mfcc_init();
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
