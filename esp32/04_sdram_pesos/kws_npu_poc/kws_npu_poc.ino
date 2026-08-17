// kws_npu_poc.ino -- Etapa 4: DETECCION DE PALABRAS CLAVE en la NPU.
//
// 8 palabras: down, go, left, no, right, stop, up, yes.
// Modelo: red densa 490 -> 256 -> 256 -> 144 -> 8, entrenada en Keras sobre
// el dataset publico de Google (mini Speech Commands) y cuantizada a int8
// con el conversor estandar de TFLite. Ver tools/train_kws_npu.py.
//
// Son 228.992 bytes de pesos: 5,5 veces lo que entraba en la BRAM de la
// placa. Viven en la SDRAM y se transmiten durante el computo.
//
// Hace dos cosas:
//   1) BENCHMARK con 200 clips reales del set de prueba (las
//      caracteristicas ya vienen calculadas en el header), comparando la
//      FPGA contra el mismo modelo en software en el ESP32.
//   2) DEMO WEB: grabas 1 segundo con el microfono, el ESP32 calcula los
//      MFCC y la FPGA dice que palabra escucho.
//
// EL MICROFONO NECESITA HTTPS: los navegadores bloquean getUserMedia en
// paginas http://. Para que ande hay que habilitar el origen a mano:
//   Chrome (PC y Android): chrome://flags -> "Insecure origins treated as
//     secure" -> agregar http://<ip-del-esp32> -> reiniciar el navegador.
//   Firefox: probar en about:config media.devices.insecure.enabled y
//     media.getusermedia.insecure.enabled en true (puede que ya no existan
//     en versiones nuevas).
// El benchmark y la parte de resultados funcionan igual sin microfono.

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include "kws_model.h"

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

// ================= WiFi =================
// Poné acá los datos de TU red. Si queda vacio o no conecta, crea su
// propia red.
const char *WIFI_SSID = "Fibertel WiFi311 2.4GHz";
const char *WIFI_PASS = "00417751437";
const char *AP_SSID = "NPU-KWS";
const char *AP_PASS = "npu12345";

SPIClass fpga_spi(FSPI);
WebServer server(80);

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

// ================= Carga del modelo =================
void configure_model() {
  npu_send(CMD_SET_NUM_LAYERS, NUM_LAYERS, 0);
  uint16_t bb = 0;
  for (int L = 0; L < NUM_LAYERS; L++) {
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((0 << 4) | L), LAYER_IN[L]);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((1 << 4) | L), LAYER_OUT[L]);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((2 << 4) | L), bb);
    bb += LAYER_OUT[L];
  }
}

// La escritura a SDRAM falla ~0.004% de los bytes y UN peso corrupto
// arruina todas las inferencias -> verificar y reintentar. La lectura por
// SPI falla ~0.15%, asi que cada sospechoso se confirma con relecturas
// (si no, aparecerian cientos de errores inexistentes).
#define MAX_BAD 4096
uint32_t bad_addr[MAX_BAD];

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
  for (int i = 0; i < WEIGHT_STREAM_BYTES; i++) {
    npu_xfer(CMD_SDRAM_RD, 0, 0, rx);
    if ((int8_t)rx[1] != WEIGHT_STREAM[i]) {
      if (n_susp < MAX_BAD) suspect[n_susp] = i;
      n_susp++;
    }
  }
  spi_burst_end();
  uint32_t bad = 0;
  uint32_t n_check = (n_susp < MAX_BAD) ? n_susp : MAX_BAD;
  for (uint32_t k = 0; k < n_check; k++) {
    uint32_t a = suspect[k];
    int8_t want = WEIGHT_STREAM[a];
    if (sdram_read_at(a) == want || sdram_read_at(a) == want) continue;
    if (bad < MAX_BAD) bad_addr[bad] = a;
    bad++;
  }
  return bad;
}

void write_weights_to_sdram() {
  Serial.printf("Escribiendo %d bytes de pesos en SDRAM...\n", WEIGHT_STREAM_BYTES);
  uint32_t t0 = micros();
  npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(WSTREAM_SDRAM_BASE & 0xFFFF), (uint16_t)((WSTREAM_SDRAM_BASE >> 16) & 0x7F));
  spi_burst_begin();
  uint8_t rx[6];
  for (int i = 0; i < WEIGHT_STREAM_BYTES; i++)
    npu_xfer(CMD_SDRAM_WR, 0, (uint16_t)(uint8_t)WEIGHT_STREAM[i], rx);
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
      npu_xfer(CMD_SDRAM_WR, 0, (uint16_t)(uint8_t)WEIGHT_STREAM[bad_addr[k]], rx2);
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
  for (int i = 0; i < TOTAL_BIAS; i++) npu_xfer_wide(CMD_BIAS_WR, BIAS_FOLDED[i], 0, rx);
  spi_burst_end();
  for (int L = 0; L < NUM_LAYERS; L++) {
    uint8_t layer = (uint8_t)L;
    npu_send_wide(CMD_SET_QPARAM, LAYER_MULT[L],   (layer << 2) | 0);
    npu_send_wide(CMD_SET_QPARAM, LAYER_ZPOUT[L],  (layer << 2) | 1);
    npu_send_wide(CMD_SET_QPARAM, LAYER_ACTMIN[L], (layer << 2) | 2);
    npu_send_wide(CMD_SET_QPARAM, LAYER_ACTMAX[L], (layer << 2) | 3);
  }
}

// ================= Inferencia =================
void fpga_infer(const int8_t *feat, int8_t *out, uint32_t *t_load, uint32_t *t_compute) {
  uint8_t rx[6];
  uint32_t t0 = micros();
  npu_send(CMD_SET_ITGT, 0, 0);
  uint8_t buf[5]; int bufcount = 0;
  spi_burst_begin();
  for (int i = 0; i < N_FEATURES; i++) {
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
  for (int i = 0; i < LAYER_OUT[NUM_LAYERS - 1]; i++) out[i] = npu_dbg_read(DBG_SEL_OUT_MEM, i);
  *t_load = t1 - t0;
  *t_compute = t2 - t1;
}

uint32_t layer_stream_base[NUM_LAYERS];
void compute_stream_bases() {
  uint32_t b = 0;
  for (int L = 0; L < NUM_LAYERS; L++) {
    layer_stream_base[L] = b;
    uint32_t waves = (LAYER_OUT[L] + NLANES - 1) / NLANES;
    b += waves * LAYER_IN[L] * NLANES;
  }
}
static inline int8_t w_at(int L, int n, int i) {
  uint32_t wv = n / NLANES, ln = n % NLANES;
  return WEIGHT_STREAM[layer_stream_base[L] + ((uint32_t)wv * LAYER_IN[L] + i) * NLANES + ln];
}

int8_t sw_bufA[256], sw_bufB[256];
void sw_infer(const int8_t *feat, int8_t *out) {
  const int8_t *src = feat;
  int8_t *dst;
  uint16_t bias_base = 0;
  for (int L = 0; L < NUM_LAYERS; L++) {
    bool is_last = (L == NUM_LAYERS - 1);
    dst = is_last ? out : ((L % 2 == 0) ? sw_bufA : sw_bufB);
    for (int n = 0; n < LAYER_OUT[L]; n++) {
      int32_t acc = 0;
      for (int i = 0; i < LAYER_IN[L]; i++) acc += (int32_t)w_at(L, n, i) * (int32_t)src[i];
      int64_t biased = (int64_t)acc + (int64_t)BIAS_FOLDED[bias_base + n];
      int64_t scaled = (biased * (int64_t)LAYER_MULT[L]) >> QSHIFT;
      scaled += LAYER_ZPOUT[L];
      if (scaled < LAYER_ACTMIN[L]) scaled = LAYER_ACTMIN[L];
      if (scaled > LAYER_ACTMAX[L]) scaled = LAYER_ACTMAX[L];
      dst[n] = (int8_t)scaled;
    }
    bias_base += LAYER_OUT[L];
    src = dst;
  }
}

int argmax(const int8_t *v, int n) {
  int best = 0;
  for (int i = 1; i < n; i++) if (v[i] > v[best]) best = i;
  return best;
}

// ================= MFCC =================
// Replica EXACTAMENTE la cadena del entrenamiento (ver train_kws_npu.py).
// Las matrices mel y DCT vienen del header, calculadas por el mismo script
// que entreno el modelo -- asi no hay forma de que las formulas difieran.
static float fft_re[FFT_LENGTH], fft_im[FFT_LENGTH];
static float hann_w[FRAME_LENGTH];
static uint16_t bitrev[FFT_LENGTH];

void mfcc_init() {
  for (int i = 0; i < FRAME_LENGTH; i++)          // hann periodica, como tf.signal
    hann_w[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / FRAME_LENGTH);
  int bits = 0; while ((1 << bits) < FFT_LENGTH) bits++;
  for (int i = 0; i < FFT_LENGTH; i++) {
    uint16_t r = 0;
    for (int b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
    bitrev[i] = r;
  }
}

void fft_1024() {
  for (int i = 0; i < FFT_LENGTH; i++) {
    int j = bitrev[i];
    if (j > i) { float t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;
                 t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t; }
  }
  for (int len = 2; len <= FFT_LENGTH; len <<= 1) {
    float ang = -2.0f * (float)M_PI / len;
    for (int i = 0; i < FFT_LENGTH; i += len) {
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

// audio: 16000 muestras float en [-1,1]  ->  features int8 (N_FEATURES)
void audio_to_features(const float *audio, int8_t *feat_out) {
  const int n_bins = FFT_LENGTH / 2 + 1;
  static float mag[FFT_LENGTH / 2 + 1];
  static float logmel[N_MEL];

  for (int f = 0; f < N_FRAMES; f++) {
    const float *src = audio + (size_t)f * FRAME_STEP;
    for (int i = 0; i < FRAME_LENGTH; i++) { fft_re[i] = src[i] * hann_w[i]; fft_im[i] = 0.0f; }
    for (int i = FRAME_LENGTH; i < FFT_LENGTH; i++) { fft_re[i] = 0.0f; fft_im[i] = 0.0f; }
    fft_1024();
    for (int k = 0; k < n_bins; k++) mag[k] = sqrtf(fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]);

    for (int m = 0; m < N_MEL; m++) {
      float s = 0.0f;
      for (int k = 0; k < n_bins; k++) s += mag[k] * MEL_MATRIX[(size_t)k * N_MEL + m];
      logmel[m] = logf(s + 1e-6f);
    }
    for (int c = 0; c < N_MFCC; c++) {
      float s = 0.0f;
      for (int m = 0; m < N_MEL; m++) s += logmel[m] * DCT_MATRIX[(size_t)m * N_MFCC + c];
      int q = (int)lroundf(s / INPUT_SCALE) + INPUT_ZP;
      if (q < -128) q = -128;
      if (q > 127) q = 127;
      feat_out[f * N_MFCC + c] = (int8_t)q;
    }
  }
}

// ================= Benchmark =================
void run_benchmark() {
  int n_out = LAYER_OUT[NUM_LAYERS - 1];
  int8_t fpga_out[16], sw_out[16];
  static int8_t feat[N_FEATURES];
  int fpga_vs_ref = 0, sw_vs_ref = 0, fpga_vs_label = 0, ref_vs_label = 0;
  uint64_t fpga_us = 0, sw_us = 0;

  Serial.printf("\n=== BENCHMARK: %d clips reales ===\n", N_TEST_CLIPS);
  for (int k = 0; k < N_TEST_CLIPS; k++) {
    for (int i = 0; i < N_FEATURES; i++) feat[i] = TEST_FEATURES[(uint32_t)k * N_FEATURES + i];
    uint32_t t_load, t_comp;
    fpga_infer(feat, fpga_out, &t_load, &t_comp);
    uint32_t s0 = micros();
    sw_infer(feat, sw_out);
    uint32_t sw_t = micros() - s0;

    int p_fpga = argmax(fpga_out, n_out), p_sw = argmax(sw_out, n_out);
    int ref = TFLITE_PRED[k], label = TEST_LABELS[k];
    if (p_fpga == ref) fpga_vs_ref++;
    if (p_sw == ref) sw_vs_ref++;
    if (p_fpga == label) fpga_vs_label++;
    if (ref == label) ref_vs_label++;
    fpga_us += t_comp; sw_us += sw_t;
    if (p_fpga != ref) Serial.printf("  [%3d] DIFIERE: FPGA=%s ref=%s\n", k, WORDS[p_fpga], WORDS[ref]);
    if ((k + 1) % 50 == 0) Serial.printf("  ...%d/%d\n", k + 1, N_TEST_CLIPS);
  }

  Serial.println("\n========================================");
  Serial.printf("Coincidencia con la referencia de la PC (mide si el hardware es exacto):\n");
  Serial.printf("   FPGA        %d/%d\n", fpga_vs_ref, N_TEST_CLIPS);
  Serial.printf("   ESP32 (sw)  %d/%d\n", sw_vs_ref, N_TEST_CLIPS);
  Serial.printf("\nPrecision sobre la palabra real:\n");
  Serial.printf("   FPGA           %d/%d  (%.1f%%)\n", fpga_vs_label, N_TEST_CLIPS, 100.0 * fpga_vs_label / N_TEST_CLIPS);
  Serial.printf("   modelo en PC   %d/%d  (%.1f%%)  <- techo del modelo\n",
                ref_vs_label, N_TEST_CLIPS, 100.0 * ref_vs_label / N_TEST_CLIPS);
  Serial.printf("\nTiempo por inferencia:  FPGA %.2f ms   ESP32 %.2f ms   -> %.2fx\n",
                fpga_us / 1000.0 / N_TEST_CLIPS, sw_us / 1000.0 / N_TEST_CLIPS,
                (double)sw_us / (double)fpga_us);
  Serial.println("========================================");
}

// ================= Demo web =================
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NPU FPGA - Palabras clave</title>
<style>
 *{box-sizing:border-box}
 body{margin:0;padding:18px;background:#0e1116;color:#e6edf3;
      font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;text-align:center}
 h1{font-size:1.1rem;margin:0 0 4px}
 p.sub{margin:0 0 16px;color:#8b949e;font-size:.85rem}
 .words{color:#8b949e;font-size:.8rem;margin-bottom:16px}
 .words b{color:#e6edf3;font-weight:600}
 button{background:#1f6feb;color:#fff;border:0;border-radius:10px;
        padding:14px 28px;font-size:1rem;cursor:pointer}
 button:disabled{background:#21262d;color:#8b949e}
 #status{margin:12px 0;color:#8b949e;font-size:.85rem;min-height:1.2rem}
 .vs{display:flex;gap:10px;max-width:360px;margin:12px auto 0}
 .card{flex:1;background:#161b22;border:1px solid #30363d;border-radius:10px;padding:10px 6px}
 .card.win{border-color:#3fb950}
 .card .who{font-size:.72rem;color:#8b949e}
 .card .word{font-size:1.8rem;font-weight:700;line-height:1.3}
 .card .ms{font-size:.8rem;color:#8b949e}
 .card.win .ms{color:#3fb950;font-weight:600}
 #speed{margin-top:10px;font-size:.85rem}
 #warn{margin-top:6px;font-size:.8rem;color:#d29922;min-height:1rem}
 .bars{max-width:360px;margin:14px auto 0;text-align:left}
 .row{display:flex;align-items:center;gap:8px;margin:3px 0;font-size:.8rem}
 .row span{width:46px;color:#8b949e}
 .bar{flex:1;height:9px;background:#21262d;border-radius:5px;overflow:hidden}
 .fill{height:100%;background:#1f6feb;width:0}
 .row.top .fill{background:#3fb950}
 .row.top span{color:#3fb950;font-weight:700}
</style></head><body>
<h1>Decí una palabra</h1>
<p class="sub">La reconoce la NPU en la FPGA</p>
<div class="words">Entiende: <b>down go left no right stop up yes</b></div>
<button id="rec" onclick="record()">Grabar 1 segundo</button>
<div id="status"></div>
<div class="vs">
  <div class="card" id="cFpga"><div class="who">FPGA</div><div class="word" id="pFpga">–</div><div class="ms" id="mFpga">—</div></div>
  <div class="card" id="cEsp"><div class="who">ESP32 (software)</div><div class="word" id="pEsp">–</div><div class="ms" id="mEsp">—</div></div>
</div>
<div id="speed"></div>
<div id="warn"></div>
<div class="bars" id="bars"></div>
<script>
const $=id=>document.getElementById(id);
const SR=16000, NS=16000;
async function record(){
  if(!navigator.mediaDevices||!navigator.mediaDevices.getUserMedia){
    $('status').textContent='El navegador bloquea el microfono en paginas http:// — ver las instrucciones del sketch';
    return;
  }
  const btn=$('rec'); btn.disabled=true;
  try{
    $('status').textContent='pidiendo permiso...';
    const stream=await navigator.mediaDevices.getUserMedia({audio:{channelCount:1,
      echoCancellation:false,noiseSuppression:false,autoGainControl:false}});
    const ac=new (window.AudioContext||window.webkitAudioContext)();
    const src=ac.createMediaStreamSource(stream);
    const proc=ac.createScriptProcessor(4096,1,1);
    let chunks=[],total=0;
    const need=Math.ceil(ac.sampleRate*1.0);
    proc.onaudioprocess=e=>{
      if(total>=need)return;
      const d=e.inputBuffer.getChannelData(0);
      chunks.push(new Float32Array(d)); total+=d.length;
    };
    src.connect(proc); proc.connect(ac.destination);
    $('status').textContent='🎤 grabando... ¡hablá ahora!';
    await new Promise(r=>setTimeout(r,1100));
    proc.disconnect(); src.disconnect();
    stream.getTracks().forEach(t=>t.stop());
    // juntar y remuestrear a 16 kHz (interpolacion lineal)
    const all=new Float32Array(total); let o=0;
    for(const c of chunks){all.set(c,o);o+=c.length;}
    const ratio=ac.sampleRate/SR;
    const pcm=new Int16Array(NS);
    for(let i=0;i<NS;i++){
      const p=i*ratio, i0=Math.floor(p), fr=p-i0;
      const a=all[i0]||0, b=all[i0+1]||0;
      let v=(a+(b-a)*fr)*32767;
      pcm[i]=Math.max(-32768,Math.min(32767,v|0));
    }
    ac.close();
    $('status').textContent='calculando...';
    // base64 de PCM int16 (mucho mas liviano que mandar numeros en texto)
    const bytes=new Uint8Array(pcm.buffer);
    let bin=''; for(let i=0;i<bytes.length;i++) bin+=String.fromCharCode(bytes[i]);
    const r=await fetch('/predict',{method:'POST',body:btoa(bin)});
    const j=await r.json();
    $('pFpga').textContent=j.word;   $('mFpga').textContent=j.ms.toFixed(2)+' ms';
    $('pEsp').textContent=j.swWord;  $('mEsp').textContent=j.swMs.toFixed(2)+' ms';
    const fw=j.ms<=j.swMs;
    $('cFpga').className='card'+(fw?' win':''); $('cEsp').className='card'+(fw?'':' win');
    const x=j.swMs/j.ms;
    $('speed').textContent=fw?'La FPGA es '+x.toFixed(2)+'x más rápida'
                             :'El ESP32 es '+(1/x).toFixed(2)+'x más rápido';
    $('status').textContent='MFCC en el ESP32: '+j.mfccMs.toFixed(1)+' ms';
    $('warn').textContent=(j.word!==j.swWord)?'⚠ FPGA y ESP32 no coinciden':'';
    let h='';
    for(let i=0;i<j.scores.length;i++){
      const w=Math.round((j.scores[i]+128)/255*100);
      h+='<div class="row'+(j.words[i]===j.word?' top':'')+'"><span>'+j.words[i]+'</span>'+
         '<div class="bar"><div class="fill" style="width:'+w+'%"></div></div></div>';}
    $('bars').innerHTML=h;
  }catch(e){ $('status').textContent='error: '+e.message; }
  btn.disabled=false;
}
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

static int8_t b64val(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

static float audio_buf[CLIP_SAMPLES];

void handlePredict() {
  String body = server.arg("plain");
  // base64 -> PCM int16 -> float [-1,1]
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

  static int8_t feat[N_FEATURES];
  uint32_t m0 = micros();
  audio_to_features(audio_buf, feat);
  uint32_t mfcc_us = micros() - m0;

  int n_out = LAYER_OUT[NUM_LAYERS - 1];
  int8_t out[16], sw_out[16];
  uint32_t t_load, t_comp;
  fpga_infer(feat, out, &t_load, &t_comp);
  uint32_t s0 = micros();
  sw_infer(feat, sw_out);
  uint32_t sw_us = micros() - s0;

  int pred = argmax(out, n_out), sw_pred = argmax(sw_out, n_out);

  String json = "{\"word\":\"" + String(WORDS[pred]) + "\",\"ms\":" + String(t_comp / 1000.0, 3) +
                ",\"swWord\":\"" + String(WORDS[sw_pred]) + "\",\"swMs\":" + String(sw_us / 1000.0, 3) +
                ",\"mfccMs\":" + String(mfcc_us / 1000.0, 2) + ",\"scores\":[";
  for (int i = 0; i < n_out; i++) { json += String((int)out[i]); if (i < n_out - 1) json += ","; }
  json += "],\"words\":[";
  for (int i = 0; i < NUM_WORDS; i++) { json += "\"" + String(WORDS[i]) + "\""; if (i < NUM_WORDS - 1) json += ","; }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== PALABRAS CLAVE en la NPU (Keras + TFLite int8) ===");
  Serial.printf("Modelo: %d capas (%d", NUM_LAYERS, LAYER_IN[0]);
  for (int L = 0; L < NUM_LAYERS; L++) Serial.printf("-%d", LAYER_OUT[L]);
  Serial.printf("), %d bytes de pesos\n", WEIGHT_STREAM_BYTES);

  wait_pll_lock();
  mfcc_init();
  compute_stream_bases();
  configure_model();
  write_weights_to_sdram();
  load_biases_and_qparams();
  Serial.println("Modelo cargado.");

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
  Serial.println("Para que ande el microfono hay que habilitar el origen en el");
  Serial.println("navegador (ver el comentario del principio del sketch).");
  Serial.println("======================");
}

void loop() { server.handleClient(); }
