// mnist_npu_poc.ino -- Etapa 4: MNIST real corriendo en la NPU de la FPGA.
//
// Hace DOS cosas, en este orden:
//   1) BENCHMARK: 300 imagenes reales del set de test de MNIST, comparando
//      la FPGA contra el MISMO modelo cuantizado corriendo en software en
//      el ESP32 (precision y tiempos).
//   2) DEMO WEB: levanta un punto de acceso WiFi y sirve una pagina donde
//      se puede dibujar un digito con el mouse o el dedo; la imagen se
//      preprocesa como MNIST, se manda a la FPGA y se muestra que digito
//      cree que es.
//
// El modelo (784 -> 128 -> 64 -> 10, int8 per-tensor) esta en
// mnist_model.h, generado por tools/train_mnist_npu.py a partir de Keras +
// el conversor estandar de TFLite. Son 109.568 bytes de pesos: mas del
// doble de lo que entraba en la BRAM de la placa, por eso viven en SDRAM y
// se transmiten durante el computo.
//
// COMO USAR LA DEMO WEB: conectarse a la red WiFi "NPU-MNIST" (clave
// npu12345) desde el celular o la compu, y abrir http://192.168.4.1

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include "mnist_model.h"

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
// Poné acá los datos de TU red. Si WIFI_SSID queda vacio, o si no logra
// conectarse en 15 segundos, cae automaticamente a crear su propia red
// (asi nunca te quedas sin poder entrar a la pagina).
const char *WIFI_SSID = "Fibertel WiFi311 2.4GHz";  // <-- el nombre de tu red WiFi
const char *WIFI_PASS = "00417751437";              // <-- la clave de tu red WiFi

const char *AP_SSID = "NPU-MNIST";   // red propia, solo si falla lo de arriba
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
uint16_t bias_base_of_layer[NUM_LAYERS];

void configure_model() {
  npu_send(CMD_SET_NUM_LAYERS, NUM_LAYERS, 0);
  uint16_t bb = 0;
  for (int L = 0; L < NUM_LAYERS; L++) {
    bias_base_of_layer[L] = bb;
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((0 << 4) | L), LAYER_IN[L]);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((1 << 4) | L), LAYER_OUT[L]);
    npu_send(CMD_SET_LAYER_SHAPE, (uint16_t)((2 << 4) | L), bb);
    bb += LAYER_OUT[L];
  }
}

// La escritura a SDRAM falla ~0.004% de los bytes; UN peso corrupto en la
// primera capa arruina TODAS las inferencias, asi que se verifica y se
// reintenta. Ademas la LECTURA por SPI falla ~0.15%, asi que cada
// sospechoso se confirma con relecturas antes de darlo por malo (si no,
// aparecerian ~150 errores inexistentes en 109.000 bytes).
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
  npu_xfer(CMD_SDRAM_RD, 0, 0, rx);          // pedido del byte 0
  for (int i = 0; i < WEIGHT_STREAM_BYTES; i++) {
    npu_xfer(CMD_SDRAM_RD, 0, 0, rx);        // pide i+1, trae i
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
    if (sdram_read_at(a) == want || sdram_read_at(a) == want) continue; // falso positivo
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
void fpga_infer(const int8_t *img, int8_t *out, uint32_t *t_load, uint32_t *t_compute) {
  uint8_t rx[6];
  uint32_t t0 = micros();
  npu_send(CMD_SET_ITGT, 0, 0);
  uint8_t buf[5]; int bufcount = 0;
  spi_burst_begin();
  for (int i = 0; i < IMG_SIZE; i++) {
    buf[bufcount++] = (uint8_t)img[i];
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

// El MISMO modelo en software, para comparar. Los pesos estan en el layout
// intercalado del streaming: capa L, neurona n (ola n/8, carril n%8),
// entrada i -> base_capa + ((n/8)*in_count + i)*8 + (n%8)
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

void sw_infer(const int8_t *img, int8_t *out) {
  const int8_t *src = img;
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

// ================= Benchmark =================
void run_benchmark() {
  int n_out = LAYER_OUT[NUM_LAYERS - 1];
  int8_t fpga_out[16], sw_out[16];
  int fpga_vs_ref = 0, sw_vs_ref = 0, fpga_vs_label = 0, ref_vs_label = 0;
  uint64_t fpga_us = 0, sw_us = 0, load_us = 0;
  uint32_t fpga_min = 0xFFFFFFFF, fpga_max = 0, sw_min = 0xFFFFFFFF, sw_max = 0;

  Serial.printf("\n=== BENCHMARK: %d imagenes reales de MNIST ===\n", N_TEST_IMAGES);
  for (int k = 0; k < N_TEST_IMAGES; k++) {
    const int8_t *img = &TEST_IMAGES[(uint32_t)k * IMG_SIZE];

    uint32_t t_load, t_comp;
    fpga_infer(img, fpga_out, &t_load, &t_comp);

    uint32_t s0 = micros();
    sw_infer(img, sw_out);
    uint32_t s1 = micros();
    uint32_t sw_t = s1 - s0;

    int p_fpga = argmax(fpga_out, n_out);
    int p_sw   = argmax(sw_out, n_out);
    int ref    = TFLITE_PRED[k];
    int label  = TEST_LABELS[k];

    if (p_fpga == ref)   fpga_vs_ref++;
    if (p_sw == ref)     sw_vs_ref++;
    if (p_fpga == label) fpga_vs_label++;
    if (ref == label)    ref_vs_label++;

    fpga_us += t_comp; sw_us += sw_t; load_us += t_load;
    if (t_comp < fpga_min) fpga_min = t_comp;
    if (t_comp > fpga_max) fpga_max = t_comp;
    if (sw_t < sw_min) sw_min = sw_t;
    if (sw_t > sw_max) sw_max = sw_t;

    if (p_fpga != ref)
      Serial.printf("  [%3d] DIFIERE de la referencia: FPGA=%d ref=%d (etiqueta %d)\n", k, p_fpga, ref, label);
    if ((k + 1) % 50 == 0) Serial.printf("  ...%d/%d\n", k + 1, N_TEST_IMAGES);
  }

  Serial.println("\n========================================");
  Serial.printf("Coincidencia con la referencia de la PC (lo que mide si el hardware es exacto):\n");
  Serial.printf("   FPGA        %d/%d  (%.2f%%)\n", fpga_vs_ref, N_TEST_IMAGES, 100.0 * fpga_vs_ref / N_TEST_IMAGES);
  Serial.printf("   ESP32 (sw)  %d/%d  (%.2f%%)\n", sw_vs_ref, N_TEST_IMAGES, 100.0 * sw_vs_ref / N_TEST_IMAGES);
  Serial.printf("\nPrecision sobre la etiqueta real:\n");
  Serial.printf("   FPGA           %d/%d  (%.2f%%)\n", fpga_vs_label, N_TEST_IMAGES, 100.0 * fpga_vs_label / N_TEST_IMAGES);
  Serial.printf("   modelo en PC   %d/%d  (%.2f%%)  <- techo del modelo cuantizado\n",
                ref_vs_label, N_TEST_IMAGES, 100.0 * ref_vs_label / N_TEST_IMAGES);
  Serial.printf("\nTiempo por inferencia:\n");
  Serial.printf("   FPGA    prom %.2f ms  (min %.2f / max %.2f)\n",
                fpga_us / 1000.0 / N_TEST_IMAGES, fpga_min / 1000.0, fpga_max / 1000.0);
  Serial.printf("   ESP32   prom %.2f ms  (min %.2f / max %.2f)\n",
                sw_us / 1000.0 / N_TEST_IMAGES, sw_min / 1000.0, sw_max / 1000.0);
  Serial.printf("   -> la FPGA es %.2fx mas rapida en el computo\n", (double)sw_us / (double)fpga_us);
  Serial.printf("   (carga de la imagen por SPI, aparte: %.2f ms)\n", load_us / 1000.0 / N_TEST_IMAGES);
  Serial.println("========================================");
}

// ================= Demo web =================
// La pagina hace el MISMO preprocesado que MNIST (escalar a 20x20 y centrar
// por CENTRO DE MASA en 28x28); sin eso, un trazo crudo baja mucho la
// precision aunque el modelo este perfecto.
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>NPU FPGA - MNIST</title>
<style>
 *{box-sizing:border-box}
 body{margin:0;padding:16px;background:#0e1116;color:#e6edf3;
      font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;text-align:center}
 h1{font-size:1.1rem;font-weight:600;margin:0 0 4px}
 p.sub{margin:0 0 16px;color:#8b949e;font-size:.85rem}
 canvas{background:#000;border-radius:12px;touch-action:none;
        border:2px solid #30363d;max-width:100%;height:auto}
 .btns{margin:14px 0}
 button{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:8px;
        padding:10px 20px;font-size:.95rem;margin:0 4px;cursor:pointer}
 button.primary{background:#1f6feb;border-color:#1f6feb}
 button:active{transform:translateY(1px)}
 #meta{color:#8b949e;font-size:.8rem;min-height:1.1rem;margin-bottom:10px}
 .vs{display:flex;gap:10px;max-width:340px;margin:10px auto 0}
 .card{flex:1;background:#161b22;border:1px solid #30363d;border-radius:10px;padding:10px 6px}
 .card.win{border-color:#3fb950}
 .card .who{font-size:.72rem;color:#8b949e;letter-spacing:.04em}
 .card .digit{font-size:2.6rem;font-weight:700;line-height:1.1}
 .card .ms{font-size:.8rem;color:#8b949e}
 .card.win .ms{color:#3fb950;font-weight:600}
 #speed{margin-top:10px;font-size:.85rem;color:#e6edf3}
 #warn{margin-top:6px;font-size:.8rem;color:#d29922;min-height:1rem}
 .bars{max-width:340px;margin:14px auto 0;text-align:left}
 .row{display:flex;align-items:center;gap:8px;margin:3px 0;font-size:.8rem}
 .row span{width:14px;color:#8b949e}
 .bar{flex:1;height:9px;background:#21262d;border-radius:5px;overflow:hidden}
 .fill{height:100%;background:#1f6feb;width:0}
 .row.top .fill{background:#3fb950}
 .row.top span{color:#3fb950;font-weight:700}
</style></head><body>
<h1>Dibujá un dígito</h1>
<p class="sub">Lo clasifica la NPU en la FPGA</p>
<canvas id="cv" width="280" height="280"></canvas>
<div class="btns">
  <button class="primary" onclick="predict()">Predecir</button>
  <button onclick="clearCv()">Borrar</button>
</div>
<div id="meta"></div>
<div class="vs">
  <div class="card" id="cFpga"><div class="who">FPGA</div><div class="digit" id="pFpga">–</div><div class="ms" id="mFpga">—</div></div>
  <div class="card" id="cEsp"><div class="who">ESP32 (software)</div><div class="digit" id="pEsp">–</div><div class="ms" id="mEsp">—</div></div>
</div>
<div id="speed"></div>
<div id="warn"></div>
<div class="bars" id="bars"></div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
const tmp=document.createElement('canvas');tmp.width=28;tmp.height=28;
const tctx=tmp.getContext('2d',{willReadFrequently:true});
let drawing=false;
function $(id){return document.getElementById(id);}
function clearCv(){ctx.fillStyle='#000';ctx.fillRect(0,0,cv.width,cv.height);
  $('pFpga').textContent='–';$('pEsp').textContent='–';
  $('mFpga').textContent='—';$('mEsp').textContent='—';
  $('cFpga').className='card';$('cEsp').className='card';
  $('meta').textContent='';$('speed').textContent='';$('warn').textContent='';
  $('bars').innerHTML='';}
clearCv();
ctx.strokeStyle='#fff';ctx.lineWidth=20;ctx.lineCap='round';ctx.lineJoin='round';
function pos(e){const r=cv.getBoundingClientRect();
  const t=e.touches?e.touches[0]:e;
  return [(t.clientX-r.left)*cv.width/r.width,(t.clientY-r.top)*cv.height/r.height];}
function start(e){e.preventDefault();drawing=true;const[x,y]=pos(e);ctx.beginPath();ctx.moveTo(x,y);}
function move(e){if(!drawing)return;e.preventDefault();const[x,y]=pos(e);ctx.lineTo(x,y);ctx.stroke();}
function end(e){drawing=false;}
cv.addEventListener('mousedown',start);cv.addEventListener('mousemove',move);
window.addEventListener('mouseup',end);
cv.addEventListener('touchstart',start);cv.addEventListener('touchmove',move);
cv.addEventListener('touchend',end);

// Preprocesado estilo MNIST: recortar al trazo, escalar a 20x20 y centrar
// por centro de masa dentro de 28x28.
function extract28(){
  const W=cv.width,H=cv.height,d=ctx.getImageData(0,0,W,H).data;
  let minX=W,minY=H,maxX=-1,maxY=-1;
  for(let y=0;y<H;y++)for(let x=0;x<W;x++){
    if(d[(y*W+x)*4]>30){if(x<minX)minX=x;if(x>maxX)maxX=x;if(y<minY)minY=y;if(y>maxY)maxY=y;}}
  if(maxX<0)return null;
  const bw=maxX-minX+1,bh=maxY-minY+1,s=20/Math.max(bw,bh);
  const nw=Math.max(1,Math.round(bw*s)),nh=Math.max(1,Math.round(bh*s));
  const ox=(28-nw)/2,oy=(28-nh)/2;
  tctx.fillStyle='#000';tctx.fillRect(0,0,28,28);
  tctx.drawImage(cv,minX,minY,bw,bh,ox,oy,nw,nh);
  let td=tctx.getImageData(0,0,28,28).data,sum=0,cx=0,cy=0;
  for(let y=0;y<28;y++)for(let x=0;x<28;x++){const v=td[(y*28+x)*4];sum+=v;cx+=x*v;cy+=y*v;}
  if(!sum)return null;
  const dx=Math.round(13.5-cx/sum),dy=Math.round(13.5-cy/sum);
  tctx.fillStyle='#000';tctx.fillRect(0,0,28,28);
  tctx.drawImage(cv,minX,minY,bw,bh,ox+dx,oy+dy,nw,nh);
  td=tctx.getImageData(0,0,28,28).data;
  const out=new Array(784);
  for(let i=0;i<784;i++)out[i]=td[i*4];
  return out;
}
async function predict(){
  const px=extract28();
  if(!px){$('meta').textContent='Dibujá algo primero';return;}
  $('meta').textContent='calculando...';
  try{
    const r=await fetch('/predict',{method:'POST',body:px.join(',')});
    const j=await r.json();
    $('pFpga').textContent=j.pred;   $('mFpga').textContent=j.ms.toFixed(2)+' ms';
    $('pEsp').textContent=j.swPred;  $('mEsp').textContent=j.swMs.toFixed(2)+' ms';
    // el mas rapido queda resaltado
    const fpgaWins=j.ms<=j.swMs;
    $('cFpga').className='card'+(fpgaWins?' win':'');
    $('cEsp').className='card'+(fpgaWins?'':' win');
    const x=j.swMs/j.ms;
    $('speed').textContent=fpgaWins
      ? 'La FPGA es '+x.toFixed(2)+'x más rápida'
      : 'El ESP32 es '+(1/x).toFixed(2)+'x más rápido';
    $('meta').textContent='carga de la imagen por SPI: '+j.loadMs.toFixed(2)+' ms (aparte del cómputo)';
    // los dos corren el MISMO modelo: si difieren, algo anda mal
    $('warn').textContent=(j.pred!==j.swPred)
      ? '⚠ FPGA y ESP32 no coinciden (deberían dar lo mismo)' : '';
    let h='';
    for(let i=0;i<10;i++){
      const v=j.scores[i],w=Math.round((v+128)/255*100);
      h+='<div class="row'+(i==j.pred?' top':'')+'"><span>'+i+'</span>'+
         '<div class="bar"><div class="fill" style="width:'+w+'%"></div></div></div>';}
    $('bars').innerHTML=h;
  }catch(e){$('meta').textContent='error: '+e;}
}
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handlePredict() {
  String body = server.arg("plain");
  static int8_t img[784];
  int n = 0, val = 0;
  bool have = false;
  for (unsigned int i = 0; i <= body.length() && n < 784; i++) {
    char c = (i < body.length()) ? body[i] : ',';
    if (c >= '0' && c <= '9') { val = val * 10 + (c - '0'); have = true; }
    else if (c == ',') {
      if (have) {
        // pixel 0..255 -> normalizado 0..1 -> cuantizado como espera el modelo
        float px = val / 255.0f;
        int q = (int)lroundf(px / INPUT_SCALE) + INPUT_ZP;
        if (q < -128) q = -128;
        if (q > 127) q = 127;
        img[n++] = (int8_t)q;
      }
      val = 0; have = false;
    }
  }
  if (n < 784) { server.send(400, "application/json", "{\"error\":\"faltan pixeles\"}"); return; }

  int n_out = LAYER_OUT[NUM_LAYERS - 1];

  // Mismo digito por los dos caminos, para comparar en vivo.
  int8_t out[16];
  uint32_t t_load, t_comp;
  fpga_infer(img, out, &t_load, &t_comp);

  int8_t sw_out[16];
  uint32_t s0 = micros();
  sw_infer(img, sw_out);
  uint32_t sw_us = micros() - s0;

  int pred    = argmax(out, n_out);
  int sw_pred = argmax(sw_out, n_out);

  String json = "{\"pred\":" + String(pred) +
                ",\"ms\":" + String(t_comp / 1000.0, 3) +
                ",\"swPred\":" + String(sw_pred) +
                ",\"swMs\":" + String(sw_us / 1000.0, 3) +
                ",\"loadMs\":" + String(t_load / 1000.0, 3) +
                ",\"scores\":[";
  for (int i = 0; i < n_out; i++) { json += String((int)out[i]); if (i < n_out - 1) json += ","; }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== MNIST REAL en la NPU (Keras + TFLite int8) ===");
  Serial.printf("Modelo: %d capas (%d", NUM_LAYERS, LAYER_IN[0]);
  for (int L = 0; L < NUM_LAYERS; L++) Serial.printf("-%d", LAYER_OUT[L]);
  Serial.printf("), %d bytes de pesos\n\n", WEIGHT_STREAM_BYTES);

  wait_pll_lock();
  compute_stream_bases();
  configure_model();
  write_weights_to_sdram();
  load_biases_and_qparams();
  Serial.println("Modelo cargado.");

  run_benchmark();

  String url;
  bool connected = false;
  if (strlen(WIFI_SSID) > 0) {
    Serial.printf("\nConectando a la red \"%s\"", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
    Serial.println();
    connected = (WiFi.status() == WL_CONNECTED);
    if (connected) url = WiFi.localIP().toString();
    else Serial.println("No se pudo conectar, creando red propia...");
  }
  if (!connected) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    url = WiFi.softAPIP().toString();
  }

  server.on("/", handleRoot);
  server.on("/predict", HTTP_POST, handlePredict);
  server.begin();

  Serial.println("\n=== DEMO WEB LISTA ===");
  if (connected) Serial.printf("Conectado a \"%s\"\n", WIFI_SSID);
  else           Serial.printf("Conectate a la red WiFi \"%s\" (clave %s)\n", AP_SSID, AP_PASS);
  Serial.printf("Abri:  http://%s\n", url.c_str());
  Serial.println("======================");
}

void loop() {
  server.handleClient();
}
