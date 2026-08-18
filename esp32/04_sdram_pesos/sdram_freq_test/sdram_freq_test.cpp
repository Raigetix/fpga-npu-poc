// sdram_freq_test.cpp -- orquestador ESP32 para la caracterizacion de
// frecuencia de la SDRAM embebida (ver plan_testing_sdram.md y
// fpga_project/src/04_sdram_pesos/sdram_test_harness.v / top_sdram_freqtest.v
// para el diseño del lado FPGA).
//
// La frecuencia de clk_sys queda fija por BITSTREAM (top_freqtest_27/36/
// 45/54.v, un .fs distinto por candidato -- el chip solo tiene 2 PLLs
// fisicos, no alcanzan para conmutar en vivo entre 4). Este firmware NO
// sabe que frecuencia esta corriendo la FPGA: se lo decis vos por Serial
// al arrancar (una etiqueta de texto, solo para que quede en el log), y la
// frecuencia real la determina cual .fs este grabado en ese momento.
//
// GATE DE ARRANQUE: el primer Serial.available() no llega hasta que el
// monitor serie esta conectado y manda algo -- asi el orquestador (que
// puede reprogramarse y ejecutarse de forma no interactiva/automatica)
// nunca imprime nada antes de que haya alguien mirando. Mandale una linea
// de texto con la etiqueta de la frecuencia bajo prueba (ej. "54MHz") y
// arranca.
#include <Arduino.h>
#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10
// SPI mas lento que el de la demo de KWS (8MHz) a proposito: el candidato
// mas lento de este barrido es clk_sys=27MHz, y spi_slave.v cruza cs_n/sclk
// a su dominio 'clk' con un sincronizador de 2 etapas -- 1MHz de SPI le da
// un margen comodo (>25:1) en TODOS los candidatos, no solo en el de 54MHz
// para el que la demo normal esta afinada.
#define SPI_HZ 1000000

#define CMD_NOP          0x00
#define CMD_TEST_CONFIG  0x11
#define CMD_SET_SEED     0x12
#define CMD_TEST_START   0x13
#define CMD_TEST_RD      0x14
#define CMD_TEST_LOG_RD  0x15

SPIClass fpga_spi(FSPI);

void spi_begin() { fpga_spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0)); }
void spi_end()   { fpga_spi.endTransaction(); }

void xfer_narrow(uint8_t cmd, uint16_t a, uint16_t b, uint8_t rx[6]) {
  uint8_t tx[6] = {cmd, (uint8_t)(a >> 8), (uint8_t)a, (uint8_t)(b >> 8), (uint8_t)b, 0};
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}
void xfer_wide(uint8_t cmd, uint32_t v, uint8_t tail, uint8_t rx[6]) {
  uint8_t tx[6] = {cmd, (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v, tail};
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
}
uint8_t send_narrow(uint8_t cmd, uint16_t a, uint16_t b = 0) {
  uint8_t rx[6]; spi_begin(); xfer_narrow(cmd, a, b, rx); spi_end(); return rx[0];
}
uint8_t send_wide(uint8_t cmd, uint32_t v, uint8_t tail = 0) {
  uint8_t rx[6]; spi_begin(); xfer_wide(cmd, v, tail, rx); spi_end(); return rx[0];
}
// Lee un byte de "debug" (CMD_TEST_RD / CMD_TEST_LOG_RD): el resultado no
// sale en la MISMA trama que lo pide -- top_sdram_freqtest.v congela
// tx_snapshot durante cada trama y lo actualiza recien al soltar CS, asi
// que hace falta mandar la trama que pide el dato y despues una trama NOP
// para levantar la respuesta (mismo patron que CMD_DBG_RD en top_sdram_p3.v).
uint8_t read_dbg_byte(uint8_t cmd, uint16_t a, uint16_t b = 0) {
  uint8_t rx[6];
  spi_begin();
  xfer_narrow(cmd, a, b, rx);
  xfer_narrow(CMD_NOP, 0, 0, rx);
  spi_end();
  return rx[5];
}
uint8_t status_byte() {
  uint8_t rx[6]; spi_begin(); xfer_narrow(CMD_NOP, 0, 0, rx); spi_end(); return rx[0];
}
bool pll_locked()  { return (status_byte() & 0x02) != 0; }
bool test_is_busy(){ return (status_byte() & 0x04) != 0; }

void test_config(uint16_t addr_max, uint32_t seed) {
  send_narrow(CMD_TEST_CONFIG, addr_max, 0);
  send_wide(CMD_SET_SEED, seed, 0);
}
void test_start(uint8_t phase, uint16_t reps) {
  // fase ahora ocupa 2 bits (0=A 1=B 2=C-rafaga), repeticiones 14 bits
  // (ver top_sdram_freqtest.v).
  uint16_t a = ((reps & 0x3FFF) << 2) | (phase & 3);
  send_narrow(CMD_TEST_START, a, 0);
}

uint8_t read_summary_byte(uint16_t idx) {          // sel=1 (ver top_sdram_freqtest.v)
  uint16_t a = (uint16_t)((1u << 11) | (idx & 0x7FF));
  return read_dbg_byte(CMD_TEST_RD, a, 0);
}
uint32_t read_summary_u32(uint16_t base_idx) {     // 4 bytes consecutivos, LSB primero
  uint32_t v = 0;
  for (int i = 0; i < 4; i++) v |= ((uint32_t)read_summary_byte(base_idx + i)) << (8 * i);
  return v;
}

struct LogEntry { uint32_t addr; uint8_t expected, actual; uint16_t pass; };
LogEntry read_log_entry(uint8_t idx) {
  LogEntry e;
  uint8_t b0 = read_dbg_byte(CMD_TEST_LOG_RD, idx, 0);
  uint8_t b1 = read_dbg_byte(CMD_TEST_LOG_RD, idx, 1);
  uint8_t b2 = read_dbg_byte(CMD_TEST_LOG_RD, idx, 2);
  e.expected  = read_dbg_byte(CMD_TEST_LOG_RD, idx, 3);
  e.actual    = read_dbg_byte(CMD_TEST_LOG_RD, idx, 4);
  uint8_t p0  = read_dbg_byte(CMD_TEST_LOG_RD, idx, 5);
  uint8_t p1  = read_dbg_byte(CMD_TEST_LOG_RD, idx, 6);
  e.addr = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
  e.pass = ((uint16_t)p1 << 8) | p0;
  return e;
}

// ================= Parametros de la corrida =================
// Rango de prueba: 16KB desde la direccion 0 (varias filas/bancos de la
// SDRAM, sin llegar a pisar donde vivirian los pesos reales en la demo).
const uint16_t ADDR_MAX = 16384;
const uint32_t SEED     = 0xA5C3F17Bu;
const uint16_t REPS     = 200;
const int      RUNS     = 3;   // corridas independientes, para ver si las
                                // MISMAS direcciones fallan cada vez (celda
                                // debil, ver plan_testing_sdram.md) o si
                                // cambian (ruido de margen de tiempo)

void run_phase(int run, uint8_t phase, uint32_t &out_cnt, uint32_t &out_total) {
  unsigned long t0 = millis();
  test_start(phase, REPS);
  delay(2);
  while (test_is_busy()) delay(1);
  unsigned long dt = millis() - t0;

  uint32_t cnt   = read_summary_u32(phase == 0 ? 0 : 4);
  uint32_t total = read_summary_u32(phase == 0 ? 8 : 12);
  float pct = total ? (100.0f * (float)cnt / (float)total) : 0.0f;
  Serial.printf("  Fase %c: %lu/%lu bytes mal (%.4f%%), %lu ms\n",
                phase == 0 ? 'A' : 'B', (unsigned long)cnt, (unsigned long)total, pct, dt);

  uint32_t n_show = cnt < 256 ? cnt : 256;
  if (n_show > 0) {
    if (cnt > 256)
      Serial.printf("  (log circular de 256 -- se muestran las ULTIMAS %lu fallas de %lu totales)\n",
                    (unsigned long)n_show, (unsigned long)cnt);
    for (uint32_t i = 0; i < n_show; i++) {
      LogEntry e = read_log_entry((uint8_t)i);
      Serial.printf("    addr=0x%06lX esperado=0x%02X leido=0x%02X pasada=%u\n",
                    (unsigned long)e.addr, e.expected, e.actual, e.pass);
    }
  }
  out_cnt = cnt; out_total = total;
}

// ---- Fase C: validacion aislada de rd_burst2 (ver sdram.v / sdram_test_harness.v).
// addr_max debe ser multiplo de 8 y no pasar de 1024 (una fila completa)
// para que la rafaga de 2 palabras nunca cruce el limite de fila. Reusa
// el mismo slot de contadores que la fase A (0 y 8), asi que no hace
// falta agregar nada nuevo del lado del protocolo. ----
const uint16_t ADDR_MAX_C = 1024;
const uint16_t REPS_C     = 500;
const int      RUNS_C     = 3;

void run_phase_c(uint32_t &out_cnt, uint32_t &out_total) {
  unsigned long t0 = millis();
  test_start(2, REPS_C);
  delay(2);
  while (test_is_busy()) delay(1);
  unsigned long dt = millis() - t0;

  uint32_t cnt   = read_summary_u32(0);
  uint32_t total = read_summary_u32(8);
  float pct = total ? (100.0f * (float)cnt / (float)total) : 0.0f;
  Serial.printf("  Fase C (rafaga de 2): %lu/%lu bytes mal (%.4f%%), %lu ms\n",
                (unsigned long)cnt, (unsigned long)total, pct, dt);

  uint32_t n_show = cnt < 256 ? cnt : 256;
  if (n_show > 0) {
    if (cnt > 256)
      Serial.printf("  (log circular de 256 -- se muestran las ULTIMAS %lu fallas de %lu totales)\n",
                    (unsigned long)n_show, (unsigned long)cnt);
    for (uint32_t i = 0; i < n_show; i++) {
      LogEntry e = read_log_entry((uint8_t)i);
      Serial.printf("    addr=0x%06lX esperado=0x%02X leido=0x%02X pasada=%u\n",
                    (unsigned long)e.addr, e.expected, e.actual, e.pass);
    }
  }
  out_cnt = cnt; out_total = total;
}

void run_burst2_validation(const String &label) {
  Serial.printf("addr_max=%u seed=0x%08lX reps=%u corridas=%d\n",
                ADDR_MAX_C, (unsigned long)SEED, REPS_C, RUNS_C);
  uint32_t sum_cnt = 0, sum_total = 0;
  for (int run = 1; run <= RUNS_C; run++) {
    Serial.printf("--- Corrida %d/%d ---\n", run, RUNS_C);
    test_config(ADDR_MAX_C, SEED);
    uint32_t cnt, total;
    run_phase_c(cnt, total);
    sum_cnt += cnt; sum_total += total;
    Serial.println();
  }
  Serial.println("=== Resumen final (fase C, rafaga de 2) ===");
  Serial.printf("Etiqueta: %s\n", label.c_str());
  Serial.printf("Total: %lu/%lu (%.4f%%)\n",
                (unsigned long)sum_cnt, (unsigned long)sum_total,
                sum_total ? 100.0f * sum_cnt / sum_total : 0.0f);
  Serial.println("=== Fin ===");
}

void setup() {
  Serial.begin(115200);
  while (!Serial.available()) { delay(10); }
  String label = Serial.readStringUntil('\n');
  label.trim();
  if (label.length() == 0) label = "?";

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println();
  Serial.println("=== sdram_freq_test -- caracterizacion de frecuencia SDRAM ===");
  Serial.printf("Etiqueta de frecuencia (segun el .fs grabado): %s\n", label.c_str());
  Serial.printf("addr_max=%u seed=0x%08lX reps=%u corridas=%d\n",
                ADDR_MAX, (unsigned long)SEED, REPS, RUNS);

  if (!pll_locked()) {
    Serial.println("ERROR: el PLL no engancho (bit1 de status en 0). Abortando.");
    while (1) delay(1000);
  }
  Serial.println("PLL lock: OK\n");

  run_burst2_validation(label);
}

void loop() { delay(1000); }
