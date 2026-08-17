// sdram_p1_poc.ino -- Etapa 4, Fase 1: prueba DIRECTA de la SDRAM embebida
// del Tang Nano 20K, sin ningun cache/buffer de por medio (ver
// top_sdram_p1.v). Objetivo: confirmar que el controlador de SDRAM en si
// funciona -- escribir un patron conocido en un rango de direcciones,
// leerlo de vuelta, comparar byte a byte. Se prueba un rango CHICO cerca
// del principio y otro cerca del FINAL de los 8MB (0x7FFFFF), para
// confirmar que el direccionamiento completo funciona, no solo el
// principio.
//
// Conexionado SPI: igual que los otros sketches (ver npu_poc.ino). Los
// pines de la SDRAM en si no se conectan al ESP32 -- son internos entre la
// FPGA y el chip de SDRAM de la propia placa.

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP            0x00
#define CMD_SDRAM_SET_ADDR 0x01
#define CMD_SDRAM_WR       0x02
#define CMD_SDRAM_RD       0x03

#define SDRAM_TOTAL_BYTES 8388608UL  // 64Mbit = 8MB = 2^23

SPIClass fpga_spi(FSPI);
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

void npu_send(uint8_t cmd, uint16_t a, uint16_t b_raw) {
  uint8_t rx[6];
  spi_burst_begin();
  npu_xfer(cmd, a, b_raw, rx);
  spi_burst_end();
}

// Sondea con CMD_NOP hasta que sdram_busy (bit0 del status) baje. Devuelve
// el ultimo frame recibido (rx[1] tiene el ultimo dato leido, si aplica).
void wait_sdram_ready(uint8_t rx[6]) {
  spi_burst_begin();
  bool busy = true;
  while (busy) {
    npu_xfer(CMD_NOP, 0, 0, rx);
    busy = (rx[0] & 0x01) != 0;
  }
  spi_burst_end();
}

void wait_pll_lock() {
  uint8_t rx[6];
  uint32_t t0 = micros();
  bool locked = false;
  spi_burst_begin();
  for (int i = 0; i < 2000 && !locked; i++) {
    npu_xfer(CMD_NOP, 0, 0, rx);
    locked = (rx[0] & 0x02) != 0;
    if (!locked) delayMicroseconds(100);
  }
  spi_burst_end();
  Serial.printf("PLL lock: %s (%lu us)\n", locked ? "OK" : "NUNCA", (unsigned long)(micros() - t0));
}

void wait_sdram_init() {
  uint8_t rx[6];
  uint32_t t0 = micros();
  wait_sdram_ready(rx); // el controlador arranca con busy=1 durante su init/config interna
  Serial.printf("SDRAM listo (busy=0): tardo %lu us\n", (unsigned long)(micros() - t0));
}

void sdram_set_addr(uint32_t addr23) {
  uint16_t a = (uint16_t)(addr23 & 0xFFFF);
  uint16_t b = (uint16_t)((addr23 >> 16) & 0x7F);
  npu_send(CMD_SDRAM_SET_ADDR, a, b);
}

void sdram_write_byte(uint8_t val) {
  uint8_t rx[6];
  wait_sdram_ready(rx);
  npu_send(CMD_SDRAM_WR, 0, (uint16_t)val);
}

// g_last_busy_cyc: cuantos ciclos de clk_sys (54MHz) estuvo ocupado el
// controlador en la ULTIMA operacion completada (bytes 2-5 de la
// respuesta, 32 bits). Una lectura normal deberia dar ~5-6 ciclos; si
// aparece un numero gigante, confirma un atasco real del lado FPGA (no
// del lado ESP32/WiFi).
uint32_t g_last_busy_cyc = 0;

uint8_t sdram_read_byte() {
  uint8_t rx[6];
  wait_sdram_ready(rx);
  npu_send(CMD_SDRAM_RD, 0, 0);
  wait_sdram_ready(rx);
  g_last_busy_cyc = ((uint32_t)rx[2] << 24) | ((uint32_t)rx[3] << 16) | ((uint32_t)rx[4] << 8) | rx[5];
  return rx[1];
}

// Vuelve a leer la MISMA direccion 'n' veces (sin reescribir) para
// distinguir un dato mal escrito (siempre da el mismo valor incorrecto)
// de una lectura marginal (el valor salta de una lectura a otra).
void reread_probe(uint32_t addr, int n) {
  Serial.printf("    reread x%d en 0x%06lX: ", n, (unsigned long)addr);
  for (int i = 0; i < n; i++) {
    sdram_set_addr(addr);
    uint8_t v = sdram_read_byte();
    Serial.printf("%3d ", v);
  }
  Serial.println();
}

// Escribe 'len' bytes con el patron pattern(i) empezando en base_addr, los
// lee de vuelta, y compara. Imprime cuantos coincidieron.
void test_range(const char *label, uint32_t base_addr, int len) {
  Serial.printf("=== %s: base=0x%06lX len=%d ===\n", label, (unsigned long)base_addr, len);

  uint32_t t0 = micros();
  sdram_set_addr(base_addr);
  for (int i = 0; i < len; i++) {
    uint8_t val = (uint8_t)((i * 37 + 11) & 0xFF); // patron simple, no trivial
    sdram_write_byte(val);
  }
  uint32_t t_write = micros() - t0;

  sdram_set_addr(base_addr);
  int mism = 0;
  uint32_t slow_count = 0;      // bytes cuya lectura individual tardo != lo tipico
  uint32_t max_byte_us = 0;
  int max_byte_idx = -1;
  uint32_t max_busy_cyc = 0;
  int max_busy_idx = -1;
  uint32_t t1 = micros();
  for (int i = 0; i < len; i++) {
    uint8_t expected = (uint8_t)((i * 37 + 11) & 0xFF);
    uint32_t expected_addr = base_addr + i;

    uint32_t tb0 = micros();
    uint8_t got = sdram_read_byte();
    uint32_t byte_us = micros() - tb0;

    if (byte_us > 200) slow_count++;  // tipico es ~80us; 200us ya es sospechoso
    if (byte_us > max_byte_us) { max_byte_us = byte_us; max_byte_idx = i; }
    if (g_last_busy_cyc > max_busy_cyc) { max_busy_cyc = g_last_busy_cyc; max_busy_idx = i; }

    if (got != expected) {
      mism++;
      if (mism <= 10) {
        // g_last_busy_cyc: cuantos ciclos de clk_sys estuvo ocupado el
        // controlador en ESTA lectura. Una lectura normal da ~5-6 ciclos;
        // si aparece un numero gigante, hay un atasco real del lado FPGA
        // coincidiendo con el dato corrupto.
        Serial.printf("  [%4d] esperado=%3d fpga=%3d  tardo=%luus busy_cyc=%lu\n",
                      i, expected, got, (unsigned long)byte_us, (unsigned long)g_last_busy_cyc);
        reread_probe(base_addr + i, 5);
      }
    }
  }
  uint32_t t_read = micros() - t1;

  Serial.printf("%s: %d/%d coinciden  (escritura: %.2fms, lectura: %.2fms, %.2fus/byte lectura)\n",
                label, len - mism, len, t_write / 1000.0, t_read / 1000.0, (float)t_read / len);
  Serial.printf("  lecturas lentas (>200us): %lu/%d -- peor caso: byte[%d]=%luus, busy_cyc[%d]=%lu\n\n",
                (unsigned long)slow_count, len, max_byte_idx, (unsigned long)max_byte_us,
                max_busy_idx, (unsigned long)max_busy_cyc);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: Etapa 4 Fase 1 -- SDRAM directa ===\n");
  wait_pll_lock();
  wait_sdram_init();

  test_range("Rango bajo", 0, 256);
  test_range("Rango alto", SDRAM_TOTAL_BYTES - 256, 256);

  Serial.println("=== Fase 1 completa ===");
}

void loop() {
}
