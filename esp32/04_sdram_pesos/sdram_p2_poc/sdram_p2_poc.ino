// sdram_p2_poc.ino -- Etapa 4, Fase 2: prueba del motor de precarga
// verificada (SDRAM -> buffer de BRAM), ver top_sdram_p2.v. Objetivo:
// confirmar que, con el mecanismo de lectura verificada (hasta 3 lecturas
// por byte, mayoria), el bug de datos intermitente de Fase 1 (~0.3-0.5%,
// ver README.md seccion Etapa 4) queda resuelto EN LA PRACTICA aunque no
// se haya identificado su causa fisica exacta.
//
// Flujo: escribe un patron conocido directo en SDRAM (mismo camino de
// Fase 1: CMD_SDRAM_SET_ADDR + CMD_SDRAM_WR), dispara una precarga
// verificada de todo ese rango al buffer, y compara el buffer resultante
// byte a byte contra el patron esperado -- ademas reporta cuantos bytes
// necesitaron una segunda lectura (pf_retry_count) y cuantos las 3
// lecturas salieron distintas entre si (pf_fail3_count, deberia dar
// siempre 0).

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP                0x00
#define CMD_SDRAM_SET_ADDR     0x01
#define CMD_SDRAM_WR           0x02
#define CMD_PREFETCH_SET_BASE  0x10
#define CMD_PREFETCH_SET_LEN   0x11
#define CMD_PREFETCH_START     0x12
#define CMD_PREFETCH_RD_BYTE   0x13
#define CMD_PREFETCH_SET_RDPTR 0x14

#define BUF_DEPTH 4096UL

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

// Ultima respuesta completa capturada por wait_ready(), para que el
// llamador pueda leer buf_rd_out/pf_retry_count/pf_fail3_count justo
// despues de que busy bajo.
uint8_t g_last_rx[6];

// Sondea con CMD_NOP hasta que busy (bit0 del status) baje.
void wait_ready() {
  spi_burst_begin();
  bool busy = true;
  while (busy) {
    npu_xfer(CMD_NOP, 0, 0, g_last_rx);
    busy = (g_last_rx[0] & 0x01) != 0;
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
  uint32_t t0 = micros();
  wait_ready(); // el controlador arranca con busy=1 durante su init/config interna
  Serial.printf("SDRAM lista (busy=0): tardo %lu us\n", (unsigned long)(micros() - t0));
}

void sdram_set_addr(uint32_t addr23) {
  uint16_t a = (uint16_t)(addr23 & 0xFFFF);
  uint16_t b = (uint16_t)((addr23 >> 16) & 0x7F);
  npu_send(CMD_SDRAM_SET_ADDR, a, b);
}

void sdram_write_byte(uint8_t val) {
  wait_ready();
  npu_send(CMD_SDRAM_WR, 0, (uint16_t)val);
}

// Escribe 'len' bytes con el patron pattern(i) a partir de base_addr,
// directo en SDRAM (sin pasar por el buffer -- mismo camino ya probado en
// Fase 1).
void write_pattern(uint32_t base_addr, uint32_t len) {
  sdram_set_addr(base_addr);
  for (uint32_t i = 0; i < len; i++) {
    uint8_t val = (uint8_t)((i * 37 + 11) & 0xFF);
    sdram_write_byte(val);
  }
}

void prefetch_set_base(uint32_t addr23) {
  uint16_t a = (uint16_t)(addr23 & 0xFFFF);
  uint16_t b = (uint16_t)((addr23 >> 16) & 0x7F);
  npu_send(CMD_PREFETCH_SET_BASE, a, b);
}

void prefetch_set_len(uint16_t len) {
  npu_send(CMD_PREFETCH_SET_LEN, len, 0);
}

// Dispara la precarga y espera a que termine. Devuelve retry_count/
// fail3_count leidos de la respuesta que confirmo busy=0.
void prefetch_run(uint32_t base_addr, uint16_t len, uint16_t *retry_count, uint16_t *fail3_count) {
  prefetch_set_base(base_addr);
  prefetch_set_len(len);
  wait_ready(); // confirma que quedo libre ANTES de arrancar
  npu_send(CMD_PREFETCH_START, 0, 0);
  wait_ready(); // espera a que termine TODA la precarga (busy cubre el rango completo)
  *retry_count = ((uint16_t)g_last_rx[2] << 8) | g_last_rx[3];
  *fail3_count = ((uint16_t)g_last_rx[4] << 8) | g_last_rx[5];
}

uint8_t prefetch_read_byte(uint16_t rdptr) {
  npu_send(CMD_PREFETCH_SET_RDPTR, rdptr, 0);
  wait_ready();
  npu_send(CMD_PREFETCH_RD_BYTE, 0, 0);
  wait_ready();
  return g_last_rx[1];
}

// Escribe un patron de 'len' bytes en SDRAM, lo precarga verificado al
// buffer, y compara el buffer resultante contra el patron esperado.
void test_prefetch(const char *label, uint32_t base_addr, uint16_t len) {
  Serial.printf("=== %s: base=0x%06lX len=%u ===\n", label, (unsigned long)base_addr, len);

  uint32_t t0 = micros();
  write_pattern(base_addr, len);
  uint32_t t_write = micros() - t0;

  uint16_t retry_count = 0, fail3_count = 0;
  uint32_t t1 = micros();
  prefetch_run(base_addr, len, &retry_count, &fail3_count);
  uint32_t t_prefetch = micros() - t1;

  int mism = 0;
  uint32_t t2 = micros();
  for (uint16_t i = 0; i < len; i++) {
    uint8_t expected = (uint8_t)((i * 37 + 11) & 0xFF);
    uint8_t got = prefetch_read_byte(i);
    if (got != expected) {
      mism++;
      if (mism <= 10) Serial.printf("  [%4u] esperado=%3u buffer=%3u\n", i, expected, got);
    }
  }
  uint32_t t_readback = micros() - t2;

  Serial.printf("%s: %u/%u coinciden en el buffer  (escritura: %.2fms, precarga: %.2fms, lectura buffer: %.2fms)\n",
                label, len - mism, len, t_write / 1000.0, t_prefetch / 1000.0, t_readback / 1000.0);
  Serial.printf("  pf_retry_count=%u (%.2f%%)  pf_fail3_count=%u\n\n",
                retry_count, 100.0 * retry_count / len, fail3_count);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: Etapa 4 Fase 2 -- precarga verificada SDRAM->buffer ===\n");
  wait_pll_lock();
  wait_sdram_init();

  test_prefetch("Rango completo", 0, BUF_DEPTH);

  Serial.println("=== Fase 2 completa ===");
}

void loop() {
}
