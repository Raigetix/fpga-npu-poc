// sdram_p3_diag.ino -- Etapa 4: LA ESCRITURA A SDRAM, ¿es confiable a escala?
//
// Hipotesis: los errores que quedan en los modelos grandes no vienen de la
// LECTURA (weight_stream.v ya la protege con votacion por mayoria de 3)
// sino de la ESCRITURA, que no tiene ninguna verificacion. Un byte mal
// escrito queda corrupto de forma permanente para toda la sesion, y por eso
// el modelo falla en TODAS las inferencias.
//
// Encaja con lo observado: los tests de diagnostico que escribian pocos
// bytes (448, 960) daban perfecto, y los modelos grandes (27K y 73K bytes)
// fallan siempre. En la fase 1 se midio ~0.3-0.5% de bytes mal en un ciclo
// escritura+lectura de 256 bytes, y se atribuyo a la lectura.
//
// Este test escribe un patron grande (27040 bytes, como el modelo
// "Original") y lo lee de vuelta para medir cuantos bytes quedaron mal.
// Despues reintenta SOLO los que fallaron, para ver si la escritura
// converge con reintentos (que seria el arreglo, igual que ya se hizo en
// la etapa 3 con la carga de pesos a BRAM).

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP            0x00
#define CMD_SDRAM_SET_ADDR 0x01
#define CMD_SDRAM_WR       0x02
#define CMD_SDRAM_RD       0x0D

#define N_BYTES 27040UL   // igual que el modelo "Original" (130-128-64-32-5)
#define MAX_REPORT 15

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
void npu_send(uint8_t cmd, uint16_t a, uint16_t b_raw) {
  uint8_t rx[6]; spi_burst_begin(); npu_xfer(cmd, a, b_raw, rx); spi_burst_end();
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

void sdram_set_addr(uint32_t addr23) {
  npu_send(CMD_SDRAM_SET_ADDR, (uint16_t)(addr23 & 0xFFFF), (uint16_t)((addr23 >> 16) & 0x7F));
}

// bit2 del status = SDRAM ocupada (ver top_sdram_p3.v)
void wait_sdram_idle() {
  uint8_t rx[6];
  spi_burst_begin();
  bool busy = true;
  while (busy) { npu_xfer(CMD_NOP, 0, 0, rx); busy = (rx[0] & 0x04) != 0; }
  spi_burst_end();
}

uint8_t sdram_read_byte_at(uint32_t addr) {
  uint8_t rx[6];
  sdram_set_addr(addr);
  wait_sdram_idle();
  npu_send(CMD_SDRAM_RD, 0, 0);
  wait_sdram_idle();
  spi_burst_begin();
  npu_xfer(CMD_NOP, 0, 0, rx);   // el dato quedo en el byte 1
  spi_burst_end();
  return rx[1];
}

static inline uint8_t pattern_at(uint32_t i) { return (uint8_t)((i * 31 + 7) & 0xFF); }

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== ¿La ESCRITURA a SDRAM es confiable a escala? ===");
  Serial.printf("(%lu bytes, el mismo tamaño que el modelo Original)\n", N_BYTES);
  wait_pll_lock();

  // ---- Escritura ----
  Serial.println("\nEscribiendo...");
  uint32_t t0 = micros();
  sdram_set_addr(0);
  spi_burst_begin();
  uint8_t rx[6];
  for (uint32_t i = 0; i < N_BYTES; i++)
    npu_xfer(CMD_SDRAM_WR, 0, pattern_at(i), rx);
  spi_burst_end();
  Serial.printf("  %lu bytes en %.2f s\n", N_BYTES, (micros() - t0) / 1e6);

  // ---- Lectura de verificacion (secuencial, el puntero autoincrementa) ----
  Serial.println("Leyendo de vuelta y comparando...");
  static uint32_t bad_addr[512];
  uint32_t bad = 0;
  t0 = micros();
  sdram_set_addr(0);
  for (uint32_t i = 0; i < N_BYTES; i++) {
    wait_sdram_idle();
    npu_send(CMD_SDRAM_RD, 0, 0);
    wait_sdram_idle();
    spi_burst_begin();
    npu_xfer(CMD_NOP, 0, 0, rx);
    spi_burst_end();
    if (rx[1] != pattern_at(i)) {
      if (bad < 512) bad_addr[bad] = i;
      bad++;
      if (bad <= MAX_REPORT)
        Serial.printf("  [%6lu] esperado=%3u  leido=%3u\n", i, pattern_at(i), rx[1]);
    }
  }
  Serial.printf("  verificacion en %.2f s\n", (micros() - t0) / 1e6);
  Serial.printf("\n>> BYTES MAL: %lu de %lu  (%.3f%%)\n", bad, N_BYTES, 100.0 * bad / N_BYTES);

  if (bad == 0) {
    Serial.println(">> La escritura es confiable: el problema de los modelos");
    Serial.println("   grandes esta en otro lado.");
    Serial.println("\n=== Listo ===");
    return;
  }

  // ---- ¿Converge reintentando solo los que fallaron? ----
  uint32_t to_fix = (bad < 512) ? bad : 512;
  Serial.printf("\nReintentando los %lu bytes que fallaron...\n", to_fix);
  for (uint32_t k = 0; k < to_fix; k++) {
    sdram_set_addr(bad_addr[k]);
    wait_sdram_idle();
    npu_send(CMD_SDRAM_WR, 0, pattern_at(bad_addr[k]));
  }
  uint32_t still_bad = 0;
  for (uint32_t k = 0; k < to_fix; k++) {
    if (sdram_read_byte_at(bad_addr[k]) != pattern_at(bad_addr[k])) still_bad++;
  }
  Serial.printf(">> Despues de UN reintento siguen mal: %lu de %lu\n", still_bad, to_fix);
  Serial.println(still_bad == 0
    ? ">> Converge: alcanza con verificar y reintentar al cargar el modelo."
    : ">> NO converge del todo: hay direcciones que fallan de forma persistente.");

  Serial.println("\n=== Listo ===");
}

void loop() {}
