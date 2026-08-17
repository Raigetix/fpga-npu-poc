// npu_poc.ino — Prueba de concepto: ESP32-S3 como maestro SPI de la mini-NPU
// implementada en fpga_NPU_poc (Tang Nano 20K). Manda un producto punto de
// prueba, calcula el resultado esperado en software y lo compara con lo que
// devuelve la FPGA.
//
// Conexionado (ver constraints en fpga_project/src/top.cst):
//   ESP32-S3 GPIO12 (SCLK) -> Tang Nano 20K pin 72
//   ESP32-S3 GPIO11 (MOSI) -> Tang Nano 20K pin 79
//   ESP32-S3 GPIO13 (MISO) -> Tang Nano 20K pin 86
//   ESP32-S3 GPIO10 (CS)   -> Tang Nano 20K pin 71
//   ESP32-S3 GND           -> Tang Nano 20K GND
//   NO conectar 3V3/5V entre placas: cada una se alimenta por su propio USB.

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_NOP   0x00
#define CMD_MAC   0x01
#define CMD_RESET 0x02

SPIClass fpga_spi(FSPI);

// Manda un frame de 6 bytes y devuelve el acumulador de 48 bits (con signo)
// que la FPGA tenia cargado en ese momento (ver nota de latencia de pipeline
// en top.v: la respuesta de ESTE frame corresponde al estado de 2 frames atras).
int64_t npu_transfer(uint8_t cmd, int16_t a, int16_t b) {
  uint8_t tx[6] = {
    cmd,
    (uint8_t)(a >> 8), (uint8_t)(a & 0xFF),
    (uint8_t)(b >> 8), (uint8_t)(b & 0xFF),
    0x00
  };
  uint8_t rx[6];

  fpga_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.endTransaction();

  int64_t result = ((int64_t)rx[0] << 40) | ((int64_t)rx[1] << 32) | ((int64_t)rx[2] << 24) |
                    ((int64_t)rx[3] << 16) | ((int64_t)rx[4] << 8)  | (int64_t)rx[5];
  if (result & (1LL << 47)) result |= (~0LL << 48); // extension de signo de 48 a 64 bits
  return result;
}

void run_test() {
  const int N = 3;
  int16_t a[N] = {12, -7, 100};
  int16_t b[N] = {3, 20, -5};

  int64_t expected = 0;
  for (int i = 0; i < N; i++) expected += (int64_t)a[i] * (int64_t)b[i];

  npu_transfer(CMD_RESET, 0, 0);
  for (int i = 0; i < N; i++) {
    npu_transfer(CMD_MAC, a[i], b[i]);
    Serial.printf("  MAC: %d * %d\n", a[i], b[i]);
  }
  // 1 frame extra para que el resultado atraviese la latencia del pipeline (ver top.v)
  int64_t got = npu_transfer(CMD_NOP, 0, 0);

  Serial.printf("Esperado (calculado en el ESP32): %lld\n", expected);
  Serial.printf("Recibido de la FPGA:               %lld\n", got);
  Serial.println((expected == got) ? ">> OK: coinciden" : ">> ERROR: no coinciden");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== fpga_NPU_poc: prueba de producto punto via SPI ===");
}

void loop() {
  run_test();
  Serial.println();
  delay(3000);
}
