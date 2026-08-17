// wr_test.ino -- prueba aislada: carga unos pocos valores conocidos en
// direcciones puntuales (incluyendo bordes probables de bloque de BSRAM) de
// la memoria de pesos de la FPGA (top_debug_wr), los lee de vuelta, y
// compara. No usa la red neuronal para nada, solo prueba la memoria.

#include <SPI.h>

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

#define CMD_LOAD 0x01
#define CMD_READ 0x05

SPIClass fpga_spi(FSPI);

void npu_send(uint8_t cmd, uint16_t a, uint16_t b_raw, uint8_t rx[6]) {
  uint8_t tx[6] = {
    cmd,
    (uint8_t)(a >> 8), (uint8_t)(a & 0xFF),
    (uint8_t)(b_raw >> 8), (uint8_t)(b_raw & 0xFF),
    0x00
  };
  fpga_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  for (int i = 0; i < 6; i++) rx[i] = fpga_spi.transfer(tx[i]);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.endTransaction();
}

const uint16_t addrs[] = {0, 1, 2, 100, 2303, 2304, 8191, 8192, 13519, 13520, 16639, 16640, 20000, 27038, 27039};
const int N = sizeof(addrs) / sizeof(addrs[0]);

void setup() {
  Serial.begin(115200);
  delay(10000);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  fpga_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("=== Test aislado: memoria de pesos (load + read) ===\n");

  uint8_t rx[6];
  int8_t expected[64];

  Serial.println("Cargando valores conocidos...");
  for (int i = 0; i < N; i++) {
    int8_t val = (int8_t)((addrs[i] % 5) - 2); // -2..2, determinista
    expected[i] = val;
    npu_send(CMD_LOAD, addrs[i], (uint16_t)(uint8_t)val, rx);
  }

  Serial.println("Leyendo de vuelta y comparando...\n");
  bool all_ok = true;
  for (int i = 0; i < N; i++) {
    npu_send(CMD_READ, addrs[i], 0, rx);
    npu_send(0x00, 0, 0, rx); // NOP, margen extra de latencia
    npu_send(0x00, 0, 0, rx); // NOP, lectura final

    bool idle = (rx[4] & 0x01) != 0;
    int8_t got = (int8_t)rx[5];
    bool ok = (got == expected[i]);
    all_ok &= ok;

    Serial.printf("addr=%5u  esperado=%3d  recibido=%3d  idle=%d  %s\n",
                  addrs[i], expected[i], got, idle, ok ? "OK" : "MISMATCH");
  }

  Serial.println();
  Serial.println(all_ok ? ">> TODAS LAS DIRECCIONES COINCIDEN" : ">> HAY DIRECCIONES QUE NO COINCIDEN");
}

void loop() {
  delay(1000);
}
