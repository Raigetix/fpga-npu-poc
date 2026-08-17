// weight_stability_probe.ino -- lee DIRECTO, sin SPI de por medio, el
// resultado de debug_weight_par_direct.v: 8 pines de dato (weight_dbg en
// vivo, siempre la misma direccion), 1 heartbeat (prueba de que el reloj
// interno de la FPGA sigue vivo), 1 selector de direccion (LOW=segura=100,
// HIGH=sospechosa=1030, la que se agrupaba la corrupcion en todas las
// pruebas por SPI).
//
// Si el valor de 8 bits CAMBIA sin que nadie haya vuelto a escribir nada
// (la FPGA carga el patron una unica vez al arrancar, ver el .v), confirma
// que el problema es de memoria/hardware, aislado de SPI/CDC/FSM/mux (todo
// lo que se probo sin exito hasta ahora). Si se mantiene perfectamente
// estable en las dos direcciones, el problema esta en otro lado.
//
// IMPORTANTE: este sketch asume que la FPGA tiene cargado
// 'debug_weight_par_direct', NO top_mlp_par. Conexionado (ver top.cst):
//   ESP32 GPIO3  -- FPGA pin 73 (data_out[0])
//   ESP32 GPIO8  -- FPGA pin 74 (data_out[1])
//   ESP32 GPIO18 -- FPGA pin 75 (data_out[2])
//   ESP32 GPIO17 -- FPGA pin 85 (data_out[3])
//   ESP32 GPIO16 -- FPGA pin 77 (data_out[4])
//   ESP32 GPIO15 -- FPGA pin 27 (data_out[5])
//   ESP32 GPIO7  -- FPGA pin 28 (data_out[6])
//   ESP32 GPIO6  -- FPGA pin 25 (data_out[7])
//   ESP32 GPIO5  -- FPGA pin 26 (heartbeat)
//   ESP32 GPIO4  -- FPGA pin 29 (addr_sel, lo maneja el ESP32)

#define PIN_D0 3
#define PIN_D1 8
#define PIN_D2 18
#define PIN_D3 17
#define PIN_D4 16
#define PIN_D5 15
#define PIN_D6 7
#define PIN_D7 6
#define PIN_HEARTBEAT 5
#define PIN_ADDR_SEL  4

uint8_t read_data() {
  uint8_t v = 0;
  v |= digitalRead(PIN_D0) << 0;
  v |= digitalRead(PIN_D1) << 1;
  v |= digitalRead(PIN_D2) << 2;
  v |= digitalRead(PIN_D3) << 3;
  v |= digitalRead(PIN_D4) << 4;
  v |= digitalRead(PIN_D5) << 5;
  v |= digitalRead(PIN_D6) << 6;
  v |= digitalRead(PIN_D7) << 7;
  return v;
}

int8_t expected_for(bool risky) {
  return risky ? (int8_t)(1030 % 256) : (int8_t)100;
}

bool risky = false;
uint8_t last_value = 0;
uint32_t change_count = 0;
uint32_t sample_count = 0;
uint32_t last_status_ms = 0;
uint32_t last_switch_ms = 0;
bool last_hb = false;
uint32_t hb_toggles = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  pinMode(PIN_D2, INPUT);
  pinMode(PIN_D3, INPUT);
  pinMode(PIN_D4, INPUT);
  pinMode(PIN_D5, INPUT);
  pinMode(PIN_D6, INPUT);
  pinMode(PIN_D7, INPUT);
  pinMode(PIN_HEARTBEAT, INPUT);
  pinMode(PIN_ADDR_SEL, OUTPUT);
  digitalWrite(PIN_ADDR_SEL, LOW); // arranca en direccion segura (100)

  Serial.println("=== weight_stability_probe: lectura directa sin SPI ===");
  Serial.printf("Direccion segura=100 (esperado=%d), sospechosa=1030 (esperado=%d)\n",
                expected_for(false), expected_for(true));
  Serial.println("Empezando en direccion SEGURA (100)...\n");

  last_hb = digitalRead(PIN_HEARTBEAT);
  delay(5); // tiempo de sobra para que la FPGA cargue el patron (3392 ciclos, <1ms)
  last_value = read_data();
  last_switch_ms = millis();
}

void loop() {
  uint8_t v = read_data();
  sample_count++;

  bool hb = digitalRead(PIN_HEARTBEAT);
  if (hb != last_hb) { hb_toggles++; last_hb = hb; }

  if (v != last_value) {
    change_count++;
    Serial.printf("[%s] CAMBIO: %d -> %d  (esperado=%d)  [cambio #%lu, muestra #%lu]\n",
                  risky ? "sospechosa" : "segura",
                  (int8_t)last_value, (int8_t)v, expected_for(risky),
                  (unsigned long)change_count, (unsigned long)sample_count);
    last_value = v;
  }

  uint32_t now = millis();
  if (now - last_status_ms >= 3000) {
    last_status_ms = now;
    Serial.printf("[%s] status: valor=%d esperado=%d  cambios=%lu/%lu muestras  heartbeat_toggles=%lu\n",
                  risky ? "sospechosa" : "segura",
                  (int8_t)v, expected_for(risky),
                  (unsigned long)change_count, (unsigned long)sample_count,
                  (unsigned long)hb_toggles);
  }

  if (now - last_switch_ms >= 10000) {
    last_switch_ms = now;
    risky = !risky;
    digitalWrite(PIN_ADDR_SEL, risky ? HIGH : LOW);
    change_count = 0;
    sample_count = 0;
    delay(5);
    last_value = read_data();
    Serial.printf("\n>>> Cambiando a direccion %s (esperado=%d)\n\n",
                  risky ? "SOSPECHOSA (1030)" : "SEGURA (100)", expected_for(risky));
  }
}
