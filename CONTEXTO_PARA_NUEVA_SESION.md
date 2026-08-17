# Contexto del proyecto — para retomar en una sesión nueva

Este documento junta todo lo necesario para seguir trabajando sobre este
proyecto sin tener que redescubrirlo. Está escrito para que lo lea otra
sesión desde cero.

**Estado:** el objetivo que tenía este documento (app para grabar tu voz y
entrenar un detector de palabras propio) ya está construido y funcionando —
ver la sección 12 actualizada. El **objetivo inmediato de la próxima
sesión** ahora es otro: retomar el trabajo de ráfaga manual para el
streaming de pesos por SDRAM, que quedó documentado sin resolver en
`fpga_project/backups/.../` y en la rama de git `feature/sdram-burst-streaming`
(ver `docs/investigacion-burst-mode.md` en esa rama). El repositorio ya usa
git: rama `main` = lo que está funcionando ahora en el hardware real, esa
otra rama = el trabajo en curso.

---

## 1. Qué es este proyecto

Una **NPU (acelerador de redes neuronales) implementada en FPGA**, controlada
por un ESP32-S3 vía SPI.

- **FPGA**: Tang Nano 20K, chip Gowin GW2AR-18C. Tiene 8 carriles de
  multiplicar-acumular en paralelo y una SDRAM embebida de 8 MB donde viven
  los pesos.
- **ESP32-S3**: le carga el modelo por SPI, le manda las entradas y lee los
  resultados. También corre el mismo modelo en software para comparar.

El estado actual: **la NPU es configurable por SPI de punta a punta**. Se le
puede cargar cualquier red densa (cantidad de capas, anchos, pesos, bias,
parámetros de cuantización) sin recompilar el hardware. Ya corre dos modelos
reales entrenados con Keras + TensorFlow Lite: MNIST (dígitos) y detección de
8 palabras clave.

---

## 2. Límites del hardware (importantes al diseñar un modelo)

| Límite | Valor | Dónde |
|---|---|---|
| Tipo de capa | **solo densas** (fully-connected) | no hay convolución |
| Capas máximas | 8 | `MAX_LAYERS` |
| Entradas máximas (capa 0) | 1024 | `MAX_INPUT_WIDTH` |
| Neuronas por capa | 256 | `MAX_LAYER_WIDTH` |
| Bias totales | 2048 | `MAX_BIAS` |
| Pesos | ~8 MB (los limita la SDRAM) | en la práctica, sin límite útil |
| Aritmética | int8, estilo TFLite **per-tensor** | no soporta per-channel |
| Desplazamiento de reescalado | `QSHIFT = 20`, fijo | |

Esos límites están en
`fpga_project/src/04_sdram_pesos/mlp_engine_par_stream.v` y cambiarlos exige
recompilar el hardware (no hace falta para el objetivo de la próxima sesión).

**Rendimiento medido** (MNIST 784-128-64-10, 109.568 pesos):
FPGA 8,91 ms vs ESP32 en software 14,96 ms → **1,68x más rápido**. El cuello
de botella es el caudal de la SDRAM, no el cálculo: los 8 carriles pasan
buena parte del tiempo esperando pesos. Modelos más grandes aprovechan mejor.

---

## 3. Estado del hardware: ya está grabado, no hace falta tocarlo

El bitstream final está **grabado en la flash de la FPGA** (sobrevive a
reinicios y a desconectar el USB). El módulo top es `top_sdram_p3`.

Solo hay que reprogramar la FPGA si se cambian los límites de la tabla de
arriba. Comandos, por si hiciera falta:

```bash
# compilar (Tcl: open_project ... ; set_option -top_module top_sdram_p3 ; run all)
"D:/Gowin/Gowin_V1.9.12.02_SP2_x64/IDE/bin/gw_sh.exe" build.tcl

# grabar en RAM (volátil, para iterar)
programmer_cli.exe --device GW2AR-18C --operation_index 2 --fsFile fpga_project/impl/pnr/fpga_project.fs
# grabar en flash (permanente)
programmer_cli.exe --device GW2AR-18C --operation_index 9 --fsFile ...
```

`gw_sh.exe` falla a veces devolviendo un log vacío; se resuelve reintentando
el mismo comando.

---

## 4. Protocolo SPI (ESP32 → FPGA)

Tramas de **6 bytes** (48 bits): `[CMD][A_hi][A_lo][B_hi][B_lo][cola]`.

Respuesta (MISO): `[estado][byte leído de SDRAM][x][x][x][byte de debug]`
donde estado: bit0 = cómputo ocupado, bit1 = PLL enganchado, bit2 = SDRAM
ocupada.

| Comando | Código | Argumentos |
|---|---|---|
| NOP | 0x00 | (sondeo de estado) |
| SDRAM_SET_ADDR | 0x01 | A=dir[15:0], B[6:0]=dir[22:16] |
| SDRAM_WR | 0x02 | B_lo = byte; el puntero avanza |
| WSTREAM_SET_BASE | 0x03 | dirección base de los pesos |
| SET_NUM_LAYERS | 0x04 | A[3:0] = cantidad de capas |
| SET_LAYER_SHAPE | 0x05 | A[2:0]=capa, A[5:4]=param, B=valor. param: 0=entradas, 1=salidas, 2=base de bias |
| SET_BIAS_ADDR | 0x06 | A = puntero de escritura de bias |
| BIAS_WR | 0x07 | valor de 32 bits en bytes 1-4; puntero avanza |
| SET_QPARAM | 0x08 | valor 32 bits en bytes 1-4; cola = `{capa[2:0], param[1:0]}`. param: 0=multiplicador, 1=zero-point de salida, 2=mínimo, 3=máximo |
| START | 0x09 | dispara la inferencia |
| DBG_RD | 0x0A | A[13:11]=fuente, A[10:0]=índice. fuente: 0=actbuf_a, 1=actbuf_b, 4=entrada, 5=**salida**, 6=bias |
| SET_ITGT | 0x0B | A[10:0] = puntero de escritura de la entrada |
| IBURST5 | 0x0C | 5 bytes de entrada por trama; el puntero avanza |
| SDRAM_RD | 0x0D | lee un byte en el puntero; llega en el byte 1 de la trama siguiente |

Secuencia típica para cargar y correr un modelo:

1. `SET_NUM_LAYERS`, y por cada capa tres `SET_LAYER_SHAPE`
2. `SDRAM_SET_ADDR(0)` + muchos `SDRAM_WR` (los pesos) + `WSTREAM_SET_BASE(0)`
3. `SET_BIAS_ADDR(0)` + `BIAS_WR` por cada bias
4. Cuatro `SET_QPARAM` por capa
5. Por inferencia: `SET_ITGT(0)`, varios `IBURST5`, `START`, sondear NOP hasta
   que baje el bit de ocupado, y leer con `DBG_RD` fuente 5.

**Ojo**: si la cantidad de entradas no es múltiplo de 5, hay que completar la
última ráfaga con ceros (bug real que ya costó una sesión de depuración).

---

## 5. Cómo se ordenan los pesos en la SDRAM

El motor recorre los pesos como **una sola secuencia continua**, capa tras
capa, sin saltos. Para el paso local `a` y el carril `l` (0..7):

```
dirección = base + a*8 + l
```

Para la capa L, la "ola" w (= neurona / 8), la entrada i:

```
paso local  a = base_de_la_capa + w * entradas_de_la_capa + i
carril      l = neurona % 8
```

Si la última ola tiene menos de 8 neuronas reales, esos carriles se rellenan
con ceros. Código de referencia: `build_stream_bytes()` en
`tools/train_kws_npu.py`.

---

## 6. Cómo convertir un modelo de TFLite al formato de la NPU

La NPU implementa la misma aritmética entera que TFLite, así que un modelo
cuantizado con el flujo estándar se puede correr tal cual. Hay que traducir
tres cosas:

1. **Pesos**: TFLite los cuantiza simétricos (zero-point = 0), se usan tal
   cual, solo reordenados al layout de arriba.

2. **Bias "plegado"**: la NPU no sabe nada del zero-point de la entrada, así
   que esa corrección se precalcula:
   ```
   bias_efectivo[n] = bias_int32[n] - zero_point_entrada * suma(pesos[n][:])
   ```

3. **Multiplicador entero**:
   ```
   escala_real = escala_entrada * escala_peso / escala_salida
   MULT_INT    = round(escala_real * 2^20)
   ```

4. **Límites de activación**: para capas ocultas con ReLU fusionado, el mínimo
   es el zero-point de salida; para la capa final (logits crudos), −128. El
   máximo siempre 127.

**Importante al cuantizar**: hay que pedir per-tensor explícitamente, porque
la NPU tiene un solo multiplicador por capa:

```python
conv._experimental_disable_per_channel = True
```

---

## 7. Formato de `model.bin` (lo que lee el ESP32)

El modelo ya **no** se compila dentro del firmware: va al sistema de archivos
(LittleFS) de la flash del ESP32. Así compilar tarda 36 s en vez de minutos, y
**se puede cambiar de modelo sin recompilar**.

Todo en little endian:

```
"NPU1"                       4 bytes
num_layers, n_features, num_outputs      3 x uint16
input_scale (float), input_zp (int32), qshift (int32)
por cada capa: in_n, out_n (uint16) + mult, zp_out, act_min, act_max (int32)
n_bias (uint32) + bias[n_bias] (int32)
n_weights (uint32) + weights[n_weights] (int8)
n_mel, n_mfcc, n_bins, n_frames, frame_length, frame_step, fft_length  (uint16)
matriz mel  [n_bins * n_mel]   (float32)
matriz DCT  [n_mel * n_mfcc]   (float32)
largo_nombres (uint16) + nombres separados por \0
```

`test.bin` (opcional, para el benchmark):
```
n_clips, n_features (uint16)
features[n_clips * n_features] (int8)
labels[n_clips] (uint8)
predicciones_de_referencia[n_clips] (uint8)
```

Generarlos: `tools/train_kws_npu.py`. Subirlos: `pio run -t uploadfs`.

---

## 8. Cadena de audio (MFCC) — tiene que ser idéntica en Python y en C

```
audio 16 kHz mono, 1 segundo (16000 muestras)
  → STFT: ventana 640 (40 ms), paso 320 (20 ms), FFT 1024, Hann periódica
  → magnitud  (49 tramas x 513 bins)
  → banco de 40 filtros mel, 20–4000 Hz
  → log(mel + 1e-6)
  → DCT-II ortonormal, primeros 10 coeficientes
  → 49 x 10 = 490 características
```

Para que no haya diferencias de fórmula, **las matrices mel y DCT se exportan
en `model.bin`** y el C solo hace multiplicaciones de matriz. El C está en
`audio_to_features()` en `kws_npu_poc.cpp`.

⚠ **Sin validar en hardware.** El benchmark usa características ya calculadas
en Python, así que pasa aunque el MFCC en C esté mal. Solo se pone a prueba al
usar el micrófono. **Si el benchmark da perfecto pero la demo con voz reconoce
mal casi todo, ese es el sospechoso número uno.** Para diagnosticarlo:
calcular las características del mismo audio en Python y en el ESP32 y
compararlas número por número.

---

## 9. Rarezas del hardware que YA están mitigadas (no volver a tropezar)

Estas costaron horas de depuración. Las mitigaciones están en el código; si se
escribe código nuevo que hable con la NPU, hay que mantenerlas.

1. **La escritura a SDRAM falla ~0,004% de los bytes** (1 de cada ~27.000).
   Parece despreciable, pero **un solo peso corrupto en la primera capa
   arruina el 100% de las inferencias**. → Verificar y reescribir; converge en
   una pasada.

2. **La lectura de SDRAM por SPI falla ~0,15% de los bytes.** Si se toma cada
   diferencia como error real, aparecen cientos de errores inexistentes. →
   Confirmar cada sospechoso con dos relecturas antes de darlo por malo.

3. **El motor de streaming ya protege sus lecturas** con votación por mayoría
   de 3 sobre palabras de 32 bits.

4. *(ya arreglado en el Verilog)* Gowin fusionaba el acumulador dentro de un
   DSP `MULTADDALU18X18`, cuya semántica interna de habilitación hacía que se
   perdiera el último producto de cada ola. Se resuelve con `syn_keep` sobre
   `mul_reg` y `neuron_acc`. **Truco de diagnóstico valioso**: comparar el
   *tipo de DSP inferido* en el reporte de síntesis entre un build que anda y
   uno que no (`MULT9X9` = bien, `MULTADDALU18X18` = fusionado).

5. *(ya arreglado)* `dout32` del controlador de SDRAM no estaba registrado
   (era el bus en vivo) y `tx_snapshot` no se congelaba durante la trama SPI.

**Lección transversal**: cuando un instrumento de medición y el resultado real
se contradicen, sospechar del instrumento. Acá el verificador reportaba 43
bytes malos mientras las inferencias salían perfectas — el que mentía era el
verificador.

---

## 10. Entorno y rutas

| Qué | Dónde |
|---|---|
| Proyecto FPGA (Gowin) | `F:\Agustin\Proyectos\FPGA\fpga_NPU_poc\fpga_project` |
| Fuentes de la NPU | `fpga_project/src/04_sdram_pesos/` |
| Gowin IDE | `D:\Gowin\Gowin_V1.9.12.02_SP2_x64\` |
| Scripts de entrenamiento | `fpga_NPU_poc/tools/` |
| Python (TensorFlow 2.20) | `C:\Users\Agustin\AppData\Local\Programs\Python\Python313\python` |
| Proyecto PlatformIO | `C:\Users\Agustin\Documents\PlatformIO\Projects\260816-143649-4d_systems_esp32s3_gen4_r8n16` |
| Placa | `4d_systems_esp32s3_gen4_r8n16` — ESP32-S3, 8 MB PSRAM, 16 MB flash |
| Puerto serie | COM4, 115200 |

Comandos del ESP32:
```bash
pio run                              # compilar
pio run -t upload   --upload-port COM4   # firmware
pio run -t uploadfs --upload-port COM4   # model.bin, test.bin, index.html
```

**PlatformIO y archivos `.ino`**: PlatformIO genera prototipos escaneando el
`.ino` y **no entiende los raw string literals**. Si hay HTML/JavaScript
embebido, se mete adentro y da errores absurdos (`'async' does not name a
type`). → usar `.cpp` con `#include <Arduino.h>`.

**Al generar código C desde Python**: `"%.7g" % 0.0` da `"0"`, y `0f` **no es
un literal válido en C++**. Siempre forzar el punto decimal (`0.0f`).

---

## 11. Estado de la demo web actual

`kws_npu_poc/` sirve una página donde se graba 1 segundo y se muestra qué
palabra reconoció la FPGA, comparada contra el ESP32 en software. El
firmware (`kws_npu_poc.cpp`, **no** el `.ino` viejo que quedó en la misma
carpeta como resabio de antes de pasar a LittleFS — no se usa) es
completamente genérico: lee cantidad de salidas y nombres de palabras desde
`model.bin`, así que no hace falta tocarlo al cambiar de modelo.

**Los navegadores bloquean el micrófono en páginas http://.** La página la
sirve el ESP32 por IP, así que hay que habilitar el origen a mano:
Chrome (PC y Android) → `chrome://flags` → "Insecure origins treated as
secure" → agregar `http://<ip-del-esp32>` → Relaunch. En Firefox las
preferencias equivalentes puede que ya no existan.

**Captura de audio del navegador — ya afinada, no reinventar**: pedir 16kHz
nativo (`sampleRate:{ideal:16000}` en el constraint Y en el `AudioContext`,
así se evita el remuestreo casero por interpolación lineal), cuenta
regresiva antes de grabar, capturar un poco más de 1s y quedarse con la
ventana de mayor energía (igual que el entrenamiento), normalizar el pico a
~0.65, y en Android sumar las claves viejas `googNoiseSuppression` /
`googAutoGainControl` / etc. junto a las estándar (Chrome en Android a
veces ignora las estándar sin avisar, y el procesamiento de voz del sistema
aplana el detalle espectral que el modelo necesita).

Modelo actual: 11 palabras propias en español, red `490→256→256→128→64→32→12`
(11 palabras + clase "otro"), entrenada con `tools/train_one_word_npu.py`
sobre grabaciones propias + negativos del dataset público de Google +
aumento de canal (ver sección 12). **97% de precisión**, reconoce bien por
PC y por celular.

---

## 12. Qué se construyó esta sesión (app de voz propia) y qué quedó pendiente (ráfaga de SDRAM)

### La app de voz — completa y funcionando

El objetivo que tenía esta sección (grabar tu voz, entrenar un detector
propio, subirlo al ESP32) ya está construido:

- **`tools/train_one_word_npu.py`**: graba repeticiones de cada palabra +
  ruido de fondo por micrófono (PC), arma el dataset (negativos: tu fondo +
  silencio sintético + las otras palabras del dataset de Google), entrena,
  cuantiza y exporta `model.bin`/`test.bin`. Soporta **múltiples palabras
  en un mismo modelo** (clase 0 = "otro", una clase por palabra), no solo
  una — se puede volver a correr agregando palabras nuevas sin perder las
  grabaciones ya hechas.
- **`tools/record_server.py`**: servidor Flask local que sirve una página
  para grabar **desde el celular** (mismo código de captura ya afinado que
  la demo del ESP32) y guarda los `.wav` directo en las carpetas que
  `train_one_word_npu.py` espera — para sumar diversidad de micrófono sin
  cablear nada.
- **`tools/test_live_mic.py`**: prueba el `model.bin` ya entrenado
  reconstruyendo la inferencia int8 EN PYTHON (replica exacta de
  `sw_infer()`), grabando por micrófono en vivo — sirve para confirmar que
  el modelo en sí funciona antes de sospechar de la FPGA/ESP32.

**Lección grande de esta sesión, por si se repite el síntoma "reconoce
perfecto por PC pero no por otro micrófono"**: el modelo se sobreajusta a
la firma acústica exacta del micrófono de entrenamiento. Se confirmó
metódicamente (Chrome en PC con el mismo auricular = perfecto; Chrome en
celular = siempre "otro") que **no era un bug de código** (se probó y
descartó: remuestreo, claves de Chrome, resample nativo a 16kHz) sino una
diferencia real de hardware. Se arregló con dos cosas combinadas: (1) un
aumento de datos nuevo, `augment_channel()` en `train_one_word_npu.py` —
preénfasis con coeficiente aleatorio, simula grabar con un micrófono de
otra coloración espectral; (2) sumar un puñado de grabaciones reales
hechas con el micrófono "problema" (vía `record_server.py`) para las
palabras más importantes. Con eso alcanzó — no hizo falta re-grabar las
once palabras con el celular.

### Ráfaga de SDRAM — quedó sin resolver, no perder tiempo re-descubriendo esto

Motivación: la FPGA solo le gana al ESP32 en software cuando el modelo es
grande (ver benchmarks de MNIST vs este modelo en el README de
`fpga_project`); el motor de streaming de pesos actual usa lecturas de a
una palabra (5 ciclos, sin solapamiento) y corre 4-6x más lento que el
cómputo de los 8 carriles. Se intentó un controlador con ráfaga manual
(activar la fila una vez, 8 lecturas seguidas, precargar al final) —
**compila limpio y cierra timing (Fmax 73.8MHz contra 54MHz pedido), pero
rompe la escritura/verificación de pesos en hardware real** (4096+ bytes
mal de forma sistemática, sube incluso con el cuerpo de la ráfaga vaciado a
un no-operación). Se descartó, con evidencia real, que sea: la lógica de la
ráfaga en sí, el arranque prematuro de `weight_stream` antes de `START`, el
`casex` de Verilog, y la interacción lógica entre streaming/refresco/verificación
(se armó un simulador con Icarus Verilog y un modelo de comportamiento de
SDRAM — reproduce fielmente la tasa de corrupción **ya conocida y
mitigada** (~0.3%, ligada al refresco) pero **igual** entre la versión
vieja y la nueva, así que no es la causa). Conclusión: probablemente un
problema de margen de tiempo eléctrico real (ubicación/ruteo distinto por
tener más lógica), invisible para simulación RTL pura sin un análisis de
tiempos con retardos reales.

**Todo esto quedó documentado en detalle, con el código y el testbench de
simulación, en la rama de git `feature/sdram-burst-streaming`** —
específicamente en `docs/investigacion-burst-mode.md` de esa rama. Antes de
volver a intentarlo, leer ese documento entero — ahorra repetir 6+ rondas
de compilar/grabar/probar en hardware que ya se hicieron y descartaron.
