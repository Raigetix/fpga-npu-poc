# fpga_NPU_poc

Acelerador de redes neuronales (NPU) implementado en una FPGA Tang Nano 20K
(Gowin GW2AR-18C), controlado por un ESP32-S3 vía SPI. Corre redes densas
(fully-connected) configurables por SPI de punta a punta — capas, anchos,
pesos, bias y cuantización — sin recompilar el hardware. Incluye dos demos
funcionando: reconocimiento de dígitos MNIST y detección de palabras clave
por voz (keyword spotting), esta última entrenable con tus propias palabras
desde una app de grabación local.

**Para el contexto completo, decisiones de diseño y lecciones aprendidas
(muchas horas de depuración documentadas), ver
[`CONTEXTO_PARA_NUEVA_SESION.md`](CONTEXTO_PARA_NUEVA_SESION.md).** Este
README es solo la puerta de entrada rápida.

## Estructura

```
fpga_project/    Verilog de la NPU (Gowin IDE). Ver fpga_project/src/04_sdram_pesos/
                 para la version actual (Etapa 4: streaming por SDRAM).
esp32/           Firmware del ESP32-S3 que controla la FPGA por SPI y sirve
                 la demo web. 04_sdram_pesos/kws_npu_poc/ es el sketch activo.
tools/           Scripts de Python: entrenar modelos (MNIST, palabras clave),
                 grabar tu propia voz, y una app web local de grabacion.
```

## Estado actual

- **Rama `main`**: el hardware que está funcionando ahora mismo (grabado en
  la flash de la FPGA). Motor de streaming de pesos por SDRAM sin ráfaga
  nativa (5 ciclos por palabra, sin solapamiento) — probado y estable.
- **Rama `feature/sdram-burst-streaming`**: trabajo en curso para acelerar
  ese streaming con ráfaga manual. Compila, cierra timing, pero produce una
  falla de hardware real no resuelta (ver
  [`docs/investigacion-burst-mode.md`](docs/investigacion-burst-mode.md) en
  esa rama para el diagnóstico completo antes de retomarlo).

## Empezar rápido

**Compilar y grabar la FPGA** (necesita el Gowin IDE, ver rutas en el
CONTEXTO):
```bash
"D:/Gowin/Gowin_V1.9.12.02_SP2_x64/IDE/bin/gw_sh.exe" fpga_project/build.tcl
"D:/Gowin/Gowin_V1.9.12.02_SP2_x64/Programmer/bin/programmer_cli.exe" \
  --device GW2AR-18C --operation_index 9 \
  --fsFile fpga_project/impl/pnr/fpga_project.fs   # grabar en flash (permanente)
```
(`--operation_index 2` graba en RAM en vez de flash — volátil, para probar
sin comprometer nada.)

**Subir el firmware al ESP32** — el proyecto de PlatformIO real vive
**fuera** de esta carpeta
(`C:\Users\Agustin\Documents\PlatformIO\Projects\...`, ver el CONTEXTO). Los
archivos de `esp32/04_sdram_pesos/kws_npu_poc/` son la fuente de verdad;
hay que copiarlos a mano a esa carpeta antes de compilar con PlatformIO. Es
un punto de fricción conocido — sincronizarlos siempre en los dos sentidos.

**Entrenar un modelo con tu propia voz**:
```bash
python tools/train_one_word_npu.py     # graba palabras + entrena + exporta model.bin
python tools/record_server.py          # graba desde el celular en vez de la PC
python tools/test_live_mic.py          # prueba el modelo ya entrenado con tu voz, sin la FPGA
```

## Qué NO está en git

Ver [`.gitignore`](.gitignore): grabaciones de voz (`tools/my_words/`,
datos personales), modelos entrenados exportados y artefactos de
compilación de Gowin — todos regenerables o personales, se quedan en el
disco local sin versionar.
