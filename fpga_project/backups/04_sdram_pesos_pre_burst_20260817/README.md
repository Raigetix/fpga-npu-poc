# Backup: SDRAM sin ráfaga (última versión conocida buena)

Copia de seguridad de `fpga_project/src/04_sdram_pesos/` tomada el 2026-08-17,
justo antes de empezar a rediseñar `sdram.v` para agregar ráfaga real (burst
mode) y así acelerar el streaming de pesos. Esta carpeta **no la toca el
proyecto de Gowin** (los archivos del `.gprj` se referencian por ruta exacta,
no por carpeta), así que sirve de ancla fija sin interferir con la compilación.

## Qué hay acá

- `src/` — los 10 archivos de esta etapa (`.v`, `.cst`, `.sdc`) tal como
  estaban cuando el sistema andaba bien, con el controlador de SDRAM
  original (sin ráfaga, 5 ciclos fijos por operación, sin solapamiento).
- `bitstream/fpga_project.fs` — el bitstream YA COMPILADO de esa misma
  versión. Se puede grabar directo a la FPGA sin recompilar nada.

## Por qué se guarda esto

El controlador de SDRAM (`sdram.v`) es simple y ya probado: cada lectura o
escritura tarda 5 ciclos fijos, sin ráfaga y sin solapamiento entre
operaciones. `weight_stream.v` lo usa con verificación por mayoría de 3
(protección contra un bug de corrupción intermitente ~0.3-0.5% que persiste
incluso después de corregir un error de fase de 45° en la PLL -- ver el
comentario en `pll_sdram.v`). El resultado medido: el streaming de pesos
corre 4-6x más lento que el cómputo de los 8 carriles, que pasan la mayor
parte del tiempo esperando.

Se va a intentar un controlador con ráfaga real (aprovechando que el patrón
de acceso a los pesos ya es secuencial, sin saltos) para acercar el streaming
a la velocidad del cómputo. Es un cambio de fondo al controlador de SDRAM, y
tiene riesgo real de romper la verificación de datos que costó una sesión
entera resolver (ver el comentario de la corrupción de fase en
`pll_sdram.v` y el de la verificación por mayoría en `weight_stream.v`).

## Estado conocido de esta version (linea de base para comparar)

Medido en hardware real, modelo de 11 palabras propias (~234k pesos,
`[256,256,128,64,32]`):

```
Coincidencia FPGA vs referencia:  982/992
Precision FPGA sobre palabra real: 963/992 (97.1%)
Tiempo por inferencia:  FPGA 19.05 ms   ESP32 19.30 ms   -> 1.01x
```

FPGA: ~81.3 ns/peso, constante sea cual sea la forma de la red (confirma que
el diseño actual esta atado al caudal de la SDRAM, no al computo).

## Cómo volver atrás si el rediseño falla

**Opción rápida (sin recompilar):** grabar el bitstream ya compilado de acá
directo a la FPGA:

```bash
programmer_cli.exe --device GW2AR-18C --operation_index 9 --fsFile fpga_project/backups/04_sdram_pesos_pre_burst_20260817/bitstream/fpga_project.fs
```

(`--operation_index 2` en vez de `9` para grabar en RAM, volátil, si solo
querés probar sin comprometer la flash todavía.)

**Opción completa (si además querés el código fuente de vuelta):**

```bash
cp fpga_project/backups/04_sdram_pesos_pre_burst_20260817/src/*.v fpga_project/src/04_sdram_pesos/
cp fpga_project/backups/04_sdram_pesos_pre_burst_20260817/src/*.cst fpga_project/src/04_sdram_pesos/
cp fpga_project/backups/04_sdram_pesos_pre_burst_20260817/src/*.sdc fpga_project/src/04_sdram_pesos/
```

y volver a compilar con `gw_sh.exe build.tcl` (ver `CONTEXTO_PARA_NUEVA_SESION.md`
en la raíz del proyecto para los comandos completos).
