# Backup: verificación condicional post-refresco (nuevo punto seguro de retorno)

Copia de seguridad de `fpga_project/src/04_sdram_pesos/` tomada el 2026-08-17,
justo antes de retomar el controlador con ráfaga real (burst mode) -- esta
vez con la corrupción intermitente ya explicada y arreglada (ver
`docs/caracterizacion-frecuencia-sdram.md`), y con la esperanza de que el
mismo entendimiento ayude a que el intento de ráfaga no vuelva a fallar como
la vez anterior (ver `feature/sdram-burst-streaming` y
`docs/investigacion-burst-mode.md`). Esta carpeta **no la toca el proyecto de
Gowin** (los archivos del `.gprj` se referencian por ruta exacta, no por
carpeta), así que sirve de ancla fija sin interferir con la compilación.

## Qué hay acá

- `src/` — los 10 archivos de esta etapa (`.v`, `.cst`, `.sdc`) tal como
  estaban cuando el sistema andaba bien, YA CON el controlador de SDRAM
  original sin ráfaga (5 ciclos fijos por operación, sin solapamiento) más
  la verificación por mayoría condicional (solo la lectura que sigue a un
  refresco, no todas).
- `bitstream/fpga_project.fs` — el bitstream YA COMPILADO de esa misma
  versión. Se puede grabar directo a la FPGA sin recompilar nada.

## Qué cambió desde el backup anterior (`04_sdram_pesos_pre_burst_20260817`)

Se encontró la causa raíz de la corrupción intermitente ~0.3-0.5% que venía
desde el principio del proyecto (ver `docs/caracterizacion-frecuencia-sdram.md`
para la campaña completa): es una carrera de temporización en la lectura que
sigue justo después de un refresco de la SDRAM (~97% de la corrupción cae
ahí). `weight_stream.v` votaba por mayoría SIEMPRE, en cada palabra, como
mitigación a ciegas -- ahora vota solo en esa ~1 de cada 88 lecturas que
realmente lo necesita, con toda la lógica de decisión y ejecución centralizada
en el árbitro de `top_sdram_p3.v` (no repartida entre el árbitro y
`weight_stream.v`, que fue la causa de un primer intento fallido, ver el
historial de commits). `REFRESH_INTERVAL` también subido de 700 a 800 ciclos
(dentro del techo real de la SDRAM, ~844 ciclos a 54MHz).

## Estado conocido de esta versión (línea de base para comparar)

Medido en hardware real con el benchmark de producción (992 clips reales,
modelo de 11 palabras propias, ~234k pesos, `[256,256,128,64,32]`):

```
Coincidencia FPGA vs referencia:  982/992
Precision FPGA sobre palabra real: 963/992 (97.1%)  -- identica al backup anterior
Tiempo por inferencia:  FPGA 10.50 ms   ESP32 19.30 ms   -> 1.84x
```

Contra el backup anterior (19.05ms, 1.01x): misma precisión exacta, ~45% más
rápido -- la FPGA pasó de empatar con el ESP32 a ser casi el doble de rápida.

El cuello de botella que queda: el cómputo de 8 carriles consume un grupo de
8 pesos cada ~93ns, pero traer esos 8 pesos necesita 2 lecturas de SDRAM
(~93ns cada una) -- todavía se tarda el doble en traer los pesos de lo que
tarda en consumirlos. Es exactamente lo que el intento de ráfaga real busca
cerrar.

## Cómo volver atrás si el rediseño de ráfaga falla

**Opción rápida (sin recompilar):** grabar el bitstream ya compilado de acá
directo a la FPGA:

```bash
programmer_cli.exe --device GW2AR-18C --operation_index 9 --fsFile fpga_project/backups/04_sdram_pesos_refresh_verify_fix_20260817/bitstream/fpga_project.fs
```

(`--operation_index 2` en vez de `9` para grabar en RAM, volátil, si solo
querés probar sin comprometer la flash todavía.)

**Opción completa (si además querés el código fuente de vuelta):**

```bash
cp fpga_project/backups/04_sdram_pesos_refresh_verify_fix_20260817/src/*.v fpga_project/src/04_sdram_pesos/
cp fpga_project/backups/04_sdram_pesos_refresh_verify_fix_20260817/src/*.cst fpga_project/src/04_sdram_pesos/
cp fpga_project/backups/04_sdram_pesos_refresh_verify_fix_20260817/src/*.sdc fpga_project/src/04_sdram_pesos/
```

y volver a compilar con `gw_sh.exe build.tcl` (ver `CONTEXTO_PARA_NUEVA_SESION.md`
en la raíz del proyecto para los comandos completos).
