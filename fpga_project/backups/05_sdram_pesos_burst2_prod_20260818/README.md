# Checkpoint: rd_burst2 en produccion (2026-08-18)

Punto seguro de retorno **antes** de investigar una optimizacion mayor
(rafaga nativa de la SDRAM). Este es el mejor estado validado hasta ahora:
funcionamiento correcto y el mejor rendimiento medido de toda la sesion.

## Resultado validado (benchmark real, 992 clips)

```
FPGA        982/992   (identico byte a byte a la base -- cero regresion)
ESP32 (sw)  982/992
Precision sobre la palabra real: FPGA 963/992 (97.1%), techo del modelo 962/992 (97.0%)
Tiempo por inferencia:  FPGA 7.23 ms   ESP32 19.30 ms   -> 2.67x
```

Bitstream: `bitstream/fpga_project.fs` (top_sdram_p3, .sdc = sdram_pesos.sdc).
Commit de git: `65ca95f` (rama `main`).

## Que trae este checkpoint

- Verificacion post-refresco (doble/triple lectura) centralizada en el
  arbitro, generalizada a 64 bits para cubrir tanto lectura simple como
  `rd_burst2` con el mismo camino de mayoria (ver `top_sdram_p3.v`).
- `rd_burst2`: lee la palabra lo+hi de cada grupo de 8 pesos en una sola
  operacion SDRAM (columna y columna+1 de la misma fila), con fallback a
  dos lecturas simples cuando `lo` cae en la ultima columna de la fila
  (columna 255) -- ver `weight_stream.v` (`want_burst_now`) y el
  comentario de cabecera de `rd_burst2` en `sdram.v`.
- `REFRESH_INTERVAL = 800` ciclos (validado en
  docs/caracterizacion-frecuencia-sdram.md).

## Por que se guarda este checkpoint ahora

Antes de investigar rafaga NATIVA de la SDRAM (que el chip auto-incremente
la columna internamente, sin un comando READ nuevo por palabra) -- un
cambio bastante mas grande y riesgoso que `rd_burst2` al protocolo de
lectura de `sdram.v`. Si esa investigacion termina en un intento fallido,
este es el estado al que hay que volver.

## Restaurar este checkpoint

```
"D:\Gowin\...\Programmer\bin\programmer_cli.exe" --device GW2AR-18C \
  --operation_index 2 --fsFile "bitstream\fpga_project.fs"
```

O recompilar desde `src/` con `fpga_project/build.tcl` (top_module=top_sdram_p3,
sdram_pesos.sdc habilitado, sdram_freqtest.sdc deshabilitado en el .gprj).
