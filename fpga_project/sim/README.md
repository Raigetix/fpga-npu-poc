# Simulación (Icarus Verilog)

Testbenches usados para investigar la falla del modo ráfaga (ver
[`../../docs/investigacion-burst-mode.md`](../../docs/investigacion-burst-mode.md)).
No dependen del toolchain de Gowin — corren con
[Icarus Verilog](http://iverilog.icarus.com/) puro (`iverilog`/`vvp`),
instalable en Windows con `choco install iverilog -y` (necesita una
terminal como administrador).

`sdram_model.v` es un modelo de comportamiento de una SDR SDRAM genérica
(NO el chip real, solo el protocolo de comandos) con asserts que abortan la
simulación si el controlador viola la secuencia activar/leer/escribir/
precargar/refrescar. Tiene un `trace_en` que se puede prender por
referencia jerárquica (`model.trace_en = 1'b1;` desde el testbench) para
imprimir cada comando con dirección y valor — muy útil para diagnosticar
paso a paso.

## Cómo correr

```bash
cd fpga_project/sim
iverilog -o tb.vvp tb_write_verify_openloop.v sdram_model.v \
  ../src/04_sdram_pesos/sdram.v ../src/04_sdram_pesos/weight_stream.v
vvp tb.vvp
```

## Qué prueba cada uno

- **`tb_write_verify.v`** — escritura+verificación pura contra `sdram.v`,
  sincronizando con `busy` (más simple, pero NO es fiel al protocolo real
  del firmware).
- **`tb_write_verify_ws.v`** — igual, pero con `weight_stream.v` también
  conectado y el árbitro completo (SPI directo > streaming > refresco).
- **`tb_write_verify_openloop.v`** — **el fiel de verdad**: reproduce
  exactamente el protocolo del firmware real (`npu_send`/`npu_xfer` en
  `kws_npu_poc.cpp`), que NO consulta `busy` para nada — solo confía en que
  el tiempo de una trama SPI (~6us) le alcanza de sobra a la SDRAM. Incluye
  el patrón de lectura "de calentamiento antes del loop, cada trama trae la
  respuesta de la anterior" que usa `verify_weights()`. Este es el que hay
  que usar para cualquier investigación futura sobre timing real.
- **`tb_openloop_backup.v`** — el mismo testbench openloop pero apuntando a
  los archivos de `fpga_project/backups/04_sdram_pesos_pre_burst_20260817/src/`
  (la versión SIN ráfaga, conocida-buena) — para comparar contra la versión
  con ráfaga y confirmar si una tasa de error es nueva o preexistente.
