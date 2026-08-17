# Etapa 1 — MAC único

PoC mínima: el ESP32 manda dos operandos por SPI, la FPGA multiplica con el
DSP duro y acumula. Ver el README del proyecto (`../../../README.md`,
sección "Etapa 1") para la historia completa de qué falló y cómo se
diagnosticó.

- `top.v` — versión final, funcionando. Sketch compañero:
  `esp32/01_mac_unico/npu_poc.ino`.
- `debug_frame_counter.v` — ¿se detectan bien los frames SPI (flancos de CS)?
- `debug_miso_const.v` — aísla el camino de lectura (FPGA→ESP32).
- `debug_cmd_mask.v` — aísla la decodificación del byte de comando.
- `debug_product.v` — el producto del DSP solo, sin acumular.
- `debug_wide_accum.v` — aísla el bug de "registro auto-referenciado leído
  por puerto indexado variable" con un +1 constante.
- `debug_sum.v` — mismo aislamiento con datos reales (A+B).
- `debug_exec_counts.v` — cuenta ejecuciones de cada rama del FSM.

Cada `debug_*.v` es su propio top-level (bitstream aparte), pensado para
aislar UNA sospecha a la vez.
