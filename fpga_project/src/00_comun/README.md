# 00_comun

Infraestructura compartida por todas las etapas (01, 02, 03). Ver el README
del proyecto (`../../../README.md`) para la historia completa.

- `spi_slave.v` — esclavo SPI genérico (framing de 48 bits), escrito y
  depurado en la etapa 1, nunca modificado después.
- `top.cst` — asignación de pines física, vale para cualquier top-level.
- `top.sdc` — constraints de timing (relojes, incertidumbre). El bloque de
  `clk_sys` se edita según qué top-level esté activo en ese momento (ver
  comentario dentro del archivo).
