# Etapa 3 — MLP paralelo (8 carriles) — CERRADA

8 carriles MAC en paralelo (`mac_lane.v`, instanciado 8 veces), cada uno con
su propio banco de pesos. Objetivo: ganarle en velocidad al ESP32 corriendo
la misma red en software — cumplido. Ver el README del proyecto
(`../../../README.md`, sección "Etapa 3") para la historia completa,
incluido el diagnóstico que resultó ser cableado y no RTL.

- `top_mlp_par.v` — top-level. Sketch compañero:
  `esp32/03_mlp_paralelo/mlp_par_poc.ino`.
- `mlp_engine_par.v` — FSM de inferencia por "olas" de 8 neuronas.
- `mac_lane.v` — un carril MAC con su banco de pesos propio.
- `pll_par.v` — PLL propia de esta etapa (no se toca `pll_108mhz.v` de la
  etapa 2 para no arriesgar esa referencia que ya funciona).
- `debug_weight_par_direct.v` (+ `.cst`/`.sdc` propios) — top-level de
  diagnóstico que terminó destrabando el problema de carga de pesos: carga
  un patrón conocido en `weight_bank` sin pasar por SPI, y lo saca en vivo
  por 8 pines GPIO directo al ESP32, bypaseando SPI/CDC/FSM/mux de depuración
  por completo. Confirmó que la memoria era 100% estable, lo que apuntó el
  problema real hacia el cableado de SPI (que efectivamente lo era). Sketch
  compañero: `esp32/03_mlp_paralelo/weight_stability_probe.ino`.

**Estado:** cómputo más rápido que el ESP32 (~1.16-1.17ms vs ~1.18-1.21ms
por inferencia) y carga de pesos/bias/entrada verificada 100% correcta.
`pll_par.v` quedó fijo en 108MHz (techo real encontrado con un barrido de
frecuencia; 216MHz ya muestra violaciones de timing reales y corrompe la
placa).

## 3 situaciones de arquitectura (demo)

- `mlp_engine_par_s1.v` / `top_mlp_par_s1.v` — Situación 1: `8->12->4`
  (mínimo, el ESP32 gana por overhead fijo de SPI).
- `mlp_engine_par_s2.v` / `top_mlp_par_s2.v` — Situación 2:
  `130->160->96->32->5` (~39.4K pesos, la FPGA gana con más margen que el
  modelo original).
- `mlp_engine_par_s3.v` / `top_mlp_par_s3.v` — Situación 3:
  `130->176->88->36->5` (~41.7K pesos, 40/46 bloques de BRAM, el modelo
  más grande que entra con margen seguro).
- `pll_par_54.v` + `situaciones.sdc` — estas 3 variantes usan un bus de
  direcciones de pesos de 13 bits (en vez de 12) para poder direccionar
  más de 4096 posiciones/carril, lo que alarga el camino crítico lo
  suficiente como para que 108MHz ya no cierre timing -- corren a 54MHz
  con su propia PLL/constraint dedicadas, sin tocar `pll_par.v`/`top.sdc`
  (reservados para `top_mlp_par.v`).
- Sketch compañero de las 3: `esp32/03_mlp_paralelo/situaciones_poc.ino`
  (`#define MODEL 1/2/3`, resubir y reprogramar la FPGA con el top-level
  correspondiente para cada una).

Resultados (50/50 sin fallos en las 3): situación 1 ESP32 gana 10x
(0.012ms vs 0.127ms); situación 2 FPGA gana 2.4x (0.992ms vs 2.358ms);
situación 3 FPGA gana 2.4x (1.023ms vs 2.496ms). Ver README del proyecto
para la tabla completa.
