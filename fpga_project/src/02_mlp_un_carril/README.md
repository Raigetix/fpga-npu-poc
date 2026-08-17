# Etapa 2 — MLP con un solo carril MAC

Red completa (130→128→64→32→5) reutilizando secuencialmente el mismo MAC de
la etapa 1. Funciona correctamente pero resultó MÁS LENTA que el ESP32 en
software — eso motivó la etapa 3 (paralelizar). Ver el README del proyecto
(`../../../README.md`, sección "Etapa 2") para el detalle.

- `mlp_engine.v` + `top_mlp.v` — motor de inferencia secuencial. Sketch
  compañero: `esp32/02_mlp_un_carril/mlp_poc.ino`.
- `debug_weight_rw.v` — aísla la memoria de pesos (escribir/leer sin pasar
  por la FSM de inferencia) antes de confiar en ella dentro del motor
  completo. Sketch compañero: `esp32/02_mlp_un_carril/wr_test.ino`.
- `top_mlp_fast.v` + `pll_108mhz.v` — intento de acelerar con PLL. Primer
  intento a 108MHz dio corrupción real en placa pese a "0 violaciones" en
  STA (jitter de PLL no modelado); quedó estable en 54MHz. El nombre del
  archivo quedó del intento original a 108MHz.
