# Checkpoint: rafaga nativa de la SDRAM + IODELAY (2026-08-18)

Nuevo mejor estado validado de toda la sesion. Reemplaza el checkpoint
`05_sdram_pesos_burst2_prod_20260818` (rd_burst2 encadenado, 2.67x).

## Resultado validado (benchmark real, 992 clips, reproducido 2 veces)

```
FPGA        982/992   (identico byte a byte a la base -- cero regresion)
ESP32 (sw)  982/992
Precision sobre la palabra real: FPGA 963/992 (97.1%), techo del modelo 962/992 (97.0%)
Tiempo por inferencia:  FPGA 6.14 ms   ESP32 19.30 ms   -> 3.15x
```

Bitstream: `bitstream/fpga_project.fs` (top_sdram_p3, .sdc = sdram_pesos.sdc).
Commit de git: `d99e395` (rama `main`).

## Root cause resuelto en esta version

La SDRAM embebida del GW2AR-18 son en realidad DOS memorias de 16 bits
en el mismo encapsulado (confirmado en el testbench oficial de Gowin,
SDRC_EMB/GW2AR). Hay un desfasaje real de ~1-1.25ns entre esas dos
mitades del bus de 32 bits que un solo punto de muestreo no cubre a
54MHz bajo rafaga real (invisible en un acceso aislado, con margen de
sobra). Se resolvio con `IODELAY` (retardo programable por pin del IOB
de Gowin): 50 pasos (~1.25ns) en cada mitad del bus -- ver `sdram.v`,
parametros `DQ_LO_DELAY_TAPS`/`DQ_HI_DELAY_TAPS`.

## Que trae este checkpoint

- `BURST_LEN=2` real en el registro de modo de la SDRAM (antes 1) --
  WRITE y READ de un solo byte adaptados para dejar pasar/enmascarar
  la 2da palabra que el chip arrastra de oficio.
- `rd_burst2` reescrito a rafaga NATIVA de un solo comando (antes
  encadenaba dos comandos READ separados, incompatible con
  BURST_LEN=2 real).
- `IODELAY` en `dq_in` (mitad baja y alta del bus, 50 pasos cada una).
- Verificacion post-refresco de `top_sdram_p3.v` canalizada
  (`V_CMP2`/`V_CMP3`) para no comparar el dato del pad en el mismo
  ciclo que llega (poco margen con las restricciones de I/O reales).
- Restricciones de I/O reales para la interfaz de la SDRAM en
  `sdram_pesos.sdc` (antes no habia ninguna) -- `clk_sdram`,
  `set_input_delay`/`set_output_delay` basados en el testbench oficial
  de Gowin para esta misma memoria (~166MHz).
- `REFRESH_INTERVAL = 800` ciclos (sin cambios, ya validado antes).

## Validacion en aislamiento (antes de integrar a produccion)

Lectura/escritura simple: 0.0000% (perfecto). `rd_burst2` nativo:
~0.46%, coincide exactamente con la tasa cruda ya conocida de la
carrera de refresco (medida sin proteccion a proposito en esta fase),
confirmando que el desfasaje esta resuelto del todo y lo unico que
queda es el problema YA RESUELTO por la verificacion post-refresco.

## Restaurar este checkpoint

```
"D:\Gowin\...\Programmer\bin\programmer_cli.exe" --device GW2AR-18C \
  --operation_index 2 --fsFile "bitstream\fpga_project.fs"
```

O recompilar desde `src/` con `fpga_project/build.tcl` (top_module=top_sdram_p3,
sdram_pesos.sdc habilitado, sdram_freqtest.sdc deshabilitado en el .gprj).
