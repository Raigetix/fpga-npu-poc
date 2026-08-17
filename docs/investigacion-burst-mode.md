# Investigación: modo ráfaga para el streaming de pesos por SDRAM

**Estado: sin resolver.** Compila limpio, cierra timing (Fmax 73.8MHz contra
54MHz pedido), pero rompe la escritura/verificación de pesos en hardware
real de forma sistemática (miles de bytes mal, no se resuelve reintentando).
Se agotó el análisis lógico disponible sin encontrar la causa — este
documento existe para que la próxima sesión no repita las mismas 6+ rondas
de compilar/grabar/probar que ya se hicieron y descartaron.

## Por qué se intentó

El motor de streaming de pesos actual (`sdram.v` + `weight_stream.v` en
`main`) lee una palabra de 32 bits a la vez: activar fila, leer, precargar
— 5 ciclos por operación, sin solapamiento. El cómputo de los 8 carriles
consume 8 pesos nuevos cada ~93ns; el streaming corre 4-6x más lento (cada
palabra se verifica por mayoría de 3, así que cada lectura cuesta 2-3
operaciones de 5 ciclos). Midiendo en hardware real: con un modelo de
~234k pesos, la FPGA tardaba 19.05ms contra 19.30ms del ESP32 en software
— casi sin ventaja, mientras que con MNIST (109k pesos) la ventaja era
1.68x. La sospecha, confirmada, era que el diseño está atado al ancho de
banda de la SDRAM: la FPGA escala ~82 ns/peso de forma consistente sea cual
sea la forma de la red.

## Qué se diseñó

Ráfaga manual en `sdram.v`: en vez de activar-leer-precargar por palabra,
mantener la fila abierta y emitir 8 comandos READ seguidos (uno por ciclo,
sin auto-precharge) a columnas consecutivas, precargando recién al final.
**No usa el modo ráfaga nativo de la SDRAM** (el registro de modo sigue en
BURST_LEN=1 para todo lo demás, así el camino de lectura/escritura por byte
existente queda intacto) — son comandos READ individuales emitidos rápido,
no una ráfaga de hardware real.

`weight_stream.v` se rediseñó para pedir 8 palabras de una vez (4 grupos)
en vez de a una por vez, manteniendo la misma protección de mayoría de 3
pero a nivel de ráfaga completa (traer 2 veces y comparar; si difieren,
traer una tercera y votar por posición).

Los 4 archivos tocados (`sdram.v`, `weight_stream.v`,
`mlp_engine_par_stream.v`, `top_sdram_p3.v`) están en este mismo commit —
`git diff main` los muestra completos.

## Qué se probó y se descartó (con evidencia, no por sospecha)

Cada ítem de esta lista costó al menos una ronda completa de
compilar+grabar+reiniciar el ESP32+leer el resultado (~5-10 minutos cada
una, salvo que se indique simulación):

1. **La lógica de la ráfaga en sí.** Se vació el cuerpo del estado
   `RDBURST` a un no-operación (entra y sale sin leer nada) — **falla
   exactamente igual** (mismo conteo de bytes malos). Esto descarta que el
   bug esté en el diseño de la ráfaga.

2. **Arranque prematuro de `weight_stream` antes de `START`.** El motor de
   fetch arranca a pedir apenas la FPGA se configura, mucho antes de que el
   ESP32 escriba un peso. Se agregó una traba (`primed`, un registro que
   solo se levanta con el primer pulso de `START` real) para que no haga
   nada hasta entonces — **falla exactamente igual**. Esto descarta la
   carrera de arranque temprano.

3. **El `casex` de Verilog.** Es una construcción con fama de dar
   problemas de síntesis al agregarle patrones nuevos (resolución de
   solapamientos "no importa" no siempre obvia). Se sacó el manejo de
   `RDBURST` completamente afuera del `casex` (un `if (state==RDBURST)
   ... else casex(...)` en vez de un patrón más adentro) — **falla
   exactamente igual**.

4. **Advertencias de síntesis nuevas.** Se comparó compilando la versión de
   respaldo (sin tocar) — las mismas advertencias (`clk_d`/`sclk_d` con
   ruteo genérico, truncamiento de expresión) ya existían antes. No son
   nuevas.

5. **Interacción lógica streaming + refresco + verificación (via
   simulación con Icarus Verilog).** Se armó un modelo de comportamiento de
   SDRAM (`fpga_project/sim/sdram_model.v`) y un testbench que reproduce
   **exactamente** el protocolo real del firmware (`tb_write_verify_openloop.v`
   — sin consultar `busy`, con el patrón de "trama N trae la respuesta de
   N-1" que usa `verify_weights()`). Con tiempos realistas entre comandos
   SPI (~6us/trama, igual que el hardware), la simulación reproduce una
   tasa de error de ~0.29% (12 de 4096 bytes), ligada al refresco periódico
   — pero **corriendo el mismo testbench contra la versión de respaldo
   (sin ninguno de estos cambios) da exactamente los mismos 12 errores, en
   las mismas direcciones, con los mismos valores.** Esa tasa de error ya
   existía antes (es el bug intermitente ~0.004-0.5% ya documentado y
   mitigado con la verificación por mayoría) — no es lo que causa la falla
   catastrófica en hardware real.

## Conclusión (hipótesis de trabajo, no confirmada)

Después de descartar la lógica, la interacción entre módulos, y la
sintaxis, la simulación RTL (que solo modela el protocolo lógico con
señales ideales, sin retardos eléctricos reales) **no logra reproducir la
falla catastrófica en absoluto** — el mejor caso que reprodujo fue la tasa
de error ya conocida y ya mitigada, idéntica a la versión que funciona.
Esto apunta a que el problema ya no es de lógica: probablemente sea margen
de tiempo eléctrico real (rutas más largas o distinto posicionamiento en
el chip por tener más lógica agregada en la síntesis, incluso lógica
funcionalmente no relacionada) — algo invisible para RTL puro, que
necesitaría un análisis de tiempos con retardos reales (SDF back-annotated)
para diagnosticar con certeza.

Se probó agregar 2 ciclos de margen extra a la espera posterior a la
precarga manual de la ráfaga (el único lugar donde quedaba holgura sin
tener que agrandar el contador `cycle` de 4 bits, que está compartido por
TODOS los estados del controlador — agrandarlo con seguridad exige revisar
cada literal de ciclo del archivo, un cambio bastante más grande y
riesgoso). **Tampoco resolvió nada.**

## Ideas para la próxima sesión, en orden de esfuerzo creciente

1. **Instalar un simulador con retardos reales** (o conseguir el archivo
   SDF que genera Gowin post-P&R) para ver si ahí sí aparece una violación
   de tiempos que la síntesis estática no reporta como error duro.
2. **Agrandar el contador `cycle` de `sdram.v` a 5+ bits** con cuidado
   (actualizando TODOS los literales de ciclo de TODOS los estados, no solo
   los de `RDBURST`) para poder probar márgenes de tiempo bastante más
   generosos en la ráfaga (espaciar los comandos READ cada 2 ciclos en vez
   de cada 1, por ejemplo) sin chocar contra el techo de 15.
3. **Separar el controlador de ráfaga en un modulo/instancia totalmente
   aparte** del controlador de byte existente (dos SDRAM controllers no
   pueden compartir el mismo chip físico a la vez, pero si el problema es
   de *síntesis/ubicación* y no de protocolo, aislar más el código nuevo
   podría cambiar cómo Gowin lo ubica en el chip).
4. Si nada de esto encuentra la causa: considerar que la ganancia de la
   ráfaga (4-6x más rápido que el streaming actual, según la cuenta de
   ciclos) puede no valer el riesgo, y en cambio perseguir la otra palanca
   ya identificada y de bajo riesgo: **subir el reloj de 54MHz a 66.7MHz**
   (el propio comentario de `sdram.v` dice que los tiempos ya están
   validados hasta ahí) — ~19% más rápido, sin rediseñar el controlador,
   aunque también necesita verificarse en hardware real con cuidado (ver
   parámetros de la PLL en `pll_sdram.v`).

## Cómo volver al estado de esta investigación

Todo el código de esta rama (`feature/sdram-burst-streaming`) es exactamente
como quedó al final de esta sesión — no hace falta reconstruir nada, `git
checkout feature/sdram-burst-streaming` alcanza. El hardware físico quedó
con el bitstream de `main` (el que funciona) grabado en RAM (volátil) — para
retomar esta investigación hay que recompilar y regrabar desde esta rama.
