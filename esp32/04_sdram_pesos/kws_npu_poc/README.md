# Detección de palabras clave en la NPU

Reconoce 8 palabras en inglés (**down, go, left, no, right, stop, up, yes**)
con una red densa de 4 capas (490→256→256→144→8) entrenada en Keras sobre el
dataset público de Google (mini Speech Commands) y cuantizada a int8 con el
conversor estándar de TensorFlow Lite.

Los pesos son **228.992 bytes**: 5,5 veces lo que entra en la BRAM de la
placa. Viven en la SDRAM de la Tang Nano 20K y se transmiten a los 8 carriles
de cómputo durante la inferencia.

## Cómo subirlo

El modelo y la página **no** están compilados dentro del firmware: viven en el
sistema de archivos de la flash del ESP32. Por eso son dos comandos:

```
pio run -t uploadfs     # model.bin, test.bin, index.html
pio run -t upload       # el firmware
```

Si cambiás solo el modelo o el HTML, alcanza con el primero. Si el firmware
arranca y no encuentra los archivos, te lo avisa por el puerto serie en vez
de colgarse.

Antes de compilar, poné tu red WiFi arriba de `kws_npu_poc.cpp`
(`WIFI_SSID` / `WIFI_PASS`). Si los dejás vacíos o no logra conectarse, crea
su propia red `NPU-KWS` con clave `npu12345`.

## Habilitar el micrófono en el navegador

**El problema:** los navegadores solo permiten usar el micrófono en páginas
seguras (HTTPS) o en `localhost`. La página la sirve el ESP32 por
`http://192.168.x.x`, así que por defecto queda bloqueada. No es algo que se
pueda arreglar desde el código del ESP32: es una política del navegador.

La solución es marcar a mano esa dirección como confiable.

### Chrome (en la PC y en Android)

Es la opción que funciona con seguridad en ambos.

1. Abrí `chrome://flags`
2. Buscá **"Insecure origins treated as secure"**
3. Cambialo a **Enabled**
4. En el cuadro de texto escribí la dirección exacta del ESP32, con el
   `http://` y sin barra final. Por ejemplo:

   ```
   http://192.168.0.50
   ```

5. Tocá **Relaunch** para reiniciar el navegador

La dirección aparece en el puerto serie al arrancar. Si tu router le asigna
otra IP más adelante, hay que actualizar este valor.

### Firefox

Firefox tenía dos preferencias para esto, pero Mozilla las fue quitando y **no
pude confirmar que sigan funcionando** en las versiones actuales. Probá en
`about:config` poner en `true`:

- `media.devices.insecure.enabled`
- `media.getusermedia.insecure.enabled`

Si no aparecen o no surten efecto, usá Chrome con el procedimiento de arriba.

### Alternativa sin tocar el navegador

Servir la página desde tu PC en `localhost` (que el navegador **sí** considera
seguro) y que desde ahí le hable al ESP32. Requiere un servidor local mínimo y
habilitar CORS en el ESP32; si lo necesitás, se puede armar.

## Cómo leer los resultados

El benchmark que corre al arrancar imprime dos métricas que miden cosas
distintas, y conviene no confundirlas:

- **Coincidencia con la referencia de la PC** — mide si el *hardware* es
  exacto. Tiene que dar 200/200: significa que la FPGA reproduce bit a bit lo
  que hace el modelo cuantizado corriendo en la computadora.
- **Precisión sobre la palabra real** — mide qué tan bueno es el *modelo*. El
  techo es ~83%, y el propio benchmark lo imprime al lado para que compares
  contra el número correcto y no contra un 100% imposible.

## Qué esperar al probar con tu voz

Menos precisión que en el benchmark, y es esperable: el dataset son
mayormente hablantes nativos de inglés grabados con otros micrófonos. Tu voz
y tu micrófono no están representados en el entrenamiento.

Consejos que ayudan: decir la palabra fuerte y clara apenas empieza a grabar
(la ventana es de 1 segundo), en un lugar sin mucho ruido.

Si la FPGA y el ESP32 muestran palabras **distintas**, eso sí es un problema
real: corren el mismo modelo con la misma aritmética y deberían coincidir
siempre. La página te avisa cuando pasa.

## Nota sobre el cálculo de MFCC

El audio se convierte a características (MFCC) en el ESP32, en C. Esa cadena
tiene que dar **idéntica** a la que se usó al entrenar, porque si difiere
aunque sea un poco, la precisión se desploma. Para eliminar ese riesgo, el
script de entrenamiento exporta las tablas (banco de filtros mel y DCT) dentro
de `model.bin` en vez de que el C las recalcule con sus propias fórmulas.

Aun así, esta parte **todavía no está validada en hardware**: el benchmark usa
características ya calculadas en Python, así que pasaría igual aunque el MFCC
en C estuviera mal. Solo se pone a prueba al usar el micrófono. Si el
benchmark da perfecto pero la demo con voz reconoce mal casi todo, el MFCC es
el sospechoso principal.
