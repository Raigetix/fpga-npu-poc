"""train_mnist_npu.py -- Entrena un MLP sobre MNIST, lo cuantiza a int8 con
el conversor ESTANDAR de TensorFlow Lite, y traduce los parametros al
formato que espera la NPU del proyecto (ver
fpga_project/src/04_sdram_pesos/mlp_engine_par_stream.v).

Por que este paso existe: la NPU implementa la MISMA aritmetica que TFLite
(entero, escala por capa + zero-point), asi que un modelo entrenado y
cuantizado con el flujo normal de TensorFlow se puede correr tal cual en la
FPGA. Lo unico que hay que hacer es reordenar/reempaquetar:

  1. PESOS: TFLite los cuantiza SIMETRICOS (zero-point de peso = 0), asi que
     se usan tal cual. Se reordenan al layout que espera weight_stream.v:
     para el paso local `a` y el carril `l`, direccion = base + a*8 + l.

  2. BIAS EFECTIVO: la NPU no sabe nada del zero-point de la ENTRADA. Esa
     correccion se pliega una unica vez aca:
         bias_efectivo[n] = bias[n] - zp_entrada * sum(pesos[n][:])
     (TFLite guarda el bias como int32 en escala entrada*peso, que es
     exactamente la escala del acumulador de la NPU.)

  3. MULTIPLICADOR: TFLite define escala_real = escala_entrada *
     escala_peso / escala_salida. La NPU lo aplica como entero:
         MULT_INT = round(escala_real * 2^QSHIFT)
     Se usa un solo multiplicador POR CAPA (no por canal), asi que el
     modelo se cuantiza pidiendo explicitamente per-tensor.

Salida: un archivo .h con los pesos ya reordenados, los bias plegados, los
parametros de cuantizacion por capa, y un puñado de imagenes de prueba
reales del set de test con su etiqueta.

Uso:  python train_mnist_npu.py
"""

import os
import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf

# ---- Debe coincidir con mlp_engine_par_stream.v ----
QSHIFT = 20
NLANES = 8
MAX_LAYER_WIDTH = 256      # limite de la NPU para capas ocultas/salida
MAX_INPUT_WIDTH = 1024     # limite de la NPU para la entrada

# Arquitectura: 784 entradas (28x28) -> 10 digitos. Las ocultas entran
# comodas en el limite de 256 por capa.
LAYER_SIZES = [784, 128, 64, 10]
EPOCHS = 8
N_TEST_IMAGES = 300        # imagenes de prueba que van al sketch (benchmark)

OUT_HEADER = os.path.join(os.path.dirname(__file__), "..", "esp32",
                          "04_sdram_pesos", "mnist_npu_poc", "mnist_model.h")


def train():
    (x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()
    x_train = (x_train.astype(np.float32) / 255.0).reshape(-1, 784)
    x_test = (x_test.astype(np.float32) / 255.0).reshape(-1, 784)

    layers = [tf.keras.layers.Input(shape=(784,))]
    for n in LAYER_SIZES[1:-1]:
        layers.append(tf.keras.layers.Dense(n, activation="relu"))
    layers.append(tf.keras.layers.Dense(LAYER_SIZES[-1]))  # logits, sin softmax
    model = tf.keras.Sequential(layers)

    model.compile(optimizer="adam",
                  loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                  metrics=["accuracy"])
    model.fit(x_train, y_train, epochs=EPOCHS, batch_size=128,
              validation_split=0.1, verbose=2)

    loss, acc = model.evaluate(x_test, y_test, verbose=0)
    print(f"\nPrecision float32 sobre el set de test: {acc*100:.2f}%")
    return model, x_train, x_test, y_test


def quantize(model, x_train):
    """Cuantizacion int8 entera completa, per-tensor (un solo multiplicador
    por capa, que es lo que soporta la NPU)."""
    def representative_dataset():
        for i in range(300):
            yield [x_train[i:i + 1].astype(np.float32)]

    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]
    conv.representative_dataset = representative_dataset
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    conv.inference_input_type = tf.int8
    conv.inference_output_type = tf.int8
    # per-tensor (no per-channel): la NPU tiene UN multiplicador por capa
    conv._experimental_disable_per_channel = True
    return conv.convert()


def extract_layers(tflite_model):
    """Saca de cada capa densa: pesos int8, bias int32, y las escalas/zero
    points necesarios para el multiplicador y los limites de activacion."""
    interp = tf.lite.Interpreter(model_content=tflite_model)
    interp.allocate_tensors()
    details = {d["index"]: d for d in interp.get_tensor_details()}

    layers = []
    for op in interp._get_ops_details():
        if op["op_name"] != "FULLY_CONNECTED":
            continue
        in_idx, w_idx, b_idx = op["inputs"][0], op["inputs"][1], op["inputs"][2]
        out_idx = op["outputs"][0]

        w = interp.get_tensor(w_idx).astype(np.int32)       # (out, in)
        b = interp.get_tensor(b_idx).astype(np.int64)       # (out,)

        s_in = float(details[in_idx]["quantization"][0])
        zp_in = int(details[in_idx]["quantization"][1])
        s_w = float(details[w_idx]["quantization"][0])
        s_out = float(details[out_idx]["quantization"][0])
        zp_out = int(details[out_idx]["quantization"][1])

        layers.append(dict(w=w, b=b, s_in=s_in, zp_in=zp_in,
                           s_w=s_w, s_out=s_out, zp_out=zp_out,
                           in_idx=in_idx, out_idx=out_idx))

    # Ordenar por el orden real de ejecucion (la entrada de una es la salida
    # de la anterior).
    ordered, remaining = [], layers[:]
    cur_in = None
    while remaining:
        if cur_in is None:
            nxt = min(remaining, key=lambda L: L["in_idx"])
        else:
            cand = [L for L in remaining if L["in_idx"] == cur_in]
            nxt = cand[0] if cand else remaining[0]
        ordered.append(nxt)
        remaining.remove(nxt)
        cur_in = nxt["out_idx"]
    return ordered, interp


def to_npu_format(layers):
    """Traduce a lo que carga el ESP32: bias plegado, multiplicador entero,
    limites de activacion, y pesos reordenados al layout del streaming."""
    npu = []
    for i, L in enumerate(layers):
        out_n, in_n = L["w"].shape
        assert in_n <= MAX_INPUT_WIDTH and out_n <= MAX_LAYER_WIDTH, \
            f"capa {i} ({in_n}->{out_n}) excede los limites de la NPU"

        # bias efectivo: se pliega la correccion del zero-point de entrada
        wsum = L["w"].sum(axis=1)                       # (out,)
        bias_folded = L["b"] - int(L["zp_in"]) * wsum   # int64 -> entra en int32

        real_mult = (L["s_in"] * L["s_w"]) / L["s_out"]
        mult_int = int(round(real_mult * (1 << QSHIFT)))

        # ReLU fusionado en las ocultas -> el piso es el zero-point de salida.
        # La ultima capa son logits crudos: rango completo int8.
        is_last = (i == len(layers) - 1)
        act_min = -128 if is_last else int(L["zp_out"])
        act_max = 127

        npu.append(dict(in_n=in_n, out_n=out_n, w=L["w"], bias=bias_folded,
                        mult_int=mult_int, real_mult=real_mult,
                        zp_out=int(L["zp_out"]), act_min=act_min, act_max=act_max,
                        zp_in=int(L["zp_in"])))
        print(f"  capa {i}: {in_n:4d} -> {out_n:3d}   mult_real={real_mult:.8f} "
              f"-> MULT_INT={mult_int:7d}   zp_out={int(L['zp_out']):4d}  "
              f"act_min={act_min}  bias[{bias_folded.min()}..{bias_folded.max()}]")
        if mult_int == 0:
            print("     AVISO: MULT_INT quedo en 0 -- QSHIFT es demasiado chico "
                  "para esta escala")
    return npu


def build_stream_bytes(npu):
    """Reordena TODOS los pesos al layout que recorre weight_stream.v:
    una unica secuencia de grupos de 8 bytes (uno por carril), capa tras
    capa. Para la capa L, ola w, entrada i: el carril l lleva el peso de la
    neurona (w*8+l); si esa neurona no existe (relleno de la ultima ola),
    va 0."""
    out = bytearray()
    for L in npu:
        waves = (L["out_n"] + NLANES - 1) // NLANES
        for w in range(waves):
            for i in range(L["in_n"]):
                for l in range(NLANES):
                    n = w * NLANES + l
                    val = int(L["w"][n, i]) if n < L["out_n"] else 0
                    out.append(val & 0xFF)
    return bytes(out)


def c_array(name, data, ctype="int8_t", per_line=20):
    body = []
    for i in range(0, len(data), per_line):
        body.append("  " + ", ".join(str(int(v)) for v in data[i:i + per_line]))
    return (f"const {ctype} {name}[{len(data)}] PROGMEM = {{\n"
            + ",\n".join(body) + "\n};\n")


def main():
    model, x_train, x_test, y_test = train()
    print("\nCuantizando a int8 (per-tensor, flujo estandar de TFLite)...")
    tflite_model = quantize(model, x_train)
    layers, interp = extract_layers(tflite_model)
    print(f"Capas densas encontradas: {len(layers)}")
    npu = to_npu_format(layers)

    # Precision del modelo YA cuantizado (referencia honesta de lo que
    # deberia lograr la FPGA).
    in_det, out_det = interp.get_input_details()[0], interp.get_output_details()[0]
    s_in, zp_in = in_det["quantization"]
    correct = 0
    for k in range(2000):
        q = np.clip(np.round(x_test[k] / s_in + zp_in), -128, 127).astype(np.int8)
        interp.set_tensor(in_det["index"], q.reshape(1, -1))
        interp.invoke()
        if int(np.argmax(interp.get_tensor(out_det["index"])[0])) == int(y_test[k]):
            correct += 1
    print(f"Precision int8 (TFLite, 2000 imagenes): {correct/2000*100:.2f}%")

    stream = build_stream_bytes(npu)
    print(f"\nPesos para SDRAM: {len(stream)} bytes ({len(stream)//NLANES} grupos)")

    # Imagenes de prueba, ya cuantizadas como las espera la NPU.
    imgs, labels, expected = [], [], []
    for k in range(N_TEST_IMAGES):
        q = np.clip(np.round(x_test[k] / s_in + zp_in), -128, 127).astype(np.int8)
        imgs.append(q)
        labels.append(int(y_test[k]))
        interp.set_tensor(in_det["index"], q.reshape(1, -1))
        interp.invoke()
        expected.append(int(np.argmax(interp.get_tensor(out_det["index"])[0])))

    os.makedirs(os.path.dirname(OUT_HEADER), exist_ok=True)
    with open(OUT_HEADER, "w") as f:
        f.write("// GENERADO POR tools/train_mnist_npu.py -- no editar a mano.\n")
        f.write("// Modelo MNIST entrenado en Keras y cuantizado con el conversor\n")
        f.write("// estandar de TFLite (int8, per-tensor). Ver el script para el\n")
        f.write("// detalle de como se traducen bias/multiplicador al formato de la NPU.\n")
        f.write("#pragma once\n#include <pgmspace.h>\n\n")
        f.write(f"#define QSHIFT {QSHIFT}\n")
        f.write(f"#define NUM_LAYERS {len(npu)}\n")
        f.write(f"#define N_TEST_IMAGES {N_TEST_IMAGES}\n")
        # Escala y zero-point de la ENTRADA: hacen falta para cuantizar
        # digitos nuevos (por ejemplo los que se dibujan en la pagina web).
        # El pixel normalizado (0..1) se convierte con:
        #   q = round(pixel / INPUT_SCALE) + INPUT_ZP,  recortado a [-128,127]
        f.write(f"#define INPUT_SCALE {float(s_in):.10f}f\n")
        f.write(f"#define INPUT_ZP {int(zp_in)}\n\n")

        f.write("const uint16_t LAYER_IN[NUM_LAYERS]  = {" + ", ".join(str(L["in_n"]) for L in npu) + "};\n")
        f.write("const uint16_t LAYER_OUT[NUM_LAYERS] = {" + ", ".join(str(L["out_n"]) for L in npu) + "};\n")
        f.write("const int32_t  LAYER_MULT[NUM_LAYERS] = {" + ", ".join(str(L["mult_int"]) for L in npu) + "};\n")
        f.write("const int32_t  LAYER_ZPOUT[NUM_LAYERS] = {" + ", ".join(str(L["zp_out"]) for L in npu) + "};\n")
        f.write("const int32_t  LAYER_ACTMIN[NUM_LAYERS] = {" + ", ".join(str(L["act_min"]) for L in npu) + "};\n")
        f.write("const int32_t  LAYER_ACTMAX[NUM_LAYERS] = {" + ", ".join(str(L["act_max"]) for L in npu) + "};\n\n")

        all_bias = np.concatenate([L["bias"] for L in npu])
        f.write(f"#define TOTAL_BIAS {len(all_bias)}\n")
        f.write(c_array("BIAS_FOLDED", all_bias, "int32_t", 8))
        f.write("\n")

        f.write(f"#define WEIGHT_STREAM_BYTES {len(stream)}\n")
        f.write(c_array("WEIGHT_STREAM", [np.int8(np.uint8(b)) for b in stream], "int8_t", 24))
        f.write("\n")

        f.write(f"#define IMG_SIZE {LAYER_SIZES[0]}\n")
        flat = np.concatenate(imgs)
        f.write(c_array("TEST_IMAGES", flat, "int8_t", 28))
        f.write("const uint8_t TEST_LABELS[N_TEST_IMAGES] = {" + ", ".join(str(v) for v in labels) + "};\n")
        f.write("// Lo que predice el modelo int8 en la PC: es la referencia exacta\n"
                "// que deberia reproducir la FPGA (no la etiqueta real).\n")
        f.write("const uint8_t TFLITE_PRED[N_TEST_IMAGES] = {" + ", ".join(str(v) for v in expected) + "};\n")

    print(f"\nEscrito: {os.path.normpath(OUT_HEADER)}")
    agree = sum(1 for a, b in zip(expected, labels) if a == b)
    print(f"En las {N_TEST_IMAGES} imagenes de prueba, el modelo int8 acierta {agree}/{N_TEST_IMAGES}")


if __name__ == "__main__":
    main()
