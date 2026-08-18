"""train_gestures_npu.py -- entrena un clasificador de gestos ("varita
magica": mover el celular haciendo una figura en el aire) a partir de
grabaciones del acelerometro/giroscopio del navegador (ver
tools/gestures/gestos_data.jsonl, grabado con la pagina de captura), lo
cuantiza a int8 y lo exporta al formato de la NPU del proyecto.

Reutiliza el mismo formato NPU (cuantizacion, capas, stream de pesos) que
train_kws_npu.py -- esa parte no depende del dominio (audio, imagen o
sensor de movimiento, para la NPU es solo un vector de N_FEATURES
numeros). Lo unico que cambia es como se arma ese vector: en vez de
MFCC de audio, es la secuencia de acelerometro+giroscopio (6 canales)
remuestreada a un largo fijo.

Uso:  python train_gestures_npu.py
"""

import os
import json
import struct

import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf

# ---- Debe coincidir con mlp_engine_par_stream.v ----
QSHIFT = 20
NLANES = 8
MAX_LAYER_WIDTH = 256
MAX_INPUT_WIDTH = 1024

# ---- Parametros de la señal de movimiento ----
GESTURES = ["circulo", "rayo", "invocar", "desviar"]
CHANNELS = ["ax", "ay", "az", "gx", "gy", "gz"]
N_TIMESTEPS = 30            # remuestreo fijo de cada grabacion (variable, ~90 muestras)
N_CHANNELS = len(CHANNELS)
N_FEATURES = N_TIMESTEPS * N_CHANNELS

# Escalas de normalizacion -- llevan acelerometro (m/s^2) y giroscopio
# (grados/s) a, mas o menos, [-1, 1] antes de cuantizar. Elegidas mirando
# los rangos reales de las grabaciones (percentil 95 ~15-18 y ~300-480
# respectivamente) con margen para los picos.
ACCEL_SCALE = 40.0
GYRO_SCALE = 500.0

HIDDEN = [64, 32]     # dataset chico (grabado a mano) -> red chica
EPOCHS = 150
DROPOUT = 0.3
AUG_COPIES = 40       # ~20 grabaciones reales por gesto -> aumento agresivo

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_FILE = os.path.join(TOOLS_DIR, "gestures", "gestos_data.jsonl")
OUT_DATA = os.path.join(TOOLS_DIR, "..", "esp32", "04_sdram_pesos", "gesture_npu_poc", "data")


# ================= remuestreo de una grabacion a largo fijo =================

def resample_recording(rec):
    """rec['samples']: lista de {t, ax,ay,az,gx,gy,gz} (t en ms, arranca
    en 0). Devuelve (N_TIMESTEPS, N_CHANNELS) float32, interpolado
    linealmente sobre el tiempo normalizado [0,1] y normalizado a
    [-1,1] por canal."""
    s = rec["samples"]
    t = np.array([p["t"] for p in s], dtype=np.float64)
    t = t - t[0]
    if t[-1] <= 0:
        t = np.arange(len(s), dtype=np.float64)   # respaldo si no hay timestamps utiles
    tn = t / t[-1]
    grid = np.linspace(0.0, 1.0, N_TIMESTEPS)
    chans = [np.interp(grid, tn, [p[k] for p in s]) for k in CHANNELS]
    out = np.stack(chans, axis=1).astype(np.float32)             # (N_TIMESTEPS, 6)
    scale = np.array([ACCEL_SCALE] * 3 + [GYRO_SCALE] * 3, dtype=np.float32)
    return np.clip(out / scale, -1.0, 1.0)


def load_recordings():
    by_label = {g: [] for g in GESTURES}
    if not os.path.exists(DATA_FILE):
        raise SystemExit(f"no encontre {DATA_FILE} -- graba gestos con la pagina de captura primero")
    with open(DATA_FILE, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("label") not in by_label or len(rec.get("samples", [])) < 10:
                continue
            by_label[rec["label"]].append(resample_recording(rec))
    return by_label


def augment(x, rng):
    """x: (N_TIMESTEPS, N_CHANNELS) ya normalizado. Reescalado temporal
    leve (el gesto no siempre dura lo mismo), reescalado de amplitud (mas
    o menos fuerza) y ruido -- mismo espiritu que el aumento de audio en
    train_kws_npu.py: el dataset real es chico (grabado a mano), sin esto
    la red memoriza las ~20 grabaciones en vez de aprender la forma del
    gesto."""
    n = x.shape[0]
    warp = rng.uniform(0.85, 1.15)
    src_idx = np.clip(np.linspace(0, n - 1, n) * warp, 0, n - 1)
    out = np.stack([np.interp(src_idx, np.arange(n), x[:, c]) for c in range(x.shape[1])], axis=1)
    out = out * rng.uniform(0.8, 1.2)
    out = out + rng.normal(0.0, rng.uniform(0.0, 0.05), size=out.shape)
    return np.clip(out, -1.0, 1.0).astype(np.float32)


def split_list(items, rng, frac_val=0.15, frac_test=0.15):
    if len(items) < 6:
        return items, items, items    # muy pocos para partir
    idx = rng.permutation(len(items))
    n_val = max(1, round(len(items) * frac_val))
    n_test = max(1, round(len(items) * frac_test))
    te = [items[i] for i in idx[:n_test]]
    va = [items[i] for i in idx[n_test:n_test + n_val]]
    tr = [items[i] for i in idx[n_test + n_val:]]
    return tr, va, te


def build_dataset():
    rng = np.random.default_rng(0)
    by_label = load_recordings()
    for g in GESTURES:
        if len(by_label[g]) < 8:
            raise SystemExit(f"muy pocas grabaciones de '{g}' ({len(by_label[g])}) -- graba al menos 15-20")

    Xtr, ytr, Xva, yva, Xte, yte = [], [], [], [], [], []
    for label_idx, g in enumerate(GESTURES):
        items = by_label[g]
        tr, va, te = split_list(items, rng)
        for x in tr:
            Xtr.append(x.reshape(-1)); ytr.append(label_idx)
            for _ in range(AUG_COPIES):
                Xtr.append(augment(x, rng).reshape(-1)); ytr.append(label_idx)
        for x in va:
            Xva.append(x.reshape(-1)); yva.append(label_idx)
        for x in te:
            Xte.append(x.reshape(-1)); yte.append(label_idx)
        print(f"  '{g}': {len(items)} grabaciones -> {len(tr) * (AUG_COPIES + 1)} de entrenamiento (con aumento)")

    def finalize(X, y):
        X = np.stack(X).astype(np.float32)
        y = np.array(y, dtype=np.int32)
        idx = rng.permutation(len(X))
        return X[idx], y[idx]

    xtr, ytr = finalize(Xtr, ytr)
    xva, yva = finalize(Xva, yva)
    xte, yte = finalize(Xte, yte)
    print(f"\nDataset: entrenamiento={len(xtr)}  validacion={len(xva)}  prueba={len(xte)}")
    return xtr, ytr, xva, yva, xte, yte


# ================= entrenamiento =================

def train(xtr, ytr, xva, yva):
    layers = [tf.keras.layers.Input(shape=(N_FEATURES,))]
    for h in HIDDEN:
        layers.append(tf.keras.layers.Dense(h, activation="relu"))
        layers.append(tf.keras.layers.Dropout(DROPOUT))
    layers.append(tf.keras.layers.Dense(len(GESTURES)))
    model = tf.keras.Sequential(layers)
    model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                  loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                  metrics=["accuracy"])
    cb = tf.keras.callbacks.EarlyStopping(monitor="val_accuracy", patience=25,
                                          restore_best_weights=True, verbose=1)
    model.fit(xtr, ytr, validation_data=(xva, yva), epochs=EPOCHS,
              batch_size=32, callbacks=[cb], verbose=2)
    return model


# ================= formato NPU (identico a train_kws_npu.py -- no
# depende del dominio, ver ahi el detalle de por que cada paso) =================

def quantize(model, xtr):
    def rep():
        for i in range(min(300, len(xtr))):
            yield [xtr[i:i + 1].astype(np.float32)]
    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]
    conv.representative_dataset = rep
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    conv.inference_input_type = tf.int8
    conv.inference_output_type = tf.int8
    conv._experimental_disable_per_channel = True
    return conv.convert()


def extract_layers(tflite_model):
    interp = tf.lite.Interpreter(model_content=tflite_model)
    interp.allocate_tensors()
    details = {d["index"]: d for d in interp.get_tensor_details()}
    layers = []
    for op in interp._get_ops_details():
        if op["op_name"] != "FULLY_CONNECTED":
            continue
        in_idx, w_idx, b_idx = op["inputs"][0], op["inputs"][1], op["inputs"][2]
        out_idx = op["outputs"][0]
        layers.append(dict(
            w=interp.get_tensor(w_idx).astype(np.int32),
            b=interp.get_tensor(b_idx).astype(np.int64),
            s_in=float(details[in_idx]["quantization"][0]),
            zp_in=int(details[in_idx]["quantization"][1]),
            s_w=float(details[w_idx]["quantization"][0]),
            s_out=float(details[out_idx]["quantization"][0]),
            zp_out=int(details[out_idx]["quantization"][1]),
            in_idx=in_idx, out_idx=out_idx))
    ordered, remaining, cur_in = [], layers[:], None
    while remaining:
        if cur_in is None:
            nxt = min(remaining, key=lambda L: L["in_idx"])
        else:
            cand = [L for L in remaining if L["in_idx"] == cur_in]
            nxt = cand[0] if cand else remaining[0]
        ordered.append(nxt); remaining.remove(nxt); cur_in = nxt["out_idx"]
    return ordered, interp


def to_npu_format(layers):
    npu = []
    for i, L in enumerate(layers):
        out_n, in_n = L["w"].shape
        assert in_n <= MAX_INPUT_WIDTH and out_n <= MAX_LAYER_WIDTH, \
            f"capa {i} ({in_n}->{out_n}) excede los limites de la NPU"
        wsum = L["w"].sum(axis=1)
        bias_folded = L["b"] - int(L["zp_in"]) * wsum
        real_mult = (L["s_in"] * L["s_w"]) / L["s_out"]
        mult_int = int(round(real_mult * (1 << QSHIFT)))
        is_last = (i == len(layers) - 1)
        act_min = -128 if is_last else int(L["zp_out"])
        npu.append(dict(in_n=in_n, out_n=out_n, w=L["w"], bias=bias_folded,
                        mult_int=mult_int, zp_out=int(L["zp_out"]),
                        act_min=act_min, act_max=127))
        print(f"  capa {i}: {in_n:4d} -> {out_n:3d}  MULT_INT={mult_int:7d} "
              f"zp_out={int(L['zp_out']):4d} act_min={act_min}")
    return npu


def build_stream_bytes(npu):
    out = bytearray()
    for L in npu:
        waves = (L["out_n"] + NLANES - 1) // NLANES
        for w in range(waves):
            for i in range(L["in_n"]):
                for l in range(NLANES):
                    n = w * NLANES + l
                    out.append((int(L["w"][n, i]) if n < L["out_n"] else 0) & 0xFF)
    return bytes(out)


def export(model, xtr, xte, yte):
    print("\nCuantizando a int8 (per-tensor)...")
    tflite_model = quantize(model, xtr)
    layers, interp = extract_layers(tflite_model)
    npu = to_npu_format(layers)

    in_det, out_det = interp.get_input_details()[0], interp.get_output_details()[0]
    s_in, zp_in = in_det["quantization"]

    def quant_feat(f):
        return np.clip(np.round(f / s_in + zp_in), -128, 127).astype(np.int8)

    conf = np.zeros((len(GESTURES), len(GESTURES)), dtype=np.int64)
    preds = []
    for k in range(len(xte)):
        q = quant_feat(xte[k])
        interp.set_tensor(in_det["index"], q.reshape(1, -1))
        interp.invoke()
        p = int(np.argmax(interp.get_tensor(out_det["index"])[0]))
        preds.append(p)
        conf[yte[k], p] += 1

    acc = np.trace(conf) / max(conf.sum(), 1)
    print(f"\nPrecision int8 sobre el set de prueba: {acc * 100:.2f}%")
    for c, g in enumerate(GESTURES):
        total_c = conf[c].sum()
        recall = conf[c, c] / total_c if total_c else 0.0
        print(f"  '{g}': {recall * 100:.1f}% de aciertos ({int(conf[c, c])}/{int(total_c)})")

    # ---- model.bin: cabecera "NPUG" (variante gestos de "NPU1") -- capas,
    # bias y stream de pesos IDENTICO al formato de audio; en vez de las
    # tablas de mel/DCT, lleva los parametros de remuestreo (N_TIMESTEPS/
    # N_CHANNELS/escalas) para que el firmware sepa como convertir una
    # grabacion cruda del navegador al mismo vector de entrada. ----
    stream = build_stream_bytes(npu)
    words_blob = ("\0".join(GESTURES) + "\0").encode()

    mb = bytearray()
    mb += b"NPUG"
    mb += struct.pack("<HHH", len(npu), N_FEATURES, len(GESTURES))
    mb += struct.pack("<fii", float(s_in), int(zp_in), QSHIFT)
    for L in npu:
        mb += struct.pack("<HHiiii", L["in_n"], L["out_n"], L["mult_int"],
                          L["zp_out"], L["act_min"], L["act_max"])
    all_bias = np.concatenate([L["bias"] for L in npu]).astype(np.int32)
    mb += struct.pack("<I", len(all_bias)) + all_bias.tobytes()
    mb += struct.pack("<I", len(stream)) + bytes(stream)
    mb += struct.pack("<HHff", N_TIMESTEPS, N_CHANNELS, ACCEL_SCALE, GYRO_SCALE)
    mb += struct.pack("<H", len(words_blob)) + words_blob

    n_clips = len(xte)
    tb = bytearray()
    tb += struct.pack("<HH", n_clips, N_FEATURES)
    for k in range(n_clips):
        tb += quant_feat(xte[k]).tobytes()
    tb += bytes(int(v) for v in yte[:n_clips])
    tb += bytes(int(v) for v in preds[:n_clips])

    os.makedirs(OUT_DATA, exist_ok=True)
    with open(os.path.join(OUT_DATA, "model.bin"), "wb") as f:
        f.write(mb)
    with open(os.path.join(OUT_DATA, "test.bin"), "wb") as f:
        f.write(tb)
    print(f"\nEscritos en: {os.path.normpath(OUT_DATA)}")
    print(f"  model.bin: {len(mb)} bytes   test.bin: {len(tb)} bytes")


def main():
    print("=== Entrenar clasificador de gestos (varita magica) ===\n")
    xtr, ytr, xva, yva, xte, yte = build_dataset()
    model = train(xtr, ytr, xva, yva)
    _, acc = model.evaluate(xte, yte, verbose=0)
    print(f"\nPrecision float32 sobre el set de prueba: {acc * 100:.2f}%")
    export(model, xtr, xte, yte)
    print("\nSubilo al ESP32 con:  pio run -t uploadfs")


if __name__ == "__main__":
    main()
