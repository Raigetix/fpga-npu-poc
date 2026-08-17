"""train_kws_npu.py -- Entrena un detector de PALABRAS CLAVE (keyword
spotting) sobre el dataset publico de Google (mini Speech Commands), lo
cuantiza a int8 con el conversor estandar de TFLite, y lo exporta al
formato de la NPU del proyecto.

8 palabras: down, go, left, no, right, stop, up, yes.

Por que una red DENSA sirve para esto: el paper de referencia del area
("Hello Edge", Zhang et al.) muestra que un MLP de 3 capas ocultas alcanza
~85% en esta tarea. No hace falta convolucion, asi que entra tal cual en la
NPU (que solo hace capas densas).

CADENA DE AUDIO (tiene que replicarse EXACTO en el ESP32, si no la
precision se desploma):
    audio 16 kHz mono, 1 segundo (16000 muestras, se recorta o rellena)
      -> STFT: ventanas de 640 muestras (40 ms), paso 320 (20 ms),
               FFT de 1024  ->  49 tramas x 513 bins
      -> magnitud
      -> banco de filtros mel: 40 filtros entre 20 y 4000 Hz
      -> log(mel + 1e-6)
      -> DCT-II, se quedan los primeros 10 coeficientes
      -> 49 x 10 = 490 caracteristicas

Para que el ESP32 no tenga que recalcular nada, este script EXPORTA las dos
matrices (mel y DCT) en el header. Asi la unica cuenta que hace el C es:
FFT -> magnitud -> multiplicar por la matriz mel -> log -> multiplicar por
la matriz DCT. Cualquier diferencia de formulas queda descartada de raiz.

Uso:  python train_kws_npu.py
"""

import os
import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import tensorflow as tf

# ---- Debe coincidir con mlp_engine_par_stream.v ----
QSHIFT = 20
NLANES = 8
MAX_LAYER_WIDTH = 256
MAX_INPUT_WIDTH = 1024

# ---- Parametros de audio/MFCC ----
SAMPLE_RATE = 16000
CLIP_SAMPLES = 16000        # 1 segundo
FRAME_LENGTH = 640          # 40 ms
FRAME_STEP   = 320          # 20 ms
FFT_LENGTH   = 1024
N_MEL        = 40
MEL_LO_HZ    = 20.0
MEL_HI_HZ    = 4000.0
N_MFCC       = 10
N_FRAMES     = 1 + (CLIP_SAMPLES - FRAME_LENGTH) // FRAME_STEP   # 49
N_FEATURES   = N_FRAMES * N_MFCC                                  # 490

WORDS = ["down", "go", "left", "no", "right", "stop", "up", "yes"]
HIDDEN = [256, 256, 144]   # el limite de la NPU por capa es 256
EPOCHS = 80          # con dropout + early stopping conviene dar mas margen
N_TEST_CLIPS = 200          # ejemplos de prueba que van al sketch

# El modelo ya NO se incrusta en el firmware como header: se escribe como
# archivos binarios que van al sistema de archivos (LittleFS) del ESP32.
# Motivos: (a) compilar 1,5 MB de literales numericos tardaba minutos, y
# (b) asi se puede cambiar de modelo sin recompilar el firmware -- que es
# el mismo espiritu de la NPU configurable por SPI del lado de la FPGA.
OUT_DATA = os.path.join(os.path.dirname(__file__), "..", "esp32",
                        "04_sdram_pesos", "kws_npu_poc", "data")


def mel_matrix():
    return tf.signal.linear_to_mel_weight_matrix(
        num_mel_bins=N_MEL,
        num_spectrogram_bins=FFT_LENGTH // 2 + 1,
        sample_rate=SAMPLE_RATE,
        lower_edge_hertz=MEL_LO_HZ,
        upper_edge_hertz=MEL_HI_HZ).numpy()


def dct_matrix():
    """DCT-II ortonormal de N_MEL -> N_MFCC, como la que aplica
    tf.signal.mfccs_from_log_mel_spectrograms (se queda con los primeros
    N_MFCC coeficientes)."""
    n = np.arange(N_MEL)
    M = np.zeros((N_MEL, N_MFCC), dtype=np.float32)
    for k in range(N_MFCC):
        M[:, k] = np.cos(np.pi * k * (2 * n + 1) / (2 * N_MEL))
    M *= np.sqrt(2.0 / N_MEL)
    M[:, 0] *= np.sqrt(0.5)      # normalizacion ortonormal del termino k=0
    return M


MEL = mel_matrix()
DCT = dct_matrix()


def features_from_audio(wav):
    """wav: float32 [-1,1], longitud CLIP_SAMPLES. Devuelve (490,) float32.
    Escrito con numpy (no con tf.signal.mfcc) para que sea evidente que es
    replicable paso a paso en C."""
    stft = tf.signal.stft(wav, frame_length=FRAME_LENGTH,
                          frame_step=FRAME_STEP, fft_length=FFT_LENGTH,
                          window_fn=tf.signal.hann_window)
    mag = tf.abs(stft).numpy()                    # (49, 513)
    mel = mag @ MEL                               # (49, 40)
    logmel = np.log(mel + 1e-6)
    mfcc = logmel @ DCT                           # (49, 10)
    return mfcc.astype(np.float32).reshape(-1)


def read_wav(path):
    raw = tf.io.read_file(path)
    wav, _ = tf.audio.decode_wav(raw, desired_channels=1)
    wav = tf.squeeze(wav, -1).numpy()
    if len(wav) < CLIP_SAMPLES:
        wav = np.pad(wav, (0, CLIP_SAMPLES - len(wav)))
    return wav[:CLIP_SAMPLES]


def augment(wav, rng):
    """Aumento de datos estandar para palabras clave: desplazar en el
    tiempo (la palabra no siempre arranca en el mismo instante), variar el
    volumen y agregar algo de ruido. Sin esto la red se memoriza los clips:
    daba 99% en entrenamiento contra 78% en validacion."""
    shift = rng.integers(-1600, 1601)          # +-100 ms
    out = np.roll(wav, shift)
    if shift > 0:  out[:shift] = 0
    elif shift < 0: out[shift:] = 0
    out = out * rng.uniform(0.7, 1.3)          # volumen
    out = out + rng.normal(0, rng.uniform(0.0, 0.01), size=out.shape)  # ruido
    return np.clip(out, -1.0, 1.0).astype(np.float32)


AUG_COPIES = 2   # copias aumentadas por clip de entrenamiento


def load_dataset():
    path = tf.keras.utils.get_file(
        "mini_speech_commands.zip",
        origin="http://storage.googleapis.com/download.tensorflow.org/data/mini_speech_commands.zip",
        extract=True, cache_subdir="datasets")
    base = os.path.join(os.path.dirname(path), "mini_speech_commands")
    if not os.path.isdir(base):
        base = os.path.join(os.path.dirname(path), "mini_speech_commands_extracted",
                            "mini_speech_commands")

    items = []
    for label, word in enumerate(WORDS):
        folder = os.path.join(base, word)
        files = sorted(os.listdir(folder))
        print(f"  {word}: {len(files)} clips")
        items += [(os.path.join(folder, fn), label) for fn in files]

    rng = np.random.default_rng(0)
    idx = rng.permutation(len(items))
    items = [items[i] for i in idx]
    n_val = len(items) // 10
    te_items, va_items, tr_items = items[:n_val], items[n_val:2 * n_val], items[2 * n_val:]

    def feats(lst, augmented):
        X, y = [], []
        for p, lab in lst:
            wav = read_wav(p)
            X.append(features_from_audio(wav)); y.append(lab)
            if augmented:
                for _ in range(AUG_COPIES):
                    X.append(features_from_audio(augment(wav, rng))); y.append(lab)
        return np.stack(X).astype(np.float32), np.array(y, dtype=np.int32)

    # Extraer las caracteristicas es lo que se lleva casi todo el tiempo,
    # y no cambia entre intentos de arquitectura -> se cachea en disco.
    cache = os.path.join(os.path.dirname(__file__), "kws_features.npz")
    if os.path.exists(cache):
        print(f"\nUsando caracteristicas cacheadas ({os.path.basename(cache)})")
        d = np.load(cache)
        return d["xtr"], d["ytr"], d["xva"], d["yva"], d["xte"], d["yte"]

    print(f"\nExtrayendo caracteristicas (entrenamiento con {AUG_COPIES} copias aumentadas)...")
    xtr, ytr = feats(tr_items, True)
    xva, yva = feats(va_items, False)
    xte, yte = feats(te_items, False)
    np.savez_compressed(cache, xtr=xtr, ytr=ytr, xva=xva, yva=yva, xte=xte, yte=yte)
    print(f"  cacheadas en {os.path.basename(cache)} (borralo si cambias los parametros de MFCC)")
    return xtr, ytr, xva, yva, xte, yte


# Con 144 neuronas y dropout 0.3 la red quedaba corta (85% en
# entrenamiento): el cuello dejo de ser memorizacion y paso a ser
# capacidad. Se agranda y se afloja un poco la regularizacion.
DROPOUT = 0.2


def train(xtr, ytr, xva, yva):
    layers = [tf.keras.layers.Input(shape=(N_FEATURES,))]
    for h in HIDDEN:
        layers.append(tf.keras.layers.Dense(h, activation="relu"))
        layers.append(tf.keras.layers.Dropout(DROPOUT))
    layers.append(tf.keras.layers.Dense(len(WORDS)))
    model = tf.keras.Sequential(layers)
    model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                  loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                  metrics=["accuracy"])
    # se queda con los mejores pesos segun validacion, no con el ultimo
    # epoch (que suele estar sobreajustado)
    cb = tf.keras.callbacks.EarlyStopping(monitor="val_accuracy", patience=15,
                                          restore_best_weights=True, verbose=1)
    model.fit(xtr, ytr, validation_data=(xva, yva), epochs=EPOCHS,
              batch_size=64, callbacks=[cb], verbose=2)
    return model


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
        if mult_int == 0:
            print("     AVISO: MULT_INT quedo en 0 -- QSHIFT demasiado chico")
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


def cfloat(v):
    """Literal float valido en C. Ojo: "%.7g" de 0.0 da "0", y "0f" NO es un
    literal valido en C++ (lo toma como sufijo de usuario) -- como el banco
    de filtros mel es casi todo ceros, eso rompia la compilacion con miles
    de errores. Se fuerza siempre un punto decimal."""
    s = f"{float(v):.7g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def c_array(name, data, ctype="int8_t", per_line=20, fmt=str):
    body = []
    for i in range(0, len(data), per_line):
        body.append("  " + ", ".join(fmt(v) for v in data[i:i + per_line]))
    return (f"const {ctype} {name}[{len(data)}] PROGMEM = {{\n"
            + ",\n".join(body) + "\n};\n")


def main():
    print("Cargando dataset (la primera vez descarga ~182 MB)...")
    xtr, ytr, xva, yva, xte, yte = load_dataset()
    print(f"\nentrenamiento={len(xtr)}  validacion={len(xva)}  prueba={len(xte)}")

    model = train(xtr, ytr, xva, yva)
    _, acc = model.evaluate(xte, yte, verbose=0)
    print(f"\nPrecision float32 sobre el set de prueba: {acc*100:.2f}%")

    print("\nCuantizando a int8 (per-tensor)...")
    tflite_model = quantize(model, xtr)
    layers, interp = extract_layers(tflite_model)
    npu = to_npu_format(layers)

    in_det, out_det = interp.get_input_details()[0], interp.get_output_details()[0]
    s_in, zp_in = in_det["quantization"]

    def quant_feat(f):
        return np.clip(np.round(f / s_in + zp_in), -128, 127).astype(np.int8)

    correct = 0
    preds = []
    for k in range(len(xte)):
        q = quant_feat(xte[k])
        interp.set_tensor(in_det["index"], q.reshape(1, -1))
        interp.invoke()
        p = int(np.argmax(interp.get_tensor(out_det["index"])[0]))
        preds.append(p)
        if p == int(yte[k]):
            correct += 1
    print(f"Precision int8 (TFLite): {correct/len(xte)*100:.2f}%")

    stream = build_stream_bytes(npu)
    print(f"\nPesos para SDRAM: {len(stream)} bytes ({len(stream)//NLANES} grupos)")

    n_clips = min(N_TEST_CLIPS, len(xte))
    os.makedirs(OUT_DATA, exist_ok=True)

    # ---- model.bin: todo lo que el firmware necesita para correr el modelo ----
    # Formato (little endian).  Cabecera, capas, bias, pesos y las tablas del
    # MFCC. El firmware lo lee de a bloques: los pesos se mandan directo a la
    # SDRAM sin pasar por RAM.
    import struct
    mb = bytearray()
    mb += b"NPU1"
    mb += struct.pack("<HHH", len(npu), N_FEATURES, len(WORDS))
    mb += struct.pack("<fii", float(s_in), int(zp_in), QSHIFT)
    for L in npu:
        mb += struct.pack("<HHiiii", L["in_n"], L["out_n"], L["mult_int"],
                          L["zp_out"], L["act_min"], L["act_max"])
    all_bias = np.concatenate([L["bias"] for L in npu]).astype(np.int32)
    mb += struct.pack("<I", len(all_bias)) + all_bias.tobytes()
    mb += struct.pack("<I", len(stream)) + bytes(stream)
    # parametros y tablas del MFCC
    mb += struct.pack("<HHHHHHH", N_MEL, N_MFCC, FFT_LENGTH // 2 + 1, N_FRAMES,
                      FRAME_LENGTH, FRAME_STEP, FFT_LENGTH)
    mb += MEL.astype(np.float32).tobytes()
    mb += DCT.astype(np.float32).tobytes()
    # nombres de las palabras, separados por barra-cero
    words_blob = ("\0".join(WORDS) + "\0").encode()
    mb += struct.pack("<H", len(words_blob)) + words_blob

    with open(os.path.join(OUT_DATA, "model.bin"), "wb") as f:
        f.write(mb)
    print(f"\nmodel.bin: {len(mb)} bytes")

    # ---- test.bin: clips de prueba para el benchmark (opcional) ----
    tb = bytearray()
    tb += struct.pack("<HH", n_clips, N_FEATURES)
    for k in range(n_clips):
        tb += quant_feat(xte[k]).tobytes()
    tb += bytes(int(v) for v in yte[:n_clips])
    tb += bytes(int(v) for v in preds[:n_clips])
    with open(os.path.join(OUT_DATA, "test.bin"), "wb") as f:
        f.write(tb)
    print(f"test.bin:  {len(tb)} bytes ({n_clips} clips)")
    print(f"\nEscritos en: {os.path.normpath(OUT_DATA)}")
    print("Subilos al ESP32 con:  pio run -t uploadfs")


if __name__ == "__main__":
    main()
