"""train_one_word_npu.py -- Graba tu voz diciendo UNA palabra, arma un
dataset (con negativos del dataset publico de Google + tu propio ruido de
fondo), entrena un detector binario ("la dijo" / "no la dijo"), lo cuantiza
a int8 y lo deja listo como model.bin + test.bin para subir al ESP32.

Reutiliza la cadena de audio (MFCC), la cuantizacion y el formato de salida
de train_kws_npu.py, que ya corren en el hardware -- asi no hay riesgo de
que las formulas del MFCC diverjan entre este script y el firmware.

Uso:  python train_one_word_npu.py
"""

import os
import sys
import time
from math import gcd

import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

sys.path.insert(0, os.path.dirname(__file__))
import train_kws_npu as base   # MFCC, cuantizacion, formato NPU (ya probados en hardware)

import tensorflow as tf
import sounddevice as sd
from scipy.io import wavfile

SAMPLE_RATE = base.SAMPLE_RATE          # 16000, fijo (lo espera el firmware)
CLIP_SAMPLES = base.CLIP_SAMPLES        # 16000 (1 segundo)
RECORD_SECONDS = 1.6                    # margen para que la palabra entre completa
GOOGLE_WORDS = base.WORDS               # ["down","go","left","no","right","stop","up","yes"]

HIDDEN = [256, 256, 128, 64, 32]   # ~234k pesos, embudo angosto al final
EPOCHS = 60
DROPOUT = 0.3

POS_AUG_COPIES = 25    # los positivos son pocos -> aumento agresivo
BG_AUG_COPIES = 5
SYN_SILENCE_TR = 150
SYN_SILENCE_VA = 20
SYN_SILENCE_TE = 20
MAX_GOOGLE_TRAIN_NEG = 3000   # limite para no ahogar a los positivos en el entrenamiento

SENSITIVITY = {
    "1": ("estricta", 0.7),
    "2": ("equilibrada", 1.0),
    "3": ("sensible", 1.5),
}

TOOLS_DIR = os.path.dirname(__file__)
ROOT = os.path.join(TOOLS_DIR, "my_words")             # grabaciones, una carpeta por palabra
MODELS_DIR = os.path.join(TOOLS_DIR, "my_words_models")  # modelos combinados exportados
ESP32_DATA_DIR = os.path.join(TOOLS_DIR, "..", "esp32", "04_sdram_pesos", "kws_npu_poc", "data")


# ================= utilidades de consola =================

def ask_int(prompt, default):
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        print("  no entendi, uso el valor por defecto")
        return default


def ask_choice_sensibilidad():
    print("\n¿Que tan sensible tiene que ser el detector?")
    print("  1) estricta     - casi no se activa por error, pero a veces no te va a escuchar")
    print("  2) equilibrada  - punto medio (recomendada)")
    print("  3) sensible     - te escucha casi siempre, pero se puede activar solo mas seguido")
    while True:
        raw = input("Elegi 1/2/3 [2]: ").strip() or "2"
        if raw in SENSITIVITY:
            return SENSITIVITY[raw]
        print("  opcion invalida")


# ================= grabacion =================

def _native_samplerate():
    try:
        sd.check_input_settings(samplerate=SAMPLE_RATE)
        return SAMPLE_RATE
    except Exception:
        dev = sd.query_devices(kind="input")
        return int(dev["default_samplerate"])


REC_SR = _native_samplerate()


def record_raw(seconds):
    n = int(round(REC_SR * seconds))
    audio = sd.rec(n, samplerate=REC_SR, channels=1, dtype="float32")
    sd.wait()
    audio = audio[:, 0]
    if REC_SR != SAMPLE_RATE:
        from scipy.signal import resample_poly
        g = gcd(SAMPLE_RATE, REC_SR)
        audio = resample_poly(audio, SAMPLE_RATE // g, REC_SR // g).astype(np.float32)
    return audio


def center_on_energy(audio):
    """Recorta la ventana de 1s con mas energia -- asi no depende de que el
    usuario diga la palabra justo al toque de la cuenta regresiva."""
    n = len(audio)
    if n <= CLIP_SAMPLES:
        return np.pad(audio, (0, CLIP_SAMPLES - n)).astype(np.float32), float(np.abs(audio).max() if n else 0.0)
    energy = audio.astype(np.float64) ** 2
    csum = np.cumsum(np.insert(energy, 0, 0.0))
    window_energy = csum[CLIP_SAMPLES:] - csum[:-CLIP_SAMPLES]
    start = int(np.argmax(window_energy))
    clip = audio[start:start + CLIP_SAMPLES]
    return clip.astype(np.float32), float(np.abs(clip).max())


def save_wav(path, clip):
    pcm = np.clip(clip * 32767.0, -32768, 32767).astype(np.int16)
    wavfile.write(path, SAMPLE_RATE, pcm)


def record_word_clip(word):
    for c in ("3", "2", "1", "YA!"):
        print(f"  {c}  ", end="\r", flush=True)
        time.sleep(0.5)
    print("      ", end="\r")
    audio = record_raw(RECORD_SECONDS)
    return center_on_energy(audio)


def list_wavs(folder):
    return sorted(f for f in os.listdir(folder) if f.endswith(".wav")) if os.path.isdir(folder) else []


def record_session(word):
    pos_dir = os.path.join(ROOT, word, "positive")
    bg_dir = os.path.join(ROOT, word, "background")
    os.makedirs(pos_dir, exist_ok=True)
    os.makedirs(bg_dir, exist_ok=True)

    existing = list_wavs(pos_dir)
    do_record = True
    if existing:
        print(f"\nYa hay {len(existing)} grabaciones de '{word}' en {pos_dir}")
        ans = input("¿Agregar mas (a), volver a grabar todo (r), o usar las que hay (Enter)? ").strip().lower()
        if ans == "r":
            for f in existing:
                os.remove(os.path.join(pos_dir, f))
            existing = []
        elif ans != "a":
            do_record = False

    if do_record:
        n = ask_int(f"¿Cuantas repeticiones de '{word}' queres grabar? (min. 40 recomendado)", 60)
        input(f"\nApreta Enter y quedate cerca del microfono. Vas a decir '{word}' {n} veces.\n")
        start_idx = len(list_wavs(pos_dir))
        weak = []
        for i in range(n):
            idx = start_idx + i + 1
            print(f"[{i + 1}/{n}] Decir '{word}'...")
            clip, peak = record_word_clip(word)
            save_wav(os.path.join(pos_dir, f"rec_{idx:03d}.wav"), clip)
            tag = "bien" if peak > 0.05 else "flojo, quizas repetir"
            print(f"      pico={peak:.3f}  ({tag})")
            if peak <= 0.05:
                weak.append(idx)
        if weak:
            redo = input(f"\nGrabaciones flojas: {weak}. ¿Repetirlas ahora? (Enter=si, n=no): ").strip().lower()
            if redo != "n":
                for idx in weak:
                    print(f"Repetir '{word}' (era la #{idx})...")
                    clip, peak = record_word_clip(word)
                    save_wav(os.path.join(pos_dir, f"rec_{idx:03d}.wav"), clip)
                    print(f"      pico={peak:.3f}")

    existing_bg = list_wavs(bg_dir)
    if not existing_bg:
        print("\nAhora grabemos ruido de fondo / silencio de tu ambiente,")
        print("para que el modelo aprenda a NO activarse solo porque hay ruido.")
        nbg = ask_int("¿Cuantas grabaciones de fondo (1s cada una)?", 20)
        input("Apreta Enter y quedate en silencio / con el ruido normal del ambiente...\n")
        for i in range(nbg):
            print(f"  [{i + 1}/{nbg}] grabando fondo...", end="\r", flush=True)
            audio = record_raw(1.05)
            clip, _ = center_on_energy(audio)
            save_wav(os.path.join(bg_dir, f"bg_{i + 1:03d}.wav"), clip)
        print()

    return pos_dir, bg_dir


# ================= dataset =================

def read_wav_any(path):
    sr, data = wavfile.read(path)
    data = data.astype(np.float32) / 32768.0
    if len(data) < CLIP_SAMPLES:
        data = np.pad(data, (0, CLIP_SAMPLES - len(data)))
    return data[:CLIP_SAMPLES]


def augment_channel(wav, rng):
    """Preenfasis con coeficiente aleatorio: simula que la palabra se grabo
    con un microfono de otra coloracion espectral (mas grave o mas agudo).
    Sin esto la red aprende a esperar la firma exacta del microfono de
    entrenamiento y no generaliza a otros microfonos (confirmado: el mismo
    firmware reconoce perfecto por PC con el auricular de entrenamiento pero
    falla sistematicamente con el microfono de un celular distinto)."""
    alpha = rng.uniform(-0.6, 0.6)
    out = np.empty_like(wav)
    out[0] = wav[0]
    out[1:] = wav[1:] - alpha * wav[:-1]
    return out.astype(np.float32)


def augment_with_bg(wav, bg_pool, rng):
    out = base.augment(wav, rng)
    out = augment_channel(out, rng)
    if bg_pool and rng.random() < 0.6:
        bg = bg_pool[rng.integers(len(bg_pool))]
        snr = rng.uniform(0.1, 0.5)
        out = np.clip(out + bg * snr, -1.0, 1.0).astype(np.float32)
    return out


def split_list(items, rng, frac_val=0.1, frac_test=0.1):
    if len(items) < 4:
        return items, items, items    # muy pocos para partir -- se reusan (solo pasa en pruebas rapidas)
    idx = rng.permutation(len(items))
    n_val = max(1, round(len(items) * frac_val))
    n_test = max(1, round(len(items) * frac_test))
    te = [items[i] for i in idx[:n_test]]
    va = [items[i] for i in idx[n_test:n_test + n_val]]
    tr = [items[i] for i in idx[n_test + n_val:]]
    return tr, va, te


def google_dataset_dir():
    path = tf.keras.utils.get_file(
        "mini_speech_commands.zip",
        origin="http://storage.googleapis.com/download.tensorflow.org/data/mini_speech_commands.zip",
        extract=True, cache_subdir="datasets")
    base_dir = os.path.join(os.path.dirname(path), "mini_speech_commands")
    if not os.path.isdir(base_dir):
        base_dir = os.path.join(os.path.dirname(path), "mini_speech_commands_extracted", "mini_speech_commands")
    return base_dir


def load_google_word_features():
    cache = os.path.join(TOOLS_DIR, "google_words_features_cache.npz")
    if os.path.exists(cache):
        d = np.load(cache)
        return {w: d[w] for w in d.files}
    print("Extrayendo caracteristicas del dataset publico de Google (una sola vez, se cachea; ~1-2 minutos)...", flush=True)
    folder_base = google_dataset_dir()
    by_word = {}
    for w in GOOGLE_WORDS:
        folder = os.path.join(folder_base, w)
        files = sorted(os.listdir(folder))
        feats = []
        for i, fn in enumerate(files):
            feats.append(base.features_from_audio(base.read_wav(os.path.join(folder, fn))))
            if (i + 1) % 200 == 0:
                print(f"  {w}: {i + 1}/{len(files)}...", flush=True)
        by_word[w] = np.stack(feats).astype(np.float32)
        print(f"  {w}: {len(feats)} listo", flush=True)
    np.savez_compressed(cache, **by_word)
    return by_word


def google_negative_split(words, rng):
    chosen = {w.lower() for w in words}
    by_word = load_google_word_features()
    tr, va, te = [], [], []
    for w in GOOGLE_WORDS:
        if w.lower() in chosen:
            print(f"  ('{w}' se excluye de los negativos: coincide con una palabra elegida)")
            continue
        feats = by_word[w]
        idx = rng.permutation(len(feats))
        n_val = len(feats) // 10
        n_test = len(feats) // 10
        te.append(feats[idx[:n_test]])
        va.append(feats[idx[n_test:n_test + n_val]])
        tr.append(feats[idx[n_test + n_val:]])
    return np.concatenate(tr), np.concatenate(va), np.concatenate(te)


def synth_silence(n, rng):
    return [base.features_from_audio(rng.normal(0.0, rng.uniform(0.0, 0.02), CLIP_SAMPLES).astype(np.float32))
            for _ in range(n)]


def pos_feats(wavs, augmented, bg_pool, rng):
    X = []
    for wav in wavs:
        X.append(base.features_from_audio(wav))
        if augmented:
            for _ in range(POS_AUG_COPIES):
                X.append(base.features_from_audio(augment_with_bg(wav, bg_pool, rng)))
    return X


def bg_feats(wavs, augmented, rng):
    X = []
    for wav in wavs:
        X.append(base.features_from_audio(wav))
        if augmented:
            for _ in range(BG_AUG_COPIES):
                X.append(base.features_from_audio(augment_channel(base.augment(wav, rng), rng)))
    return X


def build_dataset(words):
    """words: lista de palabras propias. Clase 0 = 'otro' (ni una ni otra),
    clases 1..N = cada palabra, en el mismo orden que la lista."""
    rng = np.random.default_rng(0)

    per_word = {}
    all_bg_tr, all_bg_va, all_bg_te = [], [], []
    for w in words:
        pos_dir, bg_dir = record_session(w)
        pos_wavs = [read_wav_any(os.path.join(pos_dir, f)) for f in list_wavs(pos_dir)]
        bg_wavs = [read_wav_any(os.path.join(bg_dir, f)) for f in list_wavs(bg_dir)]
        if len(pos_wavs) < 10:
            raise SystemExit(f"Muy pocas grabaciones de '{w}' -- graba al menos 10-20 antes de entrenar.")
        pos_tr, pos_va, pos_te = split_list(pos_wavs, rng)
        bg_tr, bg_va, bg_te = split_list(bg_wavs, rng) if bg_wavs else ([], [], [])
        per_word[w] = dict(pos_tr=pos_tr, pos_va=pos_va, pos_te=pos_te,
                           bg_tr=bg_tr, bg_va=bg_va, bg_te=bg_te, n_clips=len(pos_wavs))
        all_bg_tr += bg_tr; all_bg_va += bg_va; all_bg_te += bg_te

    print("\nExtrayendo caracteristicas de tus grabaciones (con aumento de datos)...")
    Xtr, ytr, Xva, yva, Xte, yte = [], [], [], [], [], []

    for class_idx, w in enumerate(words, start=1):
        d = per_word[w]
        bg_pool_tr = d["bg_tr"] or all_bg_tr
        bg_pool_va = d["bg_va"] or all_bg_va
        bg_pool_te = d["bg_te"] or all_bg_te
        ptr = pos_feats(d["pos_tr"], True, bg_pool_tr, rng)
        pva = pos_feats(d["pos_va"], False, bg_pool_va, rng)
        pte = pos_feats(d["pos_te"], False, bg_pool_te, rng)
        Xtr += ptr; ytr += [class_idx] * len(ptr)
        Xva += pva; yva += [class_idx] * len(pva)
        Xte += pte; yte += [class_idx] * len(pte)
        print(f"  '{w}': {d['n_clips']} clips -> {len(ptr)+len(pva)+len(pte)} con aumento")

    btr = bg_feats(all_bg_tr, True, rng)
    bva = bg_feats(all_bg_va, False, rng)
    bte = bg_feats(all_bg_te, False, rng)
    str_, sva, ste = synth_silence(SYN_SILENCE_TR, rng), synth_silence(SYN_SILENCE_VA, rng), synth_silence(SYN_SILENCE_TE, rng)

    print("Cargando negativos del dataset publico de Google (otras palabras)...")
    Gtr, Gva, Gte = google_negative_split(words, rng)
    if len(Gtr) > MAX_GOOGLE_TRAIN_NEG:
        idx = rng.choice(len(Gtr), MAX_GOOGLE_TRAIN_NEG, replace=False)
        Gtr = Gtr[idx]

    Xtr += btr + str_ + list(Gtr); ytr += [0] * (len(btr) + len(str_) + len(Gtr))
    Xva += bva + sva + list(Gva); yva += [0] * (len(bva) + len(sva) + len(Gva))
    Xte += bte + ste + list(Gte); yte += [0] * (len(bte) + len(ste) + len(Gte))

    def finalize(X, y):
        X = np.stack(X).astype(np.float32)
        y = np.array(y, dtype=np.int32)
        idx = rng.permutation(len(X))
        return X[idx], y[idx]

    xtr, ytr = finalize(Xtr, ytr)
    xva, yva = finalize(Xva, yva)
    xte, yte = finalize(Xte, yte)

    print(f"\nDataset: entrenamiento={len(xtr)}  validacion={len(xva)}  prueba={len(xte)}")
    print(f"  clases: 'otro' + {', '.join(words)}")
    return xtr, ytr, xva, yva, xte, yte


# ================= entrenamiento =================

def train(xtr, ytr, xva, yva, sens_factor, num_classes):
    counts = np.bincount(ytr, minlength=num_classes)
    total = len(ytr)
    class_weight = {}
    for c in range(num_classes):
        w = total / (num_classes * max(int(counts[c]), 1))
        if c != 0:            # clase 0 es "otro"; las palabras se pesan segun la sensibilidad elegida
            w *= sens_factor
        class_weight[c] = w
    print("\nBalance de clases: " + ", ".join(f"{c}={int(counts[c])}(w={class_weight[c]:.2f})" for c in range(num_classes)))

    layers = [tf.keras.layers.Input(shape=(base.N_FEATURES,))]
    for h in HIDDEN:
        layers.append(tf.keras.layers.Dense(h, activation="relu"))
        layers.append(tf.keras.layers.Dropout(DROPOUT))
    layers.append(tf.keras.layers.Dense(num_classes))
    model = tf.keras.Sequential(layers)
    model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                  loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                  metrics=["accuracy"])
    cb = tf.keras.callbacks.EarlyStopping(monitor="val_accuracy", patience=15,
                                          restore_best_weights=True, verbose=1)
    model.fit(xtr, ytr, validation_data=(xva, yva), epochs=EPOCHS,
              batch_size=32, class_weight=class_weight, callbacks=[cb], verbose=2)
    return model


def export(words, model, xtr, xte, yte):
    import struct

    WORDS = ["otro"] + words
    n_classes = len(WORDS)

    print("\nCuantizando a int8 (per-tensor)...")
    tflite_model = base.quantize(model, xtr)
    layers, interp = base.extract_layers(tflite_model)
    npu = base.to_npu_format(layers)
    stream = base.build_stream_bytes(npu)

    in_det, out_det = interp.get_input_details()[0], interp.get_output_details()[0]
    s_in, zp_in = in_det["quantization"]

    def quant_feat(f):
        return np.clip(np.round(f / s_in + zp_in), -128, 127).astype(np.int8)

    conf = np.zeros((n_classes, n_classes), dtype=np.int64)
    preds = []
    for k in range(len(xte)):
        q = quant_feat(xte[k])
        interp.set_tensor(in_det["index"], q.reshape(1, -1))
        interp.invoke()
        p = int(np.argmax(interp.get_tensor(out_det["index"])[0]))
        preds.append(p)
        conf[yte[k], p] += 1

    acc = np.trace(conf) / max(conf.sum(), 1)
    print(f"\nPrecision int8 sobre el set de prueba: {acc*100:.2f}%")
    for c in range(1, n_classes):
        total_c = conf[c].sum()
        recall = conf[c, c] / total_c if total_c else 0.0
        print(f"  '{WORDS[c]}': te escucha el {recall*100:.1f}% de las veces que la dijiste")
    otro_total = conf[0].sum()
    far = (otro_total - conf[0, 0]) / otro_total if otro_total else 0.0
    print(f"  falsas activaciones (FAR, con silencio/otras palabras): {far*100:.2f}%")

    words_blob = ("\0".join(WORDS) + "\0").encode()
    mb = bytearray()
    mb += b"NPU1"
    mb += struct.pack("<HHH", len(npu), base.N_FEATURES, n_classes)
    mb += struct.pack("<fii", float(s_in), int(zp_in), base.QSHIFT)
    for L in npu:
        mb += struct.pack("<HHiiii", L["in_n"], L["out_n"], L["mult_int"],
                          L["zp_out"], L["act_min"], L["act_max"])
    all_bias = np.concatenate([L["bias"] for L in npu]).astype(np.int32)
    mb += struct.pack("<I", len(all_bias)) + all_bias.tobytes()
    mb += struct.pack("<I", len(stream)) + bytes(stream)
    mb += struct.pack("<HHHHHHH", base.N_MEL, base.N_MFCC, base.FFT_LENGTH // 2 + 1,
                      base.N_FRAMES, base.FRAME_LENGTH, base.FRAME_STEP, base.FFT_LENGTH)
    mb += base.MEL.astype(np.float32).tobytes()
    mb += base.DCT.astype(np.float32).tobytes()
    mb += struct.pack("<H", len(words_blob)) + words_blob

    n_clips = len(xte)
    tb = bytearray()
    tb += struct.pack("<HH", n_clips, base.N_FEATURES)
    for k in range(n_clips):
        tb += quant_feat(xte[k]).tobytes()
    tb += bytes(int(v) for v in yte[:n_clips])
    tb += bytes(int(v) for v in preds[:n_clips])

    out_dir = os.path.join(MODELS_DIR, "_".join(words))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "model.bin"), "wb") as f:
        f.write(mb)
    with open(os.path.join(out_dir, "test.bin"), "wb") as f:
        f.write(tb)
    print(f"\nEscritos en: {os.path.normpath(out_dir)}")
    print(f"  model.bin: {len(mb)} bytes   test.bin: {len(tb)} bytes")
    return out_dir


# ================= main =================

def discover_recorded_words():
    if not os.path.isdir(ROOT):
        return []
    words = []
    for name in sorted(os.listdir(ROOT)):
        if list_wavs(os.path.join(ROOT, name, "positive")):
            words.append(name)
    return words


def ask_words():
    existing = discover_recorded_words()
    words = []
    if existing:
        print(f"Ya tenes grabaciones de: {', '.join(existing)}")
        ans = input("¿Incluirlas en este modelo? (S/n): ").strip().lower()
        if ans != "n":
            words = list(existing)

    print("\nAgrega la(s) palabra(s) nueva(s) que quieras sumar al modelo.")
    print("Dejá vacío y apreta Enter cuando termines.")
    while True:
        w = input(f"Palabra nueva ({len(words)} hasta ahora, Enter para terminar): ").strip().lower()
        if not w:
            break
        if w in words:
            print("  ya esta en la lista")
            continue
        words.append(w)

    if not words:
        raise SystemExit("hace falta al menos una palabra")
    return words


def main():
    print("=== Entrenar un detector de palabras propias para la NPU ===\n")
    words = ask_words()
    print(f"\nEl modelo va a reconocer: {', '.join(words)}  (+ 'otro' para todo lo demas)")

    _, sens_factor = ask_choice_sensibilidad()

    xtr, ytr, xva, yva, xte, yte = build_dataset(words)
    model = train(xtr, ytr, xva, yva, sens_factor, num_classes=len(words) + 1)
    out_dir = export(words, model, xtr, xte, yte)

    print(f"\nPara subirlo al ESP32:")
    print(f"  1) copia {os.path.join(out_dir, 'model.bin')} y test.bin a")
    print(f"     {os.path.normpath(ESP32_DATA_DIR)}  (reemplaza el modelo que haya)")
    print(f"  2) pio run -t uploadfs --upload-port COM4")

    ans = input("\n¿Copiar esos archivos ahora a la carpeta del ESP32? (s/N): ").strip().lower()
    if ans == "s":
        import shutil
        os.makedirs(ESP32_DATA_DIR, exist_ok=True)
        shutil.copy(os.path.join(out_dir, "model.bin"), ESP32_DATA_DIR)
        shutil.copy(os.path.join(out_dir, "test.bin"), ESP32_DATA_DIR)
        print(f"Copiado a {os.path.normpath(ESP32_DATA_DIR)}")


if __name__ == "__main__":
    main()
