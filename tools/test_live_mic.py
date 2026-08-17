"""test_live_mic.py -- Prueba el modelo YA ENTRENADO (el mismo model.bin
que correria en la NPU/ESP32) contra audio grabado en vivo por microfono,
pero calculando el MFCC con Python/TensorFlow (el mismo que uso el
entrenamiento) en vez del MFCC en C del ESP32.

Sirve para aislar la causa de "siempre da otro": si ACA reconoce bien las
palabras, el problema esta especificamente en el camino navegador -> ESP32
-> MFCC-en-C (o en la captura/remuestreo de audio del navegador), no en el
modelo entrenado ni en los pesos cuantizados. Si ACA tambien falla, el
problema esta en el modelo/los datos de entrenamiento.

La inferencia int8 replica exactamente sw_infer() de kws_npu_poc.cpp
(mismos pesos, mismo bias, mismo shift), leyendo directo el model.bin.

Uso:  python test_live_mic.py [carpeta_o_archivo_model.bin]
"""

import os
import sys
import struct

import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
sys.path.insert(0, os.path.dirname(__file__))
import train_kws_npu as base            # features_from_audio (MFCC de Python)
import train_one_word_npu as rec        # record_raw / center_on_energy / MODELS_DIR

NLANES = 8


def parse_model_bin(path):
    with open(path, "rb") as f:
        data = f.read()
    off = 0

    def rd(fmt):
        nonlocal off
        v = struct.unpack_from(fmt, data, off)
        off += struct.calcsize(fmt)
        return v

    magic = data[off:off + 4]; off += 4
    if magic != b"NPU1":
        raise SystemExit(f"{path} no tiene el formato NPU1 esperado")

    num_layers, n_features, num_outputs = rd("<HHH")
    input_scale, input_zp, qshift = rd("<fii")

    layers = []
    for _ in range(num_layers):
        in_n, out_n, mult, zp_out, act_min, act_max = rd("<HHiiii")
        layers.append(dict(in_n=in_n, out_n=out_n, mult=mult, zp_out=zp_out,
                           act_min=act_min, act_max=act_max))

    (n_bias,) = rd("<I")
    bias = np.frombuffer(data, dtype="<i4", count=n_bias, offset=off).copy(); off += n_bias * 4
    (n_weights,) = rd("<I")
    weights = np.frombuffer(data, dtype=np.int8, count=n_weights, offset=off).copy(); off += n_weights
    n_mel, n_mfcc, n_bins, n_frames, frame_length, frame_step, fft_length = rd("<HHHHHHH")
    mel = np.frombuffer(data, dtype="<f4", count=n_bins * n_mel, offset=off).reshape(n_bins, n_mel).copy()
    off += n_bins * n_mel * 4
    dct = np.frombuffer(data, dtype="<f4", count=n_mel * n_mfcc, offset=off).reshape(n_mel, n_mfcc).copy()
    off += n_mel * n_mfcc * 4
    (wlen,) = rd("<H")
    words_blob = data[off:off + wlen]; off += wlen
    words = [w.decode() for w in words_blob.split(b"\0") if w]

    # Reconstruir la matriz de pesos por capa a partir del stream lineal
    # (mismo layout que usa la NPU / w_at() en el ESP32: ver build_stream_bytes).
    bias_base = 0
    stream_base = 0
    for L in layers:
        waves = (L["out_n"] + NLANES - 1) // NLANES
        n = waves * L["in_n"] * NLANES
        chunk = weights[stream_base:stream_base + n]
        W = chunk.reshape(waves, L["in_n"], NLANES).transpose(0, 2, 1).reshape(waves * NLANES, L["in_n"])
        L["W"] = W[:L["out_n"]].astype(np.int64)
        L["bias"] = bias[bias_base:bias_base + L["out_n"]].astype(np.int64)
        bias_base += L["out_n"]
        stream_base += n

    return dict(num_layers=num_layers, n_features=n_features, num_outputs=num_outputs,
               input_scale=input_scale, input_zp=input_zp, qshift=qshift, layers=layers,
               n_mel=n_mel, n_mfcc=n_mfcc, words=words)


def infer_int8(model, feat_i8):
    """Replica EXACTA de sw_infer() en kws_npu_poc.cpp."""
    x = feat_i8.astype(np.int64)
    for L in model["layers"]:
        acc = L["W"] @ x + L["bias"]
        scaled = (acc * np.int64(L["mult"])) >> model["qshift"]
        scaled = scaled + L["zp_out"]
        x = np.clip(scaled, L["act_min"], L["act_max"])
    return x


def quantize_feat(model, feat):
    q = np.round(feat / model["input_scale"] + model["input_zp"])
    return np.clip(q, -128, 127).astype(np.int8)


def find_model_path(arg):
    if arg:
        return os.path.join(arg, "model.bin") if os.path.isdir(arg) else arg
    root = rec.MODELS_DIR
    dirs = sorted(os.listdir(root)) if os.path.isdir(root) else []
    if not dirs:
        raise SystemExit(f"no hay modelos en {root} -- corre primero train_one_word_npu.py")
    if len(dirs) == 1:
        return os.path.join(root, dirs[0], "model.bin")
    print("Modelos disponibles:")
    for i, d in enumerate(dirs):
        print(f"  {i}: {d}")
    idx = int(input("Elegi uno: ").strip())
    return os.path.join(root, dirs[idx], "model.bin")


def main():
    path = find_model_path(sys.argv[1] if len(sys.argv) > 1 else None)
    print(f"Cargando {path}")
    model = parse_model_bin(path)
    print(f"Palabras: {model['words']}")
    print(f"input_scale={model['input_scale']:.5f}  input_zp={model['input_zp']}  qshift={model['qshift']}")
    print("\nApreta Enter, esperá la cuenta regresiva y decí una palabra. Ctrl+C para salir.")

    while True:
        input("\n[Enter para grabar] ")
        for c in ("3", "2", "1", "YA!"):
            print(f"  {c}  ", end="\r", flush=True)
            import time; time.sleep(0.5)
        print("      ", end="\r")
        audio = rec.record_raw(1.6)
        clip, peak = rec.center_on_energy(audio)

        feat = base.features_from_audio(clip)
        qf = quantize_feat(model, feat)
        print(f"  pico={peak:.3f}   features: min={qf.min()} max={qf.max()} media={qf.mean():.1f}")

        logits = infer_int8(model, qf)
        order = np.argsort(logits)[::-1]
        print("  resultado:")
        for i in order[:5]:
            marker = " <--" if i == order[0] else ""
            print(f"    {model['words'][i]:14s} {int(logits[i]):5d}{marker}")


if __name__ == "__main__":
    main()
