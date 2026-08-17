"""record_server.py -- Server local para grabar muestras de entrenamiento
DESDE EL CELULAR (o cualquier navegador en la misma red), sin necesitar
sounddevice ni acceso fisico a la PC.

Corre en la PC, sirve una pagina simple por HTTP, y guarda lo que grabues
directo en las carpetas que espera train_one_word_npu.py
(tools/my_words/<palabra>/positive/ o /background/) -- mismo formato (WAV,
16 kHz, mono, 1 segundo), mismo esquema de nombres, así después corres el
entrenamiento normal y ya las toma.

La captura de audio en el navegador (cuenta regresiva, pedido de 16kHz
nativo, centrado en la ventana de mas energia, normalizacion de volumen)
es la misma que ya se probo y afino en la demo de la NPU -- no la
reinventamos.

Uso:
  python record_server.py
  Abrir en el celular:  http://<ip-de-esta-pc>:5000   (misma red WiFi)
"""

import os
import socket
import struct
import sys

from flask import Flask, request, jsonify, Response

sys.path.insert(0, os.path.dirname(__file__))
import train_one_word_npu as rec   # SAMPLE_RATE, CLIP_SAMPLES, ROOT, list_wavs

app = Flask(__name__)

SAMPLE_RATE = rec.SAMPLE_RATE   # 16000
CLIP_SAMPLES = rec.CLIP_SAMPLES  # 16000 (1 segundo)


def word_dir(word, kind):
    d = os.path.join(rec.ROOT, word, kind)
    os.makedirs(d, exist_ok=True)
    return d


def next_index(folder, prefix):
    existing = rec.list_wavs(folder)
    nums = []
    for f in existing:
        try:
            nums.append(int(f[len(prefix):len(prefix) + 3]))
        except ValueError:
            pass
    return (max(nums) + 1) if nums else 1


def save_wav(path, pcm16_bytes):
    n = len(pcm16_bytes) // 2
    with open(path, "wb") as f:
        f.write(b"RIFF")
        f.write(struct.pack("<I", 36 + len(pcm16_bytes)))
        f.write(b"WAVEfmt ")
        f.write(struct.pack("<IHHIIHH", 16, 1, 1, SAMPLE_RATE, SAMPLE_RATE * 2, 2, 16))
        f.write(b"data")
        f.write(struct.pack("<I", len(pcm16_bytes)))
        f.write(pcm16_bytes)
    return n


@app.get("/")
def index():
    return Response(PAGE, mimetype="text/html")


@app.get("/api/words")
def api_words():
    words = rec.discover_recorded_words()
    counts = {}
    for w in words:
        counts[w] = dict(
            positive=len(rec.list_wavs(os.path.join(rec.ROOT, w, "positive"))),
            background=len(rec.list_wavs(os.path.join(rec.ROOT, w, "background"))))
    return jsonify(counts)


@app.post("/api/upload")
def api_upload():
    word = request.args.get("word", "").strip().lower()
    kind = request.args.get("kind", "positive")
    if not word or kind not in ("positive", "background"):
        return jsonify(error="parametros invalidos"), 400

    pcm = request.get_data()
    if len(pcm) != CLIP_SAMPLES * 2:
        return jsonify(error=f"tamano inesperado: {len(pcm)} bytes (se esperaban {CLIP_SAMPLES*2})"), 400

    prefix = "rec_" if kind == "positive" else "bg_"
    folder = word_dir(word, kind)
    idx = next_index(folder, prefix)
    path = os.path.join(folder, f"{prefix}{idx:03d}.wav")
    save_wav(path, pcm)
    return jsonify(ok=True, index=idx, path=path)


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


PAGE = """<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Grabar muestras</title>
<style>
 *{box-sizing:border-box}
 body{margin:0;padding:18px;background:#0e1116;color:#e6edf3;
      font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;text-align:center}
 h1{font-size:1.1rem;margin:0 0 16px}
 label{display:block;text-align:left;font-size:.8rem;color:#8b949e;margin:12px 0 4px}
 input,select{width:100%;padding:10px;border-radius:8px;border:1px solid #30363d;
              background:#161b22;color:#e6edf3;font-size:1rem}
 button{background:#1f6feb;color:#fff;border:0;border-radius:10px;
        padding:14px 28px;font-size:1rem;cursor:pointer;margin-top:16px;width:100%}
 button:disabled{background:#21262d;color:#8b949e}
 button.stop{background:#da3633}
 #status{margin:14px 0;color:#8b949e;font-size:.9rem;min-height:1.4rem;white-space:pre-line}
 #count{font-size:.8rem;color:#8b949e;margin-top:6px}
 .row{display:flex;gap:10px}
 .row>div{flex:1}
</style></head><body>
<h1>Grabar muestras de entrenamiento</h1>

<label>Palabra (o "fondo" para ruido ambiente)</label>
<input id="word" list="wordlist" placeholder="ej: hola">
<datalist id="wordlist"></datalist>

<div class="row">
  <div>
    <label>Tipo</label>
    <select id="kind">
      <option value="positive">Palabra</option>
      <option value="background">Fondo / silencio</option>
    </select>
  </div>
  <div>
    <label>Repeticiones</label>
    <input id="n" type="number" value="20" min="1" max="300">
  </div>
</div>

<div id="count"></div>
<button id="go">Empezar</button>
<div id="status"></div>

<script>
const $=id=>document.getElementById(id);
const SR=16000, NS=16000;
let stopFlag=false;

async function refreshCounts(){
  const r = await fetch('/api/words'); const words = await r.json();
  const dl = $('wordlist'); dl.innerHTML='';
  for(const w in words) dl.innerHTML += `<option value="${w}">`;
  const w = $('word').value.trim().toLowerCase();
  if(w && words[w]){
    $('count').textContent = `ya tenes: ${words[w].positive} palabra, ${words[w].background} fondo`;
  } else {
    $('count').textContent = w ? 'palabra nueva' : '';
  }
}
$('word').addEventListener('input', refreshCounts);
refreshCounts();

function sleep(ms){ return new Promise(r=>setTimeout(r,ms)); }

async function recordOneClip(){
  const stream=await navigator.mediaDevices.getUserMedia({audio:{channelCount:1,
    sampleRate:{ideal:16000},
    echoCancellation:false,noiseSuppression:false,autoGainControl:false,
    googEchoCancellation:false,googAutoGainControl:false,googNoiseSuppression:false,
    googHighpassFilter:false,googTypingNoiseDetection:false,googAudioMirroring:false}});
  const ac=new (window.AudioContext||window.webkitAudioContext)({sampleRate:16000});
  const src=ac.createMediaStreamSource(stream);
  const proc=ac.createScriptProcessor(4096,1,1);
  let chunks=[],total=0,recording=false;
  const CAPTURE_S=1.6;
  const need=Math.ceil(ac.sampleRate*CAPTURE_S);
  proc.onaudioprocess=e=>{
    if(!recording||total>=need)return;
    const d=e.inputBuffer.getChannelData(0);
    chunks.push(new Float32Array(d)); total+=d.length;
  };
  src.connect(proc); proc.connect(ac.destination);

  for(const c of ['3','2','1','¡YA!']){
    $('status').textContent=c;
    await sleep(500);
  }
  $('status').textContent='🎤 grabando...';
  recording=true;
  await new Promise(r=>{ const chk=()=>{ if(total>=need) r(); else setTimeout(chk,20); }; chk(); });
  proc.disconnect(); src.disconnect();
  stream.getTracks().forEach(t=>t.stop());

  const all=new Float32Array(total); let o=0;
  for(const c of chunks){all.set(c,o);o+=c.length;}
  const ratio=ac.sampleRate/SR;
  const nOut=Math.floor(total/ratio);
  const resampled=new Float32Array(nOut);
  for(let i=0;i<nOut;i++){
    const p=i*ratio, i0=Math.floor(p), fr=p-i0;
    const a=all[i0]||0, b=all[i0+1]||0;
    resampled[i]=a+(b-a)*fr;
  }
  let start=0;
  if(nOut>NS){
    const cum=new Float64Array(nOut+1);
    for(let i=0;i<nOut;i++) cum[i+1]=cum[i]+resampled[i]*resampled[i];
    let best=-1;
    for(let s=0;s<=nOut-NS;s++){
      const en=cum[s+NS]-cum[s];
      if(en>best){best=en; start=s;}
    }
  }
  let peak=0;
  for(let i=0;i<NS;i++){ const v=Math.abs(resampled[start+i]||0); if(v>peak) peak=v; }
  const gain=peak>1e-6 ? Math.min(0.65/peak,20) : 1;
  const pcm=new Int16Array(NS);
  for(let i=0;i<NS;i++){
    let v=(resampled[start+i]||0)*gain*32767;
    pcm[i]=Math.max(-32768,Math.min(32767,v|0));
  }
  ac.close();
  return {pcm, peak};
}

async function recordBackground(){
  // para "fondo" no hace falta centrar en energia -- graba 1s de silencio/ruido tal cual
  const stream=await navigator.mediaDevices.getUserMedia({audio:{channelCount:1,
    sampleRate:{ideal:16000},
    echoCancellation:false,noiseSuppression:false,autoGainControl:false,
    googEchoCancellation:false,googAutoGainControl:false,googNoiseSuppression:false,
    googHighpassFilter:false,googTypingNoiseDetection:false,googAudioMirroring:false}});
  const ac=new (window.AudioContext||window.webkitAudioContext)({sampleRate:16000});
  const src=ac.createMediaStreamSource(stream);
  const proc=ac.createScriptProcessor(4096,1,1);
  let chunks=[],total=0;
  const need=Math.ceil(ac.sampleRate*1.05);
  proc.onaudioprocess=e=>{
    if(total>=need)return;
    const d=e.inputBuffer.getChannelData(0);
    chunks.push(new Float32Array(d)); total+=d.length;
  };
  src.connect(proc); proc.connect(ac.destination);
  await new Promise(r=>{ const chk=()=>{ if(total>=need) r(); else setTimeout(chk,20); }; chk(); });
  proc.disconnect(); src.disconnect();
  stream.getTracks().forEach(t=>t.stop());
  const all=new Float32Array(total); let o=0;
  for(const c of chunks){all.set(c,o);o+=c.length;}
  const ratio=ac.sampleRate/SR;
  const pcmF=new Float32Array(NS);
  for(let i=0;i<NS;i++){
    const p=i*ratio, i0=Math.floor(p), fr=p-i0;
    const a=all[i0]||0, b=all[i0+1]||0;
    pcmF[i]=a+(b-a)*fr;
  }
  const pcm=new Int16Array(NS);
  for(let i=0;i<NS;i++) pcm[i]=Math.max(-32768,Math.min(32767,(pcmF[i]*32767)|0));
  ac.close();
  return {pcm, peak:0};
}

async function upload(word, kind, pcm){
  const r = await fetch(`/api/upload?word=${encodeURIComponent(word)}&kind=${kind}`, {
    method:'POST', body: pcm.buffer});
  return r.json();
}

$('go').addEventListener('click', async ()=>{
  const word = $('word').value.trim().toLowerCase();
  const kind = $('kind').value;
  const n = parseInt($('n').value) || 1;
  if(!word){ $('status').textContent='falta la palabra'; return; }

  const btn = $('go');
  if(btn.textContent==='Parar'){ stopFlag=true; return; }
  stopFlag=false;
  btn.textContent='Parar'; btn.classList.add('stop');

  try{
    for(let i=1;i<=n && !stopFlag;i++){
      $('status').textContent = kind==='positive'
        ? `[${i}/${n}] Decí "${word}"...`
        : `[${i}/${n}] Silencio / ruido ambiente...`;
      await sleep(300);
      const {pcm, peak} = kind==='positive' ? await recordOneClip() : await recordBackground();
      const res = await upload(word, kind, pcm);
      if(res.error){ $('status').textContent = 'ERROR: '+res.error; break; }
      const tag = kind==='positive' ? (peak>0.05?'bien':'flojo, capaz repetir') : 'ok';
      $('status').textContent = `[${i}/${n}] guardado #${res.index}  pico=${peak.toFixed(3)} (${tag})`;
      await sleep(400);
    }
    $('status').textContent += stopFlag ? '\\n\\nParado.' : '\\n\\n¡Listo!';
  }catch(e){
    $('status').textContent = 'error: '+e.message;
  }
  btn.textContent='Empezar'; btn.classList.remove('stop');
  refreshCounts();
});
</script></body></html>
"""


if __name__ == "__main__":
    ip = local_ip()
    print(f"\n=== Servidor de grabacion ===")
    print(f"Abri en el celular (misma red WiFi):  http://{ip}:5000")
    print(f"Guarda en: {os.path.normpath(rec.ROOT)}")
    app.run(host="0.0.0.0", port=5000)
