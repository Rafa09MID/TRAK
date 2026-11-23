import os
import io
import time
import base64
import requests
import cv2
import numpy as np
from PIL import Image
from dotenv import load_dotenv
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from google import genai
from google.genai import types

# Cargar variables de entorno desde .env
env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".env")
load_dotenv(dotenv_path=env_path)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
ESP32_URL = os.getenv("ESP32_URL", "http://192.168.68.107")

# Inicializar cliente oficial de Gemini
ai_client = None
if GEMINI_API_KEY:
    ai_client = genai.Client(api_key=GEMINI_API_KEY)

app = FastAPI(title="TRAK - Servidor de Calibración Visual y Gimbal")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Servir archivos estáticos del frontend
static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
os.makedirs(static_dir, exist_ok=True)
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Modelos Pydantic para peticiones
class ImagePayload(BaseModel):
    image_base64: str  # Data URL: "data:image/jpeg;base64,..."
    prompt: str = ""

class ServoTarget(BaseModel):
    cabeceo: int
    rotacion: int

class AlignRequest(BaseModel):
    image_base64: str
    target_x: float  # 0.0 a 1.0 (pantalla del celular)
    target_y: float  # 0.0 a 1.0 (pantalla del celular)
    cabeceo_actual: int
    rotacion_actual: int

def decodificar_imagen(base64_str: str) -> np.ndarray:
    if "," in base64_str:
        base64_str = base64_str.split(",")[1]
    img_bytes = base64.b64decode(base64_str)
    nparr = np.frombuffer(img_bytes, np.uint8)
    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)

def detectar_laser_opencv(img_bgr: np.ndarray):
    """
    Detector de alta precisión calibrado para cámara de celular:
    Detecta el núcleo blanco-rosado saturado y halo rojo del láser en cualquier condición de luz.
    Filtra reflejos grandes y prioriza puntos concentrados de alta intensidad.
    """
    h, w = img_bgr.shape[:2]
    
    # 1. Filtro en espacio HSV con saturación permisiva para sensores saturados
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 25, 150]), np.array([18, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([155, 25, 150]), np.array([180, 255, 255]))
    hsv_mask = cv2.bitwise_or(mask1, mask2)

    # 2. Filtro diferencial en canales BGR (el canal Rojo supera fuertemente al Verde y Azul)
    b, g, r = cv2.split(img_bgr)
    diff_mask = (r > g + 15) & (r > b + 10) & (r > 140)
    
    # Combinar ambas condiciones
    laser_mask = cv2.bitwise_and(hsv_mask, diff_mask.astype(np.uint8) * 255)

    contours, _ = cv2.findContours(laser_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filtrar solo contornos que tengan tamaño compatible con un láser (radio entre 1 y 40 px)
    laser_candidates = []
    for c in contours:
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        if 1.0 <= radius <= 45.0:
            area = cv2.contourArea(c)
            # Intensidad en el centro
            ix, iy = int(np.clip(cx, 0, w - 1)), int(np.clip(cy, 0, h - 1))
            val = int(r[iy, ix])
            laser_candidates.append({
                "cx": cx, "cy": cy, "radius": radius,
                "score": val * 2.0 - (int(g[iy, ix]) + int(b[iy, ix]))
            })

    if laser_candidates:
        # Elegir el candidato con mayor intensidad diferencial
        best = max(laser_candidates, key=lambda k: k["score"])
        return round(best["cx"] / w, 4), round(best["cy"] / h, 4), True, round(best["radius"], 1)

    # Fallback: Máximo pico diferencial de intensidad roja en la imagen completa
    score = r.astype(np.float32) * 2.0 - (g.astype(np.float32) + b.astype(np.float32))
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(score)
    if max_val > 70:
        return round(max_loc[0] / w, 4), round(max_loc[1] / h, 4), True, 4.0

    return 0.5, 0.5, False, 0.0

@app.get("/")
def home():
    index_path = os.path.join(static_dir, "index.html")
    if os.path.exists(index_path):
        with open(index_path, "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    return HTMLResponse("<h1>TRAK Backend Activo</h1><p>Frontend en /static/index.html</p>")

@app.post("/api/detect_laser")
def api_detect_laser(payload: ImagePayload):
    try:
        img = decodificar_imagen(payload.image_base64)
        x, y, detected, r = detectar_laser_opencv(img)
        return {"detected": detected, "x": x, "y": y, "radius": r}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@app.post("/api/gemini_analyze")
def api_gemini_analyze(payload: ImagePayload):
    if not ai_client:
        return {"error": "API Key de Gemini no configurada."}

    try:
        if "," in payload.image_base64:
            base64_data = payload.image_base64.split(",")[1]
        else:
            base64_data = payload.image_base64

        img_bytes = base64.b64decode(base64_data)

        prompt_text = payload.prompt if payload.prompt else (
            "Eres el sistema de visión artificial de TRAK. "
            "Observa esta imagen tomada desde la cámara del celular. "
            "1. Identifica dónde se encuentra el punto rojo del láser. "
            "2. Describe los objetos visibles o el plano de calibración. "
            "3. Evalúa si el punto láser está centrado o hacia qué dirección (arriba, abajo, izquierda, derecha) "
            "debe moverse el gimbal para alcanzar la zona deseada. Responde de forma concisa y técnica."
        )

        # Prioridad Gemini 3.8 Flash, con fallback a 3.7 y 3.5 en caso de alta demanda
        models_to_try = ["gemini-3.8-flash", "gemini-3.7-flash", "gemini-3.5-flash"]
        response_text = None
        selected_model = None

        for model_candidate in models_to_try:
            try:
                response = ai_client.models.generate_content(
                    model=model_candidate,
                    contents=[
                        types.Part.from_bytes(data=img_bytes, mime_type="image/jpeg"),
                        prompt_text
                    ]
                )
                if response and response.text:
                    response_text = response.text
                    selected_model = model_candidate
                    break
            except Exception as model_err:
                print(f"Aviso: {model_candidate} con pico de demanda, probando siguiente modelo...")
                continue

        if not response_text:
            return JSONResponse(status_code=500, content={"error": "Modelos de Gemini en alta demanda temporal. Reintenta en unos segundos."})

        return {"analysis": response_text, "model": selected_model}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@app.post("/api/esp32/servo")
def proxy_servo(target: ServoTarget):
    try:
        url = f"{ESP32_URL}/api/servo?cabeceo={target.cabeceo}&rotacion={target.rotacion}"
        res = requests.post(url, timeout=2)
        return res.json()
    except Exception as e:
        return {"error": f"No se pudo comunicar con ESP32: {str(e)}"}

@app.post("/api/esp32/laser")
def proxy_laser():
    try:
        url = f"{ESP32_URL}/api/laser/toggle"
        res = requests.post(url, timeout=2)
        return res.json()
    except Exception as e:
        return {"error": f"No se pudo comunicar con ESP32: {str(e)}"}

@app.get("/api/esp32/status")
def proxy_status():
    try:
        url = f"{ESP32_URL}/api/status"
        res = requests.get(url, timeout=2)
        return res.json()
    except Exception as e:
        return {"error": f"No se pudo comunicar con ESP32: {str(e)}"}

@app.post("/api/align_step")
def align_step(req: AlignRequest):
    """
    Paso de calibración asistida:
    Calcula el error (dx, dy) entre el láser actual y el objetivo tocado por el usuario,
    y ajusta los servos de cabeceo y rotación proporcionalmente.
    """
    img = decodificar_imagen(req.image_base64)
    lx, ly, detected, _ = detectar_laser_opencv(img)

    if not detected:
        return {
            "status": "laser_not_found",
            "message": "Láser no detectado en cámara.",
            "cabeceo": req.cabeceo_actual,
            "rotacion": req.rotacion_actual
        }

    # Error en coordenadas normalizadas [-1.0, 1.0]
    err_x = req.target_x - lx  # Horizontal (afecta rotación)
    err_y = req.target_y - ly  # Vertical (afecta cabeceo)

    # Ganancia proporcional (grados por delta de pantalla)
    KP_ROT = 28.0
    KP_CAB = 28.0

    delta_rot = int(err_x * KP_ROT)
    delta_cab = int(-err_y * KP_CAB) # Invertido según montaje de cámara

    nuevo_cabeceo = max(0, min(180, req.cabeceo_actual + delta_cab))
    nueva_rotacion = max(0, min(180, req.rotacion_actual + delta_rot))

    # Mover el gimbal
    try:
        url = f"{ESP32_URL}/api/servo?cabeceo={nuevo_cabeceo}&rotacion={nueva_rotacion}"
        requests.post(url, timeout=2)
    except:
        pass

    distancia = (err_x**2 + err_y**2)**0.5
    alineado = distancia < 0.025  # Menos del 2.5% de tolerancia

    return {
        "status": "aligned" if alineado else "adjusting",
        "laser_x": lx,
        "laser_y": ly,
        "target_x": req.target_x,
        "target_y": req.target_y,
        "distancia": round(distancia, 4),
        "nuevo_cabeceo": nuevo_cabeceo,
        "nueva_rotacion": nueva_rotacion,
        "alineado": alineado
    }

if __name__ == "__main__":
    import uvicorn
    cert_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cert.pem")
    key_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "key.pem")

    print(f"\n=========================================================")
    print(f"  SERVIDOR TRAK INICIADO EN HTTPS")
    print(f"  Abre en tu CELULAR: https://192.168.68.108:8443")
    print(f"=========================================================\n")

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8443,
        ssl_keyfile=key_path,
        ssl_certfile=cert_path
    )
