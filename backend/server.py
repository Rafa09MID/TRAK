import os
import io
import json
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
ESP32_URL = os.getenv("ESP32_URL", "http://192.168.68.103")

def auto_detect_esp32():
    """Busca automaticamente el ESP32 en la red actual (Casa, mDNS o AP Escuela 192.168.4.1)."""
    global ESP32_URL
    candidates = [ESP32_URL, "http://esp32-laser.local", "http://192.168.4.1"]
    seen = set()
    for url in candidates:
        if not url or url in seen:
            continue
        seen.add(url)
        try:
            r = requests.get(f"{url}/api/status", timeout=1.2)
            if r.status_code == 200:
                ESP32_URL = url
                return url
        except Exception:
            continue
    return ESP32_URL

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

class NormTarget(BaseModel):
    u: float
    v: float

class SafeAreaCorners(BaseModel):
    cTL: int
    rTL: int
    cTR: int
    rTR: int
    cBR: int
    rBR: int
    cBL: int
    rBL: int

class HomographyPayload(BaseModel):
    cam_corners: list  # [[x1,y1], [x2,y2], [x3,y3], [x4,y4]] normalizado 0..1
    servo_corners: SafeAreaCorners

class PointCameraRequest(BaseModel):
    x: float
    y: float

class CommandRequest(BaseModel):
    image_base64: str
    command: str

class AlignRequest(BaseModel):
    image_base64: str
    target_x: float  # 0.0 a 1.0 (pantalla del celular)
    target_y: float  # 0.0 a 1.0 (pantalla del celular)
    cabeceo_actual: int
    rotacion_actual: int
    dx_dcab: float = 0.0
    dx_drot: float = 0.0
    dy_dcab: float = 0.0
    dy_drot: float = 0.0
    gain: float = 0.6
    max_step: int = 9

class NotifyPayload(BaseModel):
    text: str = ""
    beep: str = "none"

def _notificar_esp32(text: str = "", beep: str = "none"):
    try:
        requests.post(f"{ESP32_URL}/api/notify", params={"text": text, "beep": beep}, timeout=1.5)
    except Exception:
        pass

def decodificar_imagen(base64_str: str) -> np.ndarray:
    if "," in base64_str:
        base64_str = base64_str.split(",")[1]
    img_bytes = base64.b64decode(base64_str)
    nparr = np.frombuffer(img_bytes, np.uint8)
    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)

def llamar_gemini_rapido(img_bytes: bytes, prompt_text: str, json_mode: bool = False):
    """
    Llama a la API REST de Gemini con timeout corto y sin reintentos bloqueantes,
    priorizando gemini-3.8-flash y cayendo en <0.5s al siguiente modelo disponible si hay 503.
    """
    models_to_try = [
        ("gemini-3.8-flash", 4),
        ("gemini-3.5-flash-lite", 10),
        ("gemini-3.1-flash-lite", 10),
        ("gemini-3.6-flash", 12),
    ]
    b64_img = base64.b64encode(img_bytes).decode("utf-8")
    body = {
        "contents": [{
            "parts": [
                {"inline_data": {"mime_type": "image/jpeg", "data": b64_img}},
                {"text": prompt_text}
            ]
        }]
    }
    if json_mode:
        body["generationConfig"] = {"responseMimeType": "application/json"}

    last_err = ""
    for m, t_sec in models_to_try:
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{m}:generateContent?key={GEMINI_API_KEY}"
        try:
            res = requests.post(url, json=body, timeout=t_sec)
            if res.status_code == 200:
                data = res.json()
                candidates = data.get("candidates", [])
                if candidates:
                    parts = candidates[0].get("content", {}).get("parts", [])
                    if parts and "text" in parts[0]:
                        return parts[0]["text"], m
            last_err = f"{m}: HTTP {res.status_code}"
            print(f"Fallback rapido ({last_err})")
        except Exception as ex:
            last_err = f"{m}: {ex}"
            print(f"Fallback rapido ({last_err})")
    return None, last_err

def detectar_laser_opencv(img_bgr: np.ndarray):
    """
    Detector anti-reflejos basado en Top-Hat morfológico:
    Aísla únicamente picos puntuales de alta intensidad roja/blanca-rosada
    e ignora reflejos amplios, cajas de cartón o iluminación cálida.
    """
    h, w = img_bgr.shape[:2]
    b, g, r = cv2.split(img_bgr)
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]

    # 1. Top-Hat morfológico para aislar puntos láser y suprimir reflejos/fondos amplios
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    tophat_r = cv2.morphologyEx(r, cv2.MORPH_TOPHAT, kernel)

    # 2. Regla espectral: pico puntual en Rojo + brillo alto + dominancia de rojo sobre G y B
    r16 = r.astype(np.int16)
    red_dom = (r16 - g.astype(np.int16) > 15) & (r16 - b.astype(np.int16) > 10)
    laser_mask = ((tophat_r > 50) & (v > 180) & red_dom).astype(np.uint8) * 255

    contours, _ = cv2.findContours(laser_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    laser_candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 4.0:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        if 1.5 <= radius <= 28.0:
            ix, iy = int(np.clip(cx, 0, w - 1)), int(np.clip(cy, 0, h - 1))
            peak_score = float(tophat_r[iy, ix]) + float(v[iy, ix]) * 0.5 + float(r16[iy, ix] - g[iy, ix])
            laser_candidates.append({
                "cx": cx,
                "cy": cy,
                "radius": radius,
                "score": peak_score
            })

    if laser_candidates:
        best = max(laser_candidates, key=lambda k: k["score"])
        return round(best["cx"] / w, 4), round(best["cy"] / h, 4), True, round(best["radius"], 1)

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

        response_text, selected_model = llamar_gemini_rapido(img_bytes, prompt_text, json_mode=False)

        if not response_text:
            return JSONResponse(status_code=500, content={"error": f"Modelos de Gemini en alta demanda ({selected_model}). Reintenta."})

        return {"analysis": response_text, "model": selected_model}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@app.post("/api/identify_and_point")
def api_identify_and_point(payload: ImagePayload):
    """
    Identifica el objeto principal visible en el rango de visión usando Gemini 3.8 Flash
    y devuelve su nombre, descripción y coordenadas normalizadas (0..1) para apuntar el láser.
    """
    if not ai_client:
        return JSONResponse(status_code=500, content={"error": "API Key de Gemini no configurada."})

    try:
        base64_data = payload.image_base64.split(",")[1] if "," in payload.image_base64 else payload.image_base64
        img_bytes = base64.b64decode(base64_data)

        prompt_text = (
            "Analiza esta imagen tomada por el sistema láser TRAK. "
            "Identifica el objeto físico principal más relevante dentro de la escena (ignora el punto rojo del láser mismo). "
            "Devuelve ÚNICAMENTE un JSON válido con esta estructura exacta (coordenadas box_2d en escala 0 a 1000 [ymin, xmin, ymax, xmax]):\n"
            "{\n"
            '  "objeto": "Nombre corto del objeto en español",\n'
            '  "descripcion": "Qué es y dónde se encuentra brevemente (1 frase)",\n'
            '  "box_2d": [ymin, xmin, ymax, xmax],\n'
            '  "otros_objetos": [{"nombre": "...", "box_2d": [ymin, xmin, ymax, xmax]}]\n'
            "}"
        )

        raw_text, selected_model = llamar_gemini_rapido(img_bytes, prompt_text, json_mode=True)
        if not raw_text:
            return JSONResponse(status_code=500, content={"error": f"No se pudo identificar objetos ({selected_model})."})

        raw = raw_text.strip()
        if raw.startswith("```"):
            raw = raw.split("\n", 1)[1].rsplit("```", 1)[0].strip()
        parsed = json.loads(raw)

        box = parsed.get("box_2d", [400, 400, 600, 600])
        if len(box) != 4:
            box = [400, 400, 600, 600]

        ymin, xmin, ymax, xmax = [float(v) / 1000.0 for v in box]
        target_x = round((xmin + xmax) / 2.0, 4)
        target_y = round((ymin + ymax) / 2.0, 4)

        otros = []
        for obj in parsed.get("otros_objetos", [])[:4]:
            b = obj.get("box_2d", [])
            if len(b) == 4:
                oy1, ox1, oy2, ox2 = [float(v) / 1000.0 for v in b]
                otros.append({
                    "nombre": obj.get("nombre", "Objeto"),
                    "x": round((ox1 + ox2) / 2.0, 4),
                    "y": round((oy1 + oy2) / 2.0, 4),
                    "box": [oy1, ox1, oy2, ox2]
                })

        objeto_nombre = parsed.get("objeto", "Objeto detectado")
        _notificar_esp32(objeto_nombre, "detect")

        return {
            "objeto": objeto_nombre,
            "descripcion": parsed.get("descripcion", ""),
            "target_x": target_x,
            "target_y": target_y,
            "box": [ymin, xmin, ymax, xmax],
            "otros": otros,
            "model": selected_model
        }
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

@app.post("/api/esp32/move_norm")
def proxy_move_norm(target: NormTarget):
    try:
        url = f"{ESP32_URL}/api/move_norm?u={target.u}&v={target.v}"
        res = requests.post(url, timeout=2)
        return res.json()
    except Exception as e:
        return {"error": f"No se pudo comunicar con ESP32: {str(e)}"}

@app.post("/api/esp32/center")
def proxy_center():
    try:
        url = f"{ESP32_URL}/api/center"
        res = requests.post(url, timeout=2)
        return res.json()
    except Exception as e:
        return {"error": f"No se pudo comunicar con ESP32: {str(e)}"}

@app.post("/api/esp32/rect")
def proxy_rect():
    try:
        url = f"{ESP32_URL}/api/rect/toggle"
        res = requests.post(url, timeout=2)
        return res.json()
    except Exception as e:
        return {"error": f"No se pudo comunicar con ESP32: {str(e)}"}

@app.post("/api/esp32/notify")
def proxy_notify(payload: NotifyPayload):
    _notificar_esp32(payload.text, payload.beep)
    return {"sent": True, "text": payload.text, "beep": payload.beep}

@app.post("/api/save_safe_area")
def save_safe_area(corners: SafeAreaCorners):
    import json
    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "safe_area.json")
    data = corners.model_dump()
    with open(cal_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    try:
        url = (
            f"{ESP32_URL}/api/safe_area?"
            f"cTL={corners.cTL}&rTL={corners.rTL}&"
            f"cTR={corners.cTR}&rTR={corners.rTR}&"
            f"cBR={corners.cBR}&rBR={corners.rBR}&"
            f"cBL={corners.cBL}&rBL={corners.rBL}"
        )
        res = requests.post(url, timeout=3)
        return {"saved": True, "esp32": res.json(), "corners": data}
    except Exception as e:
        return {"saved": True, "esp32_error": str(e), "corners": data}

@app.get("/api/safe_area")
def get_safe_area():
    import json
    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "safe_area.json")
    if os.path.exists(cal_path):
        with open(cal_path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {"cTL": 150, "rTL": 68, "cTR": 150, "rTR": 180, "cBR": 81, "rBR": 180, "cBL": 81, "rBL": 68}

@app.post("/api/align_step")
def align_step(req: AlignRequest):
    """
    Paso de calibración asistida con Jacobiano 2x2:
    Calcula el error (dx, dy) entre el láser actual y el objetivo tocado por el usuario,
    e invierte el Jacobiano medido para mover los servos exactamente al punto objetivo.
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
    err_x = req.target_x - lx
    err_y = req.target_y - ly

    distancia = (err_x**2 + err_y**2)**0.5
    alineado = distancia < 0.025  # Menos del 2.5% de tolerancia

    if alineado:
        return {
            "status": "aligned",
            "laser_x": lx,
            "laser_y": ly,
            "target_x": req.target_x,
            "target_y": req.target_y,
            "distancia": round(distancia, 4),
            "nuevo_cabeceo": req.cabeceo_actual,
            "nueva_rotacion": req.rotacion_actual,
            "alineado": True
        }

    det = req.dx_dcab * req.dy_drot - req.dx_drot * req.dy_dcab
    max_s = float(max(2, min(15, req.max_step)))
    g = float(max(0.2, min(0.85, req.gain)))

    if abs(det) > 1e-5:
        # Usar matriz inversa del Jacobiano medido con paso suave limitado
        raw_dcab = (req.dy_drot * err_x - req.dx_drot * err_y) / det
        raw_drot = (-req.dy_dcab * err_x + req.dx_dcab * err_y) / det
        delta_cab = int(round(max(-max_s, min(max_s, raw_dcab * g))))
        delta_rot = int(round(max(-max_s, min(max_s, raw_drot * g))))
    else:
        # Fallback proporcional suave si no hay Jacobiano
        KP_ROT = 20.0
        KP_CAB = 20.0
        delta_rot = int(round(max(-max_s, min(max_s, err_x * KP_ROT * g))))
        delta_cab = int(round(max(-max_s, min(max_s, -err_y * KP_CAB * g))))

    # Asegurar al menos 1 grado de movimiento si no está alineado y el cálculo dio 0
    if delta_cab == 0 and abs(err_y) > 0.015:
        delta_cab = 1 if (abs(det) > 1e-6 and raw_dcab > 0) or (abs(det) <= 1e-6 and err_y < 0) else -1
    if delta_rot == 0 and abs(err_x) > 0.015:
        delta_rot = 1 if (abs(det) > 1e-6 and raw_drot > 0) or (abs(det) <= 1e-6 and err_x > 0) else -1

    nuevo_cabeceo = max(0, min(180, req.cabeceo_actual + delta_cab))
    nueva_rotacion = max(0, min(180, req.rotacion_actual + delta_rot))

    # Mover el gimbal
    try:
        url = f"{ESP32_URL}/api/servo?cabeceo={nuevo_cabeceo}&rotacion={nueva_rotacion}"
        requests.post(url, timeout=2)
    except:
        pass

    return {
        "status": "adjusting",
        "laser_x": lx,
        "laser_y": ly,
        "target_x": req.target_x,
        "target_y": req.target_y,
        "distancia": round(distancia, 4),
        "nuevo_cabeceo": nuevo_cabeceo,
        "nueva_rotacion": nueva_rotacion,
        "alineado": False
    }

# =====================================================================
# VISIÓN LOCAL OPENCV + CIELAB (BOLITAS DE PAPEL Y COLORES) + HOMOGRAFÍA
# =====================================================================

def _cal_file_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "safe_area.json")

def _cargar_calibracion():
    p = _cal_file_path()
    if os.path.exists(p):
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}

def clasificar_color_lab(L, A, B, H, S, V):
    """
    Clasificación de color en espacio CIELAB + HSV robusta ante cambios de iluminación.
    Distingue tonos blancos, verde pastel, verde intenso, amarillo, azul, rosa/rojo.
    """
    chroma = ((A - 128.0)**2 + (B - 128.0)**2)**0.5

    # Blanco: alta luminancia y baja cromaticidad
    if L > 145 and chroma < 18 and S < 65:
        return "Blanco"

    # Verde pastel: componente verde en LAB (A < 122) o Hue verde con saturación suave y brillo alto
    if (A < 122 and L > 110 and S < 155) or (34 <= H <= 92 and 25 <= S <= 145 and V > 115):
        return "Verde pastel"

    # Verde intenso
    if A < 115 or (35 <= H <= 90 and S > 145):
        return "Verde"

    # Amarillo
    if B > 142 and 112 <= A <= 142:
        return "Amarillo"

    # Azul
    if B < 114 and A < 140:
        return "Azul"

    # Rosa / Rojo
    if A > 140:
        return "Rosa" if L > 135 and S < 160 else "Rojo"

    if L > 125 and chroma < 15:
        return "Blanco"

    return "Otro"

def detectar_objetos_locales(img_bgr: np.ndarray, cam_corners=None):
    """
    Detecta objetos / bolitas de papel en <15ms usando OpenCV + CIELAB,
    excluye el punto láser y clasifica el color de cada objeto.
    """
    h, w = img_bgr.shape[:2]
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    l_ch, a_ch, b_ch = cv2.split(lab)

    # Máscara de objetos claros o coloridos sobre el fondo oscuro/neutro
    blur_l = cv2.GaussianBlur(l_ch, (5, 5), 0)
    _, thresh_l = cv2.threshold(blur_l, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # También incluir objetos coloridos por cromaticidad LAB
    chroma = np.sqrt((a_ch.astype(np.float32) - 128)**2 + (b_ch.astype(np.float32) - 128)**2)
    mask_chroma = (chroma > 16).astype(np.uint8) * 255
    mask = cv2.bitwise_or(thresh_l, mask_chroma)

    # Si tenemos polígono del Área Segura en cámara, restringir búsqueda dentro de él
    if cam_corners and len(cam_corners) == 4:
        poly_mask = np.zeros((h, w), dtype=np.uint8)
        pts = np.array([[int(p[0]*w), int(p[1]*h)] for p in cam_corners], dtype=np.int32)
        cv2.fillPoly(poly_mask, [pts], 255)
        mask = cv2.bitwise_and(mask, poly_mask)

    # Limpiar ruido morfológico
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Excluir la ubicación actual del punto láser para no confundirlo con una bolita
    lx, ly, laser_ok, lr = detectar_laser_opencv(img_bgr)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    objetos = []
    idx = 1

    for c in contours:
        area = cv2.contourArea(c)
        if area < 120 or area > (h * w * 0.45):
            continue

        x, y, bw, bh = cv2.boundingRect(c)
        cx = (x + bw / 2.0) / w
        cy = (y + bh / 2.0) / h

        # Ignorar si coincide exactamente con el punto láser y es muy pequeño
        if laser_ok and ((cx - lx)**2 + (cy - ly)**2)**0.5 < 0.03 and area < 600:
            continue

        obj_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(obj_mask, [c], -1, 255, -1)

        L_med = float(np.median(l_ch[obj_mask == 255]))
        A_med = float(np.median(a_ch[obj_mask == 255]))
        B_med = float(np.median(b_ch[obj_mask == 255]))
        H_med = float(np.median(hsv[:, :, 0][obj_mask == 255]))
        S_med = float(np.median(hsv[:, :, 1][obj_mask == 255]))
        V_med = float(np.median(hsv[:, :, 2][obj_mask == 255]))

        color_nombre = clasificar_color_lab(L_med, A_med, B_med, H_med, S_med, V_med)
        (_, _), radius = cv2.minEnclosingCircle(c)

        objetos.append({
            "id": idx,
            "color": color_nombre,
            "nombre": f"Objeto {color_nombre}",
            "x": round(cx, 4),
            "y": round(cy, 4),
            "box": [round(y/h, 4), round(x/w, 4), round((y+bh)/h, 4), round((x+bw)/w, 4)],
            "radius": round(float(radius), 1),
            "area": int(area)
        })
        idx += 1

    # Ordenar por área descendente
    objetos.sort(key=lambda o: o["area"], reverse=True)
    return objetos

@app.post("/api/calibrate_homography")
def calibrate_homography(payload: HomographyPayload):
    """
    Calcula y guarda la matriz Homografía 3x3 entre Cámara (x,y) y Área Segura (u,v)
    junto con los ángulos calibrados del ESP32.
    """
    if len(payload.cam_corners) != 4:
        return JSONResponse(status_code=400, content={"error": "Se requieren 4 esquinas de cámara."})

    src = np.array(payload.cam_corners, dtype=np.float32)
    dst = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)

    data = payload.servo_corners.model_dump()
    data["cam_corners"] = payload.cam_corners
    data["homography"] = H.tolist() if H is not None else None

    with open(_cal_file_path(), "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    try:
        url = (
            f"{ESP32_URL}/api/safe_area?"
            f"cTL={data['cTL']}&rTL={data['rTL']}&"
            f"cTR={data['cTR']}&rTR={data['rTR']}&"
            f"cBR={data['cBR']}&rBR={data['rBR']}&"
            f"cBL={data['cBL']}&rBL={data['rBL']}"
        )
        requests.post(url, timeout=3)
    except:
        pass

    return {"saved": True, "homography_ready": H is not None}

@app.post("/api/verify_calibration")
def verify_calibration(payload: ImagePayload):
    """
    Verifica si el láser físico coincide con el centro esperado (0.5, 0.5)
    usando la Homografía calibrada.
    """
    cal = _cargar_calibracion()
    H_list = cal.get("homography")
    if not H_list:
        return {"verified": False, "error": "No hay homografía guardada."}

    img = decodificar_imagen(payload.image_base64)
    lx, ly, detected, _ = detectar_laser_opencv(img)
    if not detected:
        return {"verified": False, "error": "Láser no visible durante verificación."}

    H = np.array(H_list, dtype=np.float32)
    pt = np.array([[[lx, ly]]], dtype=np.float32)
    uv = cv2.perspectiveTransform(pt, H)[0][0]
    u_meas, v_meas = float(uv[0]), float(uv[1])
    err = ((u_meas - 0.5)**2 + (v_meas - 0.5)**2)**0.5

    return {
        "verified": err < 0.08,
        "error_pct": round(err * 100, 1),
        "u_meas": round(u_meas, 3),
        "v_meas": round(v_meas, 3),
        "laser_cam": [lx, ly]
    }

@app.post("/api/point_camera_xy")
def point_camera_xy(req: PointCameraRequest):
    """
    Apunta el láser físicamente en 1 solo paso a cualquier coordenada (x, y) de la cámara
    usando la Homografía 3x3 calibrada.
    """
    cal = _cargar_calibracion()
    H_list = cal.get("homography")
    if not H_list:
        return JSONResponse(status_code=400, content={"error": "Calibra el Área Segura primero para apuntar directo por homografía."})

    H = np.array(H_list, dtype=np.float32)
    pt = np.array([[[req.x, req.y]]], dtype=np.float32)
    uv = cv2.perspectiveTransform(pt, H)[0][0]
    u = max(0.0, min(1.0, float(uv[0])))
    v = max(0.0, min(1.0, float(uv[1])))

    try:
        res = requests.post(f"{ESP32_URL}/api/move_norm?u={u:.3f}&v={v:.3f}", timeout=2)
        return {"pointed": True, "u": round(u, 3), "v": round(v, 3), "esp32": res.json()}
    except Exception as e:
        return {"pointed": False, "error": str(e)}

@app.post("/api/detect_objects_local")
def api_detect_objects_local(payload: ImagePayload):
    """
    Endpoint rápido (<15ms) de visión local OpenCV + CIELAB para detectar bolitas y colores.
    """
    t0 = time.time()
    cal = _cargar_calibracion()
    img = decodificar_imagen(payload.image_base64)
    objetos = detectar_objetos_locales(img, cal.get("cam_corners"))
    return {
        "count": len(objetos),
        "objects": objetos,
        "ms": round((time.time() - t0) * 1000, 1)
    }

@app.post("/api/command_point")
def api_command_point(req: CommandRequest):
    """
    Interpreta un comando (ej. 'verde pastel', 'blancas', o lenguaje natural),
    busca localmente con OpenCV CIELAB y apunta con homografía o consulta a Gemini si es complejo.
    """
    cmd = req.command.lower().strip()
    cal = _cargar_calibracion()
    img = decodificar_imagen(req.image_base64)
    objetos = detectar_objetos_locales(img, cal.get("cam_corners"))

    color_map = [
        ("verde pastel", "Verde pastel"),
        ("pastel", "Verde pastel"),
        ("blanca", "Blanco"),
        ("blanco", "Blanco"),
        ("verde", "Verde"),
        ("amarill", "Amarillo"),
        ("azul", "Azul"),
        ("rosa", "Rosa"),
        ("rojo", "Rojo"),
        ("roja", "Rojo"),
    ]

    target_color = None
    for kw, col in color_map:
        if kw in cmd:
            target_color = col
            break

    if target_color:
        matches = [o for o in objetos if o["color"].lower() == target_color.lower()]
        if matches:
            top = matches[0]
            _notificar_esp32(top["nombre"], "detect")
            pointed = False
            if cal.get("homography"):
                point_camera_xy(PointCameraRequest(x=top["x"], y=top["y"]))
                pointed = True
            return {
                "source": "opencv_local",
                "matched_color": target_color,
                "count": len(matches),
                "target": top,
                "all_matches": matches,
                "pointed_directly": pointed
            }

    # Si no es un filtro de color local o no encontró coincidencia, usar Gemini
    return api_identify_and_point(ImagePayload(image_base64=req.image_base64))

if __name__ == "__main__":
    import socket
    import uvicorn
    cert_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cert.pem")
    key_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "key.pem")

    esp_found = auto_detect_esp32()
    ips = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except Exception:
        pass

    print(f"\n=========================================================")
    print(f"  SERVIDOR TRAK INICIADO EN HTTPS (PUERTO 8443)")
    print(f"  ESP32 detectado en: {esp_found}")
    if ips:
        for ip in ips:
            print(f"  Abre en tu CELULAR: https://{ip}:8443")
    else:
        print(f"  Abre en tu CELULAR: https://192.168.68.107:8443")
    print(f"=========================================================\n")

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8443,
        ssl_keyfile=key_path,
        ssl_certfile=cert_path
    )
