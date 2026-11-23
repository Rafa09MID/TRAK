#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Credenciales Wi-Fi de tu casa
const char* WIFI_SSID     = "KanHouse";
const char* WIFI_PASSWORD = "k#it12$49&33)d%";

// Red Wi-Fi propia de respaldo
const char* AP_SSID       = "ESP32-Laser-Gimbal";
const char* AP_PASS       = "12345678";

// Definición de pines
#define PIN_CABECEO   4   // Servo cabeceo (Pin 4)
#define PIN_ROTACION  5   // Servo rotación (Pin 5)
#define PIN_LASER     6   // Láser (Pin 6)
#define PIN_BOTON     0   // Botón BOOT

// PWM por Hardware Puro (LEDC)
#define CANAL_CABECEO   0
#define CANAL_ROTACION  1
#define PWM_FREQ        50
#define PWM_RES         14

#define DUTY_MIN  410   // 0°
#define DUTY_MAX  1966  // 180°

// -------------------------------------------------------------
// COORDENADAS CALIBRADAS DE LAS 4 ESQUINAS (ZONA SEGURA)
// Se guardan en memoria no volátil (NVS) para persistir tras reinicios
// -------------------------------------------------------------
Preferences prefs;

// Valores por defecto (se sobrescriben al calibrar desde la web)
int cTL = 150, rTL = 68;   // Esquina Superior Izquierda (u=0, v=0)
int cTR = 150, rTR = 180;  // Esquina Superior Derecha   (u=1, v=0)
int cBR = 81,  rBR = 180;  // Esquina Inferior Derecha   (u=1, v=1)
int cBL = 81,  rBL = 68;   // Esquina Inferior Izquierda (u=0, v=1)

// Pines RGB de la placa
const int RGB_PINS[] = {48, 38, 21};
Adafruit_NeoPixel led48(1, 48, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led38(1, 38, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led21(1, 21, NEO_GRB + NEO_KHZ800);

WebServer server(80);

// Variables de estado
int anguloCabeceo = 115;
int anguloRotacion = 124;
float normU = 0.5;
float normV = 0.5;
bool estadoLaser = true;
bool modoRectangulo = false;
int ladoActual = 1;
float pasoPerimetro = 0.0;

unsigned long ultimoDebounce = 0;
unsigned long ultimoPasoRect = 0;

void ponerColorLed(uint8_t r, uint8_t g, uint8_t b) {
    led48.setPixelColor(0, led48.Color(r, g, b));
    led48.show();
    led38.setPixelColor(0, led38.Color(r, g, b));
    led38.show();
    led21.setPixelColor(0, led21.Color(r, g, b));
    led21.show();
    for (int pin : RGB_PINS) {
        neopixelWrite(pin, r, g, b);
    }
}

void moverCabeceoHW(int angulo) {
    anguloCabeceo = constrain(angulo, 0, 180);
    int duty = map(anguloCabeceo, 0, 180, DUTY_MIN, DUTY_MAX);
    ledcWrite(CANAL_CABECEO, duty);
}

void moverRotacionHW(int angulo) {
    anguloRotacion = constrain(angulo, 0, 180);
    int duty = map(anguloRotacion, 0, 180, DUTY_MIN, DUTY_MAX);
    ledcWrite(CANAL_ROTACION, duty);
}

void actualizarLaser(bool encender) {
    estadoLaser = encender;
    digitalWrite(PIN_LASER, estadoLaser ? HIGH : LOW);
}

// Interpolación bilineal dentro del Área Segura: u in [0,1], v in [0,1]
void moverNormalizado(float u, float v) {
    u = constrain(u, 0.0f, 1.0f);
    v = constrain(v, 0.0f, 1.0f);
    normU = u;
    normV = v;

    float cTop = cTL * (1.0f - u) + cTR * u;
    float cBot = cBL * (1.0f - u) + cBR * u;
    int targetC = round(cTop * (1.0f - v) + cBot * v);

    float rTop = rTL * (1.0f - u) + rTR * u;
    float rBot = rBL * (1.0f - u) + rBR * u;
    int targetR = round(rTop * (1.0f - v) + rBot * v);

    moverCabeceoHW(targetC);
    moverRotacionHW(targetR);
}

void cargarCalibracion() {
    prefs.begin("trak_cal", true);
    cTL = prefs.getInt("cTL", 150);
    rTL = prefs.getInt("rTL", 68);
    cTR = prefs.getInt("cTR", 150);
    rTR = prefs.getInt("rTR", 180);
    cBR = prefs.getInt("cBR", 81);
    rBR = prefs.getInt("rBR", 180);
    cBL = prefs.getInt("cBL", 81);
    rBL = prefs.getInt("rBL", 68);
    prefs.end();
}

void guardarCalibracion() {
    prefs.begin("trak_cal", false);
    prefs.putInt("cTL", cTL);
    prefs.putInt("rTL", rTL);
    prefs.putInt("cTR", cTR);
    prefs.putInt("rTR", rTR);
    prefs.putInt("cBR", cBR);
    prefs.putInt("rBR", rBR);
    prefs.putInt("cBL", cBL);
    prefs.putInt("rBL", rBL);
    prefs.end();
}

// -------------------------------------------------------------
// PÁGINA WEB LOCAL DEL LÁSER (ÁREA SEGURA INTERACTIVA)
// -------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>TRAK - Control Área Segura</title>
    <style>
        :root {
            --bg: #070b14;
            --card: rgba(15, 23, 42, 0.94);
            --border: rgba(255, 255, 255, 0.12);
            --primary: #00d2ff;
            --primary-glow: rgba(0, 210, 255, 0.4);
            --safe: #10b981;
            --safe-glow: rgba(16, 185, 129, 0.45);
            --laser-on: #ff0055;
            --laser-glow: rgba(255, 0, 85, 0.6);
            --text: #f8fafc;
            --text-dim: #94a3b8;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        body {
            background: radial-gradient(circle at 50% 0%, #162238, var(--bg));
            color: var(--text);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 14px;
            user-select: none;
            -webkit-user-select: none;
        }
        .container {
            width: 100%;
            max-width: 440px;
            background: var(--card);
            border: 1px solid var(--border);
            border-radius: 24px;
            padding: 20px;
            box-shadow: 0 20px 50px rgba(0,0,0,0.65);
        }
        header { text-align: center; margin-bottom: 14px; }
        header h1 { font-size: 1.25rem; font-weight: 800; color: var(--primary); letter-spacing: 0.5px; text-shadow: 0 0 12px var(--primary-glow); }
        .badge { display: inline-block; padding: 4px 12px; border-radius: 12px; font-size: 0.72rem; background: rgba(16, 185, 129, 0.15); color: var(--safe); margin-top: 6px; font-weight: 700; border: 1px solid rgba(16, 185, 129, 0.3); }
        .status-box {
            background: rgba(0,0,0,0.35);
            border: 1px solid var(--border);
            border-radius: 14px;
            padding: 10px;
            margin-bottom: 14px;
            display: flex;
            justify-content: space-around;
            text-align: center;
        }
        .status-item span { display: block; font-size: 0.65rem; color: var(--text-dim); text-transform: uppercase; }
        .status-item b { font-size: 1.05rem; color: var(--primary); }

        /* CUADRADO INTERACTIVO DEL ÁREA SEGURA */
        .touchpad-wrapper {
            position: relative;
            width: 100%;
            aspect-ratio: 1 / 1;
            background: radial-gradient(circle at center, rgba(16, 185, 129, 0.08), rgba(0, 0, 0, 0.55));
            border: 2px solid var(--safe);
            border-radius: 18px;
            box-shadow: 0 0 25px rgba(16, 185, 129, 0.18), inset 0 0 25px rgba(16, 185, 129, 0.08);
            margin-bottom: 14px;
            overflow: hidden;
            touch-action: none;
            cursor: crosshair;
        }
        .grid-lines {
            position: absolute;
            inset: 0;
            background-image:
                linear-gradient(to right, rgba(255,255,255,0.06) 1px, transparent 1px),
                linear-gradient(to bottom, rgba(255,255,255,0.06) 1px, transparent 1px);
            background-size: 25% 25%;
            pointer-events: none;
        }
        .crosshair-center {
            position: absolute;
            left: 50%; top: 50%;
            width: 18px; height: 18px;
            transform: translate(-50%, -50%);
            border: 1px dashed rgba(255,255,255,0.35);
            border-radius: 50%;
            pointer-events: none;
        }
        .corner-tag {
            position: absolute;
            font-size: 0.62rem;
            font-weight: 700;
            color: rgba(16, 185, 129, 0.85);
            padding: 4px 6px;
            pointer-events: none;
        }
        .tag-tl { top: 2px; left: 4px; }
        .tag-tr { top: 2px; right: 4px; }
        .tag-bl { bottom: 2px; left: 4px; }
        .tag-br { bottom: 2px; right: 4px; }

        #laserCursor {
            position: absolute;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: radial-gradient(circle, #ffffff 18%, #ff0055 45%, rgba(255,0,85,0) 75%);
            box-shadow: 0 0 18px #ff0055;
            transform: translate(-50%, -50%);
            left: 50%;
            top: 50%;
            pointer-events: none;
            transition: left 0.05s linear, top 0.05s linear;
        }

        .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        button {
            padding: 13px 10px;
            border-radius: 13px;
            border: 1px solid var(--border);
            background: rgba(255,255,255,0.06);
            color: var(--text);
            font-size: 0.85rem;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.15s ease;
        }
        button:active { transform: scale(0.96); }
        .btn-center {
            background: var(--primary);
            color: #05101d;
            border-color: var(--primary);
            box-shadow: 0 0 16px var(--primary-glow);
        }
        .btn-laser {
            background: rgba(255, 0, 85, 0.14);
            border-color: rgba(255, 0, 85, 0.35);
            color: #ff3377;
        }
        .btn-laser.active {
            background: var(--laser-on);
            color: white;
            box-shadow: 0 0 22px var(--laser-glow);
        }
        .btn-rect {
            grid-column: span 2;
            background: rgba(16, 185, 129, 0.14);
            border-color: rgba(16, 185, 129, 0.4);
            color: #34d399;
        }
        .btn-rect.active {
            background: var(--safe);
            color: #05101d;
            box-shadow: 0 0 20px var(--safe-glow);
        }
        footer { text-align: center; margin-top: 12px; font-size: 0.68rem; color: var(--text-dim); }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>⚡ TRAK ÁREA SEGURA</h1>
            <div class="badge" id="ladoBadge">Pulsa en el cuadrado para mover el láser</div>
        </header>

        <div class="status-box">
            <div class="status-item"><span>Posición (X, Y)</span><b id="statusXY">50%, 50%</b></div>
            <div class="status-item"><span>Cabeceo</span><b id="statusC">--°</b></div>
            <div class="status-item"><span>Rotación</span><b id="statusR">--°</b></div>
            <div class="status-item"><span>Láser</span><b id="statusL">ON</b></div>
        </div>

        <div class="touchpad-wrapper" id="safePad">
            <div class="grid-lines"></div>
            <div class="crosshair-center"></div>
            <span class="corner-tag tag-tl" id="tagTL">TL</span>
            <span class="corner-tag tag-tr" id="tagTR">TR</span>
            <span class="corner-tag tag-bl" id="tagBL">BL</span>
            <span class="corner-tag tag-br" id="tagBR">BR</span>
            <div id="laserCursor"></div>
        </div>

        <div class="btn-grid">
            <button class="btn-center" onclick="irAlCentro()">🎯 Centro Área Segura</button>
            <button id="btnLaser" class="btn-laser active" onclick="toggleLaser()">⚡ Láser: <span id="laserText">ON</span></button>
            <button id="btnRect" class="btn-rect" onclick="toggleRect()">📐 Patrullar Perímetro del Área Segura</button>
        </div>

        <footer id="calFooter">Cargando calibración...</footer>
    </div>

    <script>
        const pad = document.getElementById('safePad');
        const cursor = document.getElementById('laserCursor');
        let dragging = false;
        let lastSend = 0;

        function enviarCoordenada(u, v, force = false) {
            u = Math.max(0, Math.min(1, u));
            v = Math.max(0, Math.min(1, v));

            cursor.style.left = (u * 100) + '%';
            cursor.style.top = (v * 100) + '%';
            document.getElementById('statusXY').innerText = `${Math.round(u*100)}%, ${Math.round(v*100)}%`;

            const now = Date.now();
            if (!force && now - lastSend < 65) return;
            lastSend = now;

            fetch(`/api/move_norm?u=${u.toFixed(3)}&v=${v.toFixed(3)}`, { method: 'POST' })
                .then(r => r.json())
                .then(aplicarEstado)
                .catch(() => {});
        }

        function procesarEventoPad(e, force = false) {
            const rect = pad.getBoundingClientRect();
            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;
            const u = (clientX - rect.left) / rect.width;
            const v = (clientY - rect.top) / rect.height;
            enviarCoordenada(u, v, force);
        }

        pad.addEventListener('pointerdown', (e) => { dragging = true; procesarEventoPad(e, true); });
        pad.addEventListener('pointermove', (e) => { if (dragging) procesarEventoPad(e, false); });
        window.addEventListener('pointerup', () => { dragging = false; });

        function irAlCentro() {
            enviarCoordenada(0.5, 0.5, true);
        }

        function toggleRect() {
            fetch('/api/rect/toggle', { method: 'POST' }).then(r => r.json()).then(aplicarEstado);
        }

        function toggleLaser() {
            fetch('/api/laser/toggle', { method: 'POST' }).then(r => r.json()).then(aplicarEstado);
        }

        function aplicarEstado(data) {
            document.getElementById('statusC').innerText = data.cabeceo + '°';
            document.getElementById('statusR').innerText = data.rotacion + '°';

            if (!dragging && data.u !== undefined && data.v !== undefined) {
                cursor.style.left = (data.u * 100) + '%';
                cursor.style.top = (data.v * 100) + '%';
                document.getElementById('statusXY').innerText = `${Math.round(data.u*100)}%, ${Math.round(data.v*100)}%`;
            }

            if (data.cal) {
                document.getElementById('tagTL').innerText = `TL (${data.cal.cTL}°/${data.cal.rTL}°)`;
                document.getElementById('tagTR').innerText = `TR (${data.cal.cTR}°/${data.cal.rTR}°)`;
                document.getElementById('tagBL').innerText = `BL (${data.cal.cBL}°/${data.cal.rBL}°)`;
                document.getElementById('tagBR').innerText = `BR (${data.cal.cBR}°/${data.cal.rBR}°)`;
                document.getElementById('calFooter').innerText = `Esquinas guardadas en ESP32 NVS | Listo`;
            }

            const btnR = document.getElementById('btnRect');
            const ladoB = document.getElementById('ladoBadge');
            if (data.rect) {
                btnR.classList.add('active');
                btnR.innerText = '⏸️ Pausar Patrullaje del Área Segura';
                ladoB.innerText = `Patrullando Lado ${data.lado} / 4`;
            } else {
                btnR.classList.remove('active');
                btnR.innerText = '📐 Patrullar Perímetro del Área Segura';
                ladoB.innerText = 'Pulsa en el cuadrado para mover el láser';
            }

            const btnL = document.getElementById('btnLaser');
            const statusL = document.getElementById('statusL');
            if (data.laser) {
                btnL.classList.add('active');
                document.getElementById('laserText').innerText = 'ON';
                statusL.innerText = 'ON';
                statusL.style.color = '#ff0055';
            } else {
                btnL.classList.remove('active');
                document.getElementById('laserText').innerText = 'OFF';
                statusL.innerText = 'OFF';
                statusL.style.color = '#94a3b8';
            }
        }

        function refrescar() {
            if (!dragging) {
                fetch('/api/status').then(r => r.json()).then(aplicarEstado).catch(() => {});
            }
        }
        refrescar();
        setInterval(refrescar, 500);
    </script>
</body>
</html>
)rawliteral";

String jsonEstado() {
    return String("{\"cabeceo\":") + anguloCabeceo +
           ",\"rotacion\":" + anguloRotacion +
           ",\"u\":" + String(normU, 3) +
           ",\"v\":" + String(normV, 3) +
           ",\"laser\":" + (estadoLaser ? "true" : "false") +
           ",\"rect\":" + (modoRectangulo ? "true" : "false") +
           ",\"lado\":" + ladoActual +
           ",\"cal\":{\"cTL\":" + cTL + ",\"rTL\":" + rTL +
           ",\"cTR\":" + cTR + ",\"rTR\":" + rTR +
           ",\"cBR\":" + cBR + ",\"rBR\":" + rBR +
           ",\"cBL\":" + cBL + ",\"rBL\":" + rBL + "}}";
}

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handleStatus() { server.send(200, "application/json", jsonEstado()); }

void handleSetServo() {
    if (server.hasArg("cabeceo") && server.hasArg("rotacion")) {
        modoRectangulo = false;
        int c = server.arg("cabeceo").toInt();
        int r = server.arg("rotacion").toInt();
        moverCabeceoHW(c);
        moverRotacionHW(r);
    }
    server.send(200, "application/json", jsonEstado());
}

void handleMoveNorm() {
    if (server.hasArg("u") && server.hasArg("v")) {
        modoRectangulo = false;
        float u = server.arg("u").toFloat();
        float v = server.arg("v").toFloat();
        moverNormalizado(u, v);
    }
    server.send(200, "application/json", jsonEstado());
}

void handleCenter() {
    modoRectangulo = false;
    moverNormalizado(0.5f, 0.5f);
    server.send(200, "application/json", jsonEstado());
}

void handleSafeArea() {
    if (server.hasArg("cTL") && server.hasArg("rTL") &&
        server.hasArg("cTR") && server.hasArg("rTR") &&
        server.hasArg("cBR") && server.hasArg("rBR") &&
        server.hasArg("cBL") && server.hasArg("rBL")) {
        cTL = constrain(server.arg("cTL").toInt(), 0, 180);
        rTL = constrain(server.arg("rTL").toInt(), 0, 180);
        cTR = constrain(server.arg("cTR").toInt(), 0, 180);
        rTR = constrain(server.arg("rTR").toInt(), 0, 180);
        cBR = constrain(server.arg("cBR").toInt(), 0, 180);
        rBR = constrain(server.arg("rBR").toInt(), 0, 180);
        cBL = constrain(server.arg("cBL").toInt(), 0, 180);
        rBL = constrain(server.arg("rBL").toInt(), 0, 180);
        guardarCalibracion();
        modoRectangulo = false;
        moverNormalizado(0.5f, 0.5f);
    }
    server.send(200, "application/json", jsonEstado());
}

void handleToggleLaser() {
    actualizarLaser(!estadoLaser);
    server.send(200, "application/json", jsonEstado());
}

void handleToggleRect() {
    modoRectangulo = !modoRectangulo;
    if (modoRectangulo) {
        actualizarLaser(true);
        ladoActual = 1;
        pasoPerimetro = 0.0f;
    }
    server.send(200, "application/json", jsonEstado());
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);

    pinMode(PIN_BOTON, INPUT_PULLUP);
    pinMode(PIN_LASER, OUTPUT);
    actualizarLaser(true);

    led48.begin();
    led38.begin();
    led21.begin();

    cargarCalibracion();

    // Configurar LEDC
    ledcSetup(CANAL_CABECEO, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_CABECEO, CANAL_CABECEO);

    ledcSetup(CANAL_ROTACION, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_ROTACION, CANAL_ROTACION);

    // Iniciar en el centro del Área Segura calibrada
    moverNormalizado(0.5f, 0.5f);
    ponerColorLed(0, 180, 255);

    // Wi-Fi Dual
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Servidor Web
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/servo", HTTP_POST, handleSetServo);
    server.on("/api/move_norm", HTTP_POST, handleMoveNorm);
    server.on("/api/center", HTTP_POST, handleCenter);
    server.on("/api/safe_area", HTTP_POST, handleSafeArea);
    server.on("/api/laser/toggle", HTTP_POST, handleToggleLaser);
    server.on("/api/rect/toggle", HTTP_POST, handleToggleRect);
    server.begin();

    // mDNS & OTA
    MDNS.begin("esp32-laser");
    ArduinoOTA.setHostname("esp32-laser");
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();

    if (digitalRead(PIN_BOTON) == LOW) {
        if (millis() - ultimoDebounce > 300) {
            ultimoDebounce = millis();
            modoRectangulo = !modoRectangulo;
            if (modoRectangulo) {
                actualizarLaser(true);
                ladoActual = 1;
                pasoPerimetro = 0.0f;
            }
        }
    }

    // Patrullaje continuo usando las 4 esquinas calibradas (u, v)
    if (modoRectangulo) {
        if (millis() - ultimoPasoRect >= 25) {
            ultimoPasoRect = millis();
            pasoPerimetro += 0.02f;
            if (pasoPerimetro >= 1.0f) {
                pasoPerimetro = 0.0f;
                ladoActual = (ladoActual % 4) + 1;
            }

            switch (ladoActual) {
                case 1: // TL -> TR (Borde superior)
                    ponerColorLed(0, 150, 255);
                    moverNormalizado(pasoPerimetro, 0.0f);
                    break;
                case 2: // TR -> BR (Lateral derecho)
                    ponerColorLed(0, 255, 60);
                    moverNormalizado(1.0f, pasoPerimetro);
                    break;
                case 3: // BR -> BL (Borde inferior)
                    ponerColorLed(255, 220, 0);
                    moverNormalizado(1.0f - pasoPerimetro, 1.0f);
                    break;
                case 4: // BL -> TL (Lateral izquierdo)
                    ponerColorLed(255, 0, 100);
                    moverNormalizado(0.0f, 1.0f - pasoPerimetro);
                    break;
            }
        }
    }
}
