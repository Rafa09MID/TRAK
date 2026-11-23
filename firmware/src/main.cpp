#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
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
// COORDENADAS CALIBRADAS DEL RECTÁNGULO (ZONA SEGURA)
// -------------------------------------------------------------
// Centro horizontal: 124° | Lateral: 180° (delta 56°) -> Lateral opuesto: 68°
// Línea inferior: 81°     | Línea superior: 150°
const int C_MIN = 81;   // Cabeceo inferior
const int C_MAX = 150;  // Cabeceo superior
const int R_MIN = 68;   // Rotación izquierda
const int R_MAX = 180;  // Rotación derecha

// Pines RGB de la placa
const int RGB_PINS[] = {48, 38, 21};
Adafruit_NeoPixel led48(1, 48, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led38(1, 38, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led21(1, 21, NEO_GRB + NEO_KHZ800);

WebServer server(80);

// Variables de estado
int anguloCabeceo = 81;
int anguloRotacion = 68;
bool estadoLaser = true;
bool modoRectangulo = true;  // Activo por defecto para iniciar el patrullaje
int ladoActual = 1;

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

// -------------------------------------------------------------
// PÁGINA WEB HTML / CSS / JS EMBEBIDA
// -------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-S3 Rectángulo Calibrado</title>
    <style>
        :root {
            --bg: #0b0f19;
            --card: rgba(18, 26, 43, 0.92);
            --border: rgba(255, 255, 255, 0.1);
            --primary: #00d2ff;
            --primary-glow: rgba(0, 210, 255, 0.35);
            --laser-on: #ff0055;
            --laser-glow: rgba(255, 0, 85, 0.6);
            --text: #f0f4f8;
            --text-dim: #8ba2b5;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        body {
            background: radial-gradient(circle at 50% 10%, #152238, var(--bg));
            color: var(--text);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }
        .container {
            width: 100%;
            max-width: 460px;
            background: var(--card);
            backdrop-filter: blur(16px);
            border: 1px solid var(--border);
            border-radius: 24px;
            padding: 24px;
            box-shadow: 0 20px 50px rgba(0,0,0,0.6);
        }
        header { text-align: center; margin-bottom: 16px; }
        header h1 { font-size: 1.35rem; font-weight: 700; color: var(--primary); text-shadow: 0 0 15px var(--primary-glow); }
        .badge { display: inline-block; padding: 4px 12px; border-radius: 12px; font-size: 0.75rem; background: rgba(0, 210, 255, 0.15); color: var(--primary); margin-top: 4px; font-weight: 600; }
        .status-box {
            background: rgba(0,0,0,0.3);
            border: 1px solid var(--border);
            border-radius: 14px;
            padding: 12px;
            margin-bottom: 16px;
            display: flex;
            justify-content: space-around;
            text-align: center;
        }
        .status-item span { display: block; font-size: 0.7rem; color: var(--text-dim); text-transform: uppercase; }
        .status-item b { font-size: 1.15rem; color: var(--primary); }
        .control-group {
            background: rgba(255,255,255,0.03);
            border: 1px solid rgba(255,255,255,0.06);
            border-radius: 16px;
            padding: 14px;
            margin-bottom: 12px;
        }
        .group-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
        .group-title { font-size: 0.9rem; font-weight: 600; }
        .val-badge { font-size: 1.05rem; font-weight: 700; color: var(--primary); background: rgba(0,0,0,0.3); padding: 2px 8px; border-radius: 8px; border: 1px solid var(--border); }
        input[type=range] { width: 100%; height: 8px; border-radius: 4px; background: #1c273a; outline: none; -webkit-appearance: none; }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 22px; height: 22px; border-radius: 50%; background: var(--primary); cursor: pointer; box-shadow: 0 0 10px var(--primary-glow); }
        .scale { display: flex; justify-content: space-between; font-size: 0.68rem; color: var(--text-dim); margin-top: 4px; }
        .section-label { font-size: 0.75rem; font-weight: 700; text-transform: uppercase; color: var(--text-dim); margin: 12px 0 6px; letter-spacing: 0.5px; }
        .grid-esquinas { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
        .corner-btn { padding: 9px; font-size: 0.75rem; background: rgba(255,255,255,0.04); border-radius: 10px; border: 1px solid var(--border); color: var(--text); }
        .corner-btn:hover { background: rgba(0, 210, 255, 0.15); border-color: var(--primary); }
        .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 14px; }
        button {
            padding: 12px;
            border-radius: 12px;
            border: 1px solid var(--border);
            background: rgba(255,255,255,0.05);
            color: var(--text);
            font-size: 0.88rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s ease;
        }
        button:active { transform: scale(0.97); }
        .btn-rect {
            grid-column: span 2;
            background: rgba(0, 210, 255, 0.15);
            border-color: rgba(0, 210, 255, 0.4);
            color: #00d2ff;
        }
        .btn-rect.active {
            background: #00d2ff;
            color: #06111f;
            box-shadow: 0 0 20px var(--primary-glow);
        }
        .btn-laser {
            grid-column: span 2;
            background: rgba(255, 0, 85, 0.12);
            border-color: rgba(255, 0, 85, 0.3);
            color: #ff3377;
        }
        .btn-laser.active {
            background: var(--laser-on);
            color: white;
            box-shadow: 0 0 25px var(--laser-glow);
        }
        footer { text-align: center; margin-top: 14px; font-size: 0.72rem; color: var(--text-dim); }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>ZONA SEGURA RECTANGULAR</h1>
            <div class="badge" id="ladoBadge">Lado 1: Inferior (Azul)</div>
        </header>

        <div class="status-box">
            <div class="status-item"><span>Cabeceo (Arriba/Abajo)</span><b id="statusC">81°</b></div>
            <div class="status-item"><span>Rotación (Eje)</span><b id="statusR">68°</b></div>
            <div class="status-item"><span>Láser</span><b id="statusL">ON</b></div>
        </div>

        <!-- BOTONES PRINCIPALES -->
        <div class="btn-grid">
            <button id="btnRect" class="btn-rect active" onclick="toggleRect()">
                📐 Seguir Perímetro del Rectángulo
            </button>
            <button id="btnLaser" class="btn-laser active" onclick="toggleLaser()">
                ⚡ LÁSER: <span id="laserText">ENCENDIDO</span>
            </button>
        </div>

        <!-- ESQUINAS CALIBRADAS -->
        <div class="section-label">Saltar a Esquinas Calibradas:</div>
        <div class="grid-esquinas">
            <button class="corner-btn" onclick="moverEsquina(150, 68)">↖ Sup-Izq (150°, 68°)</button>
            <button class="corner-btn" onclick="moverEsquina(150, 180)">↗ Sup-Der (150°, 180°)</button>
            <button class="corner-btn" onclick="moverEsquina(81, 68)">↙ Inf-Izq (81°, 68°)</button>
            <button class="corner-btn" onclick="moverEsquina(81, 180)">↘ Inf-Der (81°, 180°)</button>
        </div>

        <!-- SLIDERS MANUALES -->
        <div class="section-label">Ajuste Manual Fino:</div>
        <div class="control-group">
            <div class="group-header">
                <span class="group-title">Cabeceo [81° a 150°]</span>
                <span class="val-badge" id="valCabeceo">81°</span>
            </div>
            <input type="range" id="sliderCabeceo" min="0" max="180" value="81" oninput="actualizarSlider(this, 'valCabeceo')" onchange="enviarServos()">
            <div class="scale"><span>0°</span><span>81° (Inf)</span><span>150° (Sup)</span><span>180°</span></div>
        </div>

        <div class="control-group">
            <div class="group-header">
                <span class="group-title">Rotación [68° a 180°]</span>
                <span class="val-badge" id="valRotacion">68°</span>
            </div>
            <input type="range" id="sliderRotacion" min="0" max="180" value="68" oninput="actualizarSlider(this, 'valRotacion')" onchange="enviarServos()">
            <div class="scale"><span>0°</span><span>68° (Izq)</span><span>124° (Centro)</span><span>180° (Der)</span></div>
        </div>

        <footer>Calibración: [81°..150°] x [68°..180°] | LED activo en ESP32</footer>
    </div>

    <script>
        function actualizarSlider(slider, badgeId) {
            document.getElementById(badgeId).innerText = slider.value + '°';
        }
        function enviarServos() {
            const c = document.getElementById('sliderCabeceo').value;
            const r = document.getElementById('sliderRotacion').value;
            fetch('/api/servo?cabeceo=' + c + '&rotacion=' + r, { method: 'POST' });
        }
        function moverEsquina(c, r) {
            fetch('/api/servo?cabeceo=' + c + '&rotacion=' + r, { method: 'POST' }).then(r=>r.json()).then(aplicarEstado);
        }
        function toggleRect() {
            fetch('/api/rect/toggle', { method: 'POST' }).then(r=>r.json()).then(aplicarEstado);
        }
        function toggleLaser() {
            fetch('/api/laser/toggle', { method: 'POST' }).then(r=>r.json()).then(aplicarEstado);
        }
        function aplicarEstado(data) {
            document.getElementById('statusC').innerText = data.cabeceo + '°';
            document.getElementById('statusR').innerText = data.rotacion + '°';
            document.getElementById('sliderCabeceo').value = data.cabeceo;
            document.getElementById('sliderRotacion').value = data.rotacion;
            document.getElementById('valCabeceo').innerText = data.cabeceo + '°';
            document.getElementById('valRotacion').innerText = data.rotacion + '°';

            const btnR = document.getElementById('btnRect');
            const ladoB = document.getElementById('ladoBadge');
            if (data.rect) {
                btnR.classList.add('active');
                btnR.innerText = '⏸️ Pausar Recorrido del Rectángulo';
                const nombresLados = [
                    "",
                    "Lado 1: Base Inferior (Azul)",
                    "Lado 2: Subida Derecha (Verde)",
                    "Lado 3: Borde Superior (Amarillo)",
                    "Lado 4: Bajada Izquierda (Rojo/Magenta)"
                ];
                ladoB.innerText = nombresLados[data.lado] || "Siguiendo Perímetro";
            } else {
                btnR.classList.remove('active');
                btnR.innerText = '📐 Seguir Perímetro del Rectángulo';
                ladoB.innerText = 'Control Manual / Pausa';
            }

            const btnL = document.getElementById('btnLaser');
            const statusL = document.getElementById('statusL');
            if (data.laser) {
                btnL.classList.add('active');
                document.getElementById('laserText').innerText = 'ENCENDIDO';
                statusL.innerText = 'ON';
                statusL.style.color = '#ff0055';
            } else {
                btnL.classList.remove('active');
                document.getElementById('laserText').innerText = 'APAGADO';
                statusL.innerText = 'OFF';
                statusL.style.color = '#8ba2b5';
            }
        }
        function refrescar() {
            fetch('/api/status').then(r=>r.json()).then(aplicarEstado).catch(()=>{});
        }
        refrescar();
        setInterval(refrescar, 400);
    </script>
</body>
</html>
)rawliteral";

String jsonEstado() {
    return String("{\"cabeceo\":") + anguloCabeceo +
           ",\"rotacion\":" + anguloRotacion +
           ",\"laser\":" + (estadoLaser ? "true" : "false") +
           ",\"rect\":" + (modoRectangulo ? "true" : "false") +
           ",\"lado\":" + ladoActual + "}";
}

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handleStatus() { server.send(200, "application/json", jsonEstado()); }

void handleSetServo() {
    if (server.hasArg("cabeceo") && server.hasArg("rotacion")) {
        modoRectangulo = false; // El ajuste manual pausa el recorrido automático
        int c = server.arg("cabeceo").toInt();
        int r = server.arg("rotacion").toInt();
        moverCabeceoHW(c);
        moverRotacionHW(r);
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
    }
    server.send(200, "application/json", jsonEstado());
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);

    pinMode(PIN_BOTON, INPUT_PULLUP);
    pinMode(PIN_LASER, OUTPUT);
    actualizarLaser(true); // Láser encendido para proyectar el rectángulo

    led48.begin();
    led38.begin();
    led21.begin();

    // Configurar LEDC
    ledcSetup(CANAL_CABECEO, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_CABECEO, CANAL_CABECEO);

    ledcSetup(CANAL_ROTACION, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_ROTACION, CANAL_ROTACION);

    // Posición inicial: Esquina 1 (Inferior Izquierda: Cabeceo 81°, Rotación 68°)
    anguloCabeceo = C_MIN;
    anguloRotacion = R_MIN;
    ladoActual = 1;
    moverCabeceoHW(anguloCabeceo);
    moverRotacionHW(anguloRotacion);

    // Color inicial Lado 1: Azul
    ponerColorLed(0, 150, 255);

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

    // Botón físico BOOT alterna el patrullaje del rectángulo
    if (digitalRead(PIN_BOTON) == LOW) {
        if (millis() - ultimoDebounce > 300) {
            ultimoDebounce = millis();
            modoRectangulo = !modoRectangulo;
            if (modoRectangulo) actualizarLaser(true);
        }
    }

    // -------------------------------------------------------------
    // PATRULLAJE CONTINUO DEL PERÍMETRO DEL RECTÁNGULO
    // Cada paso cambia suavemente y la luz LED cambia con cada lado
    // -------------------------------------------------------------
    if (modoRectangulo) {
        if (millis() - ultimoPasoRect >= 22) { // Control de velocidad constante y suave
            ultimoPasoRect = millis();

            switch (ladoActual) {
                // LADO 1: Base Inferior (de Esquina 1 a Esquina 2)
                // Cabeceo fijo en 81° | Rotación avanza de 68° a 180°
                case 1:
                    ponerColorLed(0, 150, 255); // LED en AZUL CIAN
                    anguloCabeceo = C_MIN;
                    anguloRotacion++;
                    if (anguloRotacion >= R_MAX) {
                        anguloRotacion = R_MAX;
                        ladoActual = 2; // Pasa a subir por la derecha
                    }
                    break;

                // LADO 2: Lateral Derecho (de Esquina 2 a Esquina 3)
                // Rotación fija en 180° | Cabeceo sube de 81° a 150°
                case 2:
                    ponerColorLed(0, 255, 60); // LED en VERDE ESMERALDA
                    anguloRotacion = R_MAX;
                    anguloCabeceo++;
                    if (anguloCabeceo >= C_MAX) {
                        anguloCabeceo = C_MAX;
                        ladoActual = 3; // Pasa a avanzar por el techo
                    }
                    break;

                // LADO 3: Borde Superior (de Esquina 3 a Esquina 4)
                // Cabeceo fijo en 150° | Rotación retrocede de 180° a 68°
                case 3:
                    ponerColorLed(255, 220, 0); // LED en AMARILLO ORO
                    anguloCabeceo = C_MAX;
                    anguloRotacion--;
                    if (anguloRotacion <= R_MIN) {
                        anguloRotacion = R_MIN;
                        ladoActual = 4; // Pasa a bajar por la izquierda
                    }
                    break;

                // LADO 4: Lateral Izquierdo (de Esquina 4 a Esquina 1)
                // Rotación fija en 68° | Cabeceo baja de 150° a 81°
                case 4:
                    ponerColorLed(255, 0, 100); // LED en ROJO / MAGENTA
                    anguloRotacion = R_MIN;
                    anguloCabeceo--;
                    if (anguloCabeceo <= C_MIN) {
                        anguloCabeceo = C_MIN;
                        ladoActual = 1; // Completa el rectángulo y reinicia
                    }
                    break;
            }

            moverCabeceoHW(anguloCabeceo);
            moverRotacionHW(anguloRotacion);
        }
    }
}
