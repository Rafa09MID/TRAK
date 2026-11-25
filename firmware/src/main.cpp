#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Red Wi-Fi por defecto: Hotspot del celular ("rafa")
String wifiSsidPrimary = "rafa";
String wifiPassPrimary = "12345678";

WiFiMulti wifiMulti;
unsigned long ultimoCheckWifi = 0;

// Red Wi-Fi propia de respaldo
const char* AP_SSID       = "ESP32-Laser-Gimbal";
const char* AP_PASS       = "12345678";

// Definición de pines
#define PIN_CABECEO   39  // Servo cabeceo (Pin 39)
#define PIN_ROTACION  40  // Servo rotación (Pin 40)
#define PIN_LASER     41  // LED multicolor 3 pines (Pin 41)
#define PIN_TRIG      15  // Ultrasónico TRIG (Pin 15)
#define PIN_ECHO      16  // Ultrasónico ECHO (Pin 16)
int distanciaCM = 999;
unsigned long ultimoPing = 0;
unsigned long ultimoNotifyMs = 0;
Adafruit_NeoPixel ledPuntero(1, PIN_LASER, NEO_GRB + NEO_KHZ800);
uint8_t ledR = 255, ledG = 255, ledB = 255; // Blanco por defecto
bool modoFiesta = false;
unsigned long inicioFiesta = 0;
unsigned long ultimoPasoFiesta = 0;
int pasoFiesta = 0;
int baseCabeceoFiesta = 90;

bool modoDisparo = false;
unsigned long inicioDisparo = 0;
unsigned long ultimoPasoDisparo = 0;
int pasoDisparo = 0;
int baseCabeceoDisparo = 135;
#define PIN_BUZZER    7   // Buzzer 2 pines (Pin 7)
#define PIN_OLED_SDA  8   // OLED SDA (Pin 8)
#define PIN_OLED_SCL  9   // OLED SCL (Pin 9)
#define PIN_BOTON     0   // Botón BOOT

// PWM por Hardware Puro (LEDC)
#define CANAL_CABECEO   0
#define CANAL_ROTACION  1
#define CANAL_BUZZER    2
#define PWM_FREQ        50
#define PWM_RES         14

#define DUTY_MIN  410   // 0°
#define DUTY_MAX  1966  // 180°

// Pantalla OLED 0.96" 128x64
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledDisponible = false;

Preferences prefs;

// Coordenadas calibradas de las 4 esquinas
int cTL = 150, rTL = 68;
int cTR = 150, rTR = 180;
int cBR = 81,  rBR = 180;
int cBL = 81,  rBL = 68;

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

String textoOledPrincipal = "Listo para apuntar";
bool oledNecesitaActualizar = true;

unsigned long ultimoDebounce = 0;
unsigned long ultimoPasoRect = 0;

// Estado no bloqueante del Buzzer
int buzzerPaso = 0;
int buzzerModo = 0; // 0=off, 1=confirm, 2=detect
unsigned long buzzerTimer = 0;

String sanitizarASCII(String s) {
    s.replace("á", "a"); s.replace("é", "e"); s.replace("í", "i");
    s.replace("ó", "o"); s.replace("ú", "u"); s.replace("ñ", "n");
    s.replace("Á", "A"); s.replace("É", "E"); s.replace("Í", "I");
    s.replace("Ó", "O"); s.replace("Ú", "U"); s.replace("Ñ", "N");
    s.replace("ü", "u"); s.replace("Ü", "U");
    return s;
}

void iniciarBuzzer(int modo) {
    buzzerModo = modo;
    buzzerPaso = 1;
    buzzerTimer = millis();
    if (modo == 1) {
        // Confirmación: tono corto nítido 2200 Hz
        ledcWriteTone(CANAL_BUZZER, 2200);
    } else if (modo == 2) {
        // Detección de objeto: doble tono ascendente 1700 -> 2500 Hz
        ledcWriteTone(CANAL_BUZZER, 1700);
    }
}

void actualizarBuzzer() {
    if (buzzerModo == 0) return;
    unsigned long dt = millis() - buzzerTimer;
    if (buzzerModo == 1) {
        if (buzzerPaso == 1 && dt >= 110) {
            ledcWriteTone(CANAL_BUZZER, 0);
            buzzerModo = 0;
        }
    } else if (buzzerModo == 2) {
        if (buzzerPaso == 1 && dt >= 95) {
            ledcWriteTone(CANAL_BUZZER, 2500);
            buzzerPaso = 2;
            buzzerTimer = millis();
        } else if (buzzerPaso == 2 && dt >= 140) {
            ledcWriteTone(CANAL_BUZZER, 0);
            buzzerModo = 0;
        }
    }
}

void dibujarOled() {
    if (!oledDisponible) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Barra superior (y=0..12)
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("TRAK  L:");
    display.print(estadoLaser ? "ON " : "OFF");

    float cartX = (normU - 0.5f) * 10.0f;
    float cartY = (0.5f - normV) * 10.0f;
    display.setCursor(74, 0);
    display.printf("%+.0f,%+.0f", cartX, cartY);
    display.drawFastHLine(0, 11, 128, SSD1306_WHITE);

    // Zona central: Objeto identificado por Gemini / OpenCV (ajustado al tamaño)
    String msg = sanitizarASCII(textoOledPrincipal);
    msg.trim();
    if (msg.length() == 0) msg = "En espera";

    display.setCursor(0, 15);
    display.setTextSize(1);
    display.println("OBJETO:");

    display.setTextWrap(true);
    if (msg.length() <= 10) {
        // Texto grande si cabe en 1 linea (max 10 chars a size 2)
        display.setTextSize(2);
        display.setCursor(0, 27);
        display.println(msg);
    } else if (msg.length() <= 20) {
        // Texto grande en 2 lineas
        display.setTextSize(2);
        display.setCursor(0, 24);
        display.println(msg.substring(0, 10));
        display.setCursor(0, 41);
        display.println(msg.substring(10));
    } else {
        // Texto compacto size 1 (21 chars/linea x 3 lineas = 63 chars sin desbordar)
        display.setTextSize(1);
        display.setCursor(0, 26);
        if (msg.length() > 60) msg = msg.substring(0, 57) + "...";
        display.println(msg);
    }

    // Pie de pantalla (y=56): IP completa para conectar facil
    display.setTextSize(1);
    display.drawFastHLine(0, 54, 128, SSD1306_WHITE);
    display.setCursor(0, 56);
    if (WiFi.status() == WL_CONNECTED) {
        display.print(WiFi.localIP().toString());
    } else {
        display.print("AP 192.168.4.1");
    }

    display.display();
    oledNecesitaActualizar = false;
}

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

unsigned long ultimoPasoLed = 0;
float ledHue = 0.0f;
float ledBreathPhase = 0.0f;

void actualizarLedGradienteRespiracion() {
    if (millis() - ultimoPasoLed < 28) return;
    ultimoPasoLed = millis();

    ledHue += 0.005f;
    if (ledHue >= 1.0f) ledHue -= 1.0f;

    ledBreathPhase += 0.06f;
    if (ledBreathPhase >= 6.283185f) ledBreathPhase -= 6.283185f;

    // Brillo suave en onda senoidal (respiracion) entre 10% y 100%
    float v = 0.10f + 0.90f * (0.5f + 0.5f * sinf(ledBreathPhase));

    // Conversion HSV -> RGB (S = 1.0)
    float h6 = ledHue * 6.0f;
    int i = (int)floorf(h6);
    float f = h6 - i;
    float q = v * (1.0f - f);
    float t = v * f;
    float r = 0, g = 0, b = 0;

    switch (i % 6) {
        case 0: r = v; g = t; b = 0; break;
        case 1: r = q; g = v; b = 0; break;
        case 2: r = 0; g = v; b = t; break;
        case 3: r = 0; g = q; b = v; break;
        case 4: r = t; g = 0; b = v; break;
        case 5: r = v; g = 0; b = q; break;
    }

    ponerColorLed((uint8_t)(r * 255.0f), (uint8_t)(g * 255.0f), (uint8_t)(b * 255.0f));
}

int targetCabeceo = 90;
int targetRotacion = 90;
unsigned long ultimoPasoServo = 0;

void escribirCabeceoPWM(int angulo) {
    anguloCabeceo = constrain(angulo, 0, 180);
    int duty = map(anguloCabeceo, 0, 180, DUTY_MIN, DUTY_MAX);
    ledcWrite(CANAL_CABECEO, duty);
}

void escribirRotacionPWM(int angulo) {
    anguloRotacion = constrain(angulo, 0, 180);
    int duty = map(anguloRotacion, 0, 180, DUTY_MIN, DUTY_MAX);
    ledcWrite(CANAL_ROTACION, duty);
}

void actualizarServosSuaves() {
    if (millis() - ultimoPasoServo < 12) return;
    ultimoPasoServo = millis();
    if (anguloCabeceo < targetCabeceo) escribirCabeceoPWM(anguloCabeceo + 1);
    else if (anguloCabeceo > targetCabeceo) escribirCabeceoPWM(anguloCabeceo - 1);

    if (anguloRotacion < targetRotacion) escribirRotacionPWM(anguloRotacion + 1);
    else if (anguloRotacion > targetRotacion) escribirRotacionPWM(anguloRotacion - 1);
}

void moverCabeceoHW(int angulo) {
    targetCabeceo = constrain(angulo, 0, 180);
    oledNecesitaActualizar = true;
}

void moverRotacionHW(int angulo) {
    targetRotacion = constrain(angulo, 0, 180);
    oledNecesitaActualizar = true;
}

void actualizarLaser(bool encender) {
    estadoLaser = encender;
    digitalWrite(PIN_LASER, estadoLaser ? HIGH : LOW);
    if (estadoLaser) {
        ledPuntero.setPixelColor(0, ledPuntero.Color(ledR, ledG, ledB));
    } else {
        ledPuntero.setPixelColor(0, 0);
    }
    ledPuntero.show();
    oledNecesitaActualizar = true;
}

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

// Web local embebida
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>TRAK - Area Segura</title>
    <style>
        :root { --bg:#F5F5F7; --card:#FFFFFF; --border:rgba(0,0,0,0.08); --blue:#007AFF; --green:#34C759; --red:#FF3B30; --text:#1D1D1F; --dim:#86868B; }
        * { box-sizing:border-box; margin:0; padding:0; font-family:-apple-system,BlinkMacSystemFont,sans-serif; }
        body { background:var(--bg); color:var(--text); min-height:100vh; display:flex; justify-content:center; align-items:center; padding:14px; }
        .container { width:100%; max-width:420px; background:var(--card); border:1px solid var(--border); border-radius:20px; padding:18px; box-shadow:0 4px 20px rgba(0,0,0,0.05); }
        header { text-align:center; margin-bottom:12px; }
        header h1 { font-size:1.15rem; font-weight:700; }
        .badge { font-size:0.75rem; color:var(--dim); margin-top:4px; }
        .status-box { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; margin-bottom:12px; text-align:center; }
        .status-item { background:var(--bg); border-radius:12px; padding:8px; }
        .status-item span { display:block; font-size:0.64rem; color:var(--dim); text-transform:uppercase; }
        .status-item b { font-size:0.95rem; }
        .touchpad { position:relative; width:100%; aspect-ratio:1/1; background:#fff; border:2px solid var(--green); border-radius:16px; margin-bottom:12px; touch-action:none; cursor:crosshair; }
        #laserCursor { position:absolute; width:22px; height:22px; border-radius:50%; background:var(--red); border:3px solid #fff; box-shadow:0 2px 8px rgba(255,59,48,0.5); transform:translate(-50%,-50%); left:50%; top:50%; pointer-events:none; }
        .btn-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
        button { padding:12px; border-radius:12px; border:1px solid var(--border); background:var(--bg); font-weight:600; font-size:0.85rem; cursor:pointer; }
        .btn-blue { background:var(--blue); color:#fff; border-color:var(--blue); }
        .btn-red.active { background:var(--red); color:#fff; border-color:var(--red); }
        .btn-full { grid-column:span 2; background:var(--green); color:#fff; border-color:var(--green); }
    </style>
</head>
<body>
    <div class="container">
        <header><h1>TRAK Area Segura</h1><div class="badge" id="oledTxt">Listo</div></header>
        <div class="status-box">
            <div class="status-item"><span>Coord</span><b id="sXY">0.0, 0.0</b></div>
            <div class="status-item"><span>Cabeceo</span><b id="sC">--°</b></div>
            <div class="status-item"><span>Rotacion</span><b id="sR">--°</b></div>
        </div>
        <div class="touchpad" id="pad"><div id="laserCursor"></div></div>
        <div class="btn-grid">
            <button class="btn-blue" onclick="fetch('/api/center',{method:'POST'}).then(r=>r.json()).then(ui)">Centro</button>
            <button id="btnL" class="btn-red active" onclick="fetch('/api/laser/toggle',{method:'POST'}).then(r=>r.json()).then(ui)">Laser: ON</button>
            <button class="btn-full" onclick="fetch('/api/rect/toggle',{method:'POST'}).then(r=>r.json()).then(ui)">Patrullar Perimetro</button>
        </div>
    </div>
    <script>
        const pad=document.getElementById('pad'), cur=document.getElementById('laserCursor');
        let drag=false, last=0;
        function send(u,v,f){
            u=Math.max(0,Math.min(1,u)); v=Math.max(0,Math.min(1,v));
            cur.style.left=(u*100)+'%'; cur.style.top=(v*100)+'%';
            document.getElementById('sXY').innerText=((u-0.5)*10).toFixed(1)+', '+((0.5-v)*10).toFixed(1);
            if(!f && Date.now()-last<65) return; last=Date.now();
            fetch(`/api/move_norm?u=${u.toFixed(3)}&v=${v.toFixed(3)}`,{method:'POST'}).then(r=>r.json()).then(ui);
        }
        pad.onpointerdown=e=>{drag=true; const r=pad.getBoundingClientRect(); send((e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height,true);};
        pad.onpointermove=e=>{if(!drag)return; const r=pad.getBoundingClientRect(); send((e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height,false);};
        window.onpointerup=()=>{drag=false;};
        function ui(d){
            document.getElementById('sC').innerText=d.cabeceo+'°';
            document.getElementById('sR').innerText=d.rotacion+'°';
            document.getElementById('oledTxt').innerText=d.oled||'Listo';
            document.getElementById('btnL').classList.toggle('active',d.laser);
            document.getElementById('btnL').innerText=d.laser?'Laser: ON':'Laser: OFF';
        }
        fetch('/api/status').then(r=>r.json()).then(ui);
    </script>
</body>
</html>
)rawliteral";

void dibujarRadarOled() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    if (modoDisparo) {
        display.setCursor(16, 0);
        display.print("! DISPARANDO !");
        int anim = (pasoDisparo % 3) * 8;
        display.drawLine(64, 63, 40 - anim, 18, SSD1306_WHITE);
        display.drawLine(64, 63, 64, 14 + (anim / 2), SSD1306_WHITE);
        display.drawLine(64, 63, 88 + anim, 18, SSD1306_WHITE);
        display.fillCircle(64, 24 + anim, 3, SSD1306_WHITE);
    } else {
        display.setCursor(0, 0);
        if (textoOledPrincipal.length() > 0 && millis() - ultimoNotifyMs < 5000) {
            display.print("OBJ: ");
            display.print(sanitizarASCII(textoOledPrincipal).substring(0, 15));
        } else {
            display.print("RADAR: ");
            if (distanciaCM < 300) {
                display.print(distanciaCM);
                display.print(" cm");
            } else {
                display.print("Libre");
            }
        }
        display.drawCircle(64, 63, 18, SSD1306_WHITE);
        display.drawCircle(64, 63, 34, SSD1306_WHITE);
        display.drawCircle(64, 63, 50, SSD1306_WHITE);
        if (distanciaCM < 100) {
            int r = map(constrain(distanciaCM, 2, 100), 2, 100, 8, 50);
            display.fillCircle(64, 63 - r, 3, SSD1306_WHITE);
        }
    }
    display.display();
}

String jsonEstado() {
    return String("{\"fw_ver\":\"v2.1\",\"cabeceo\":") + anguloCabeceo +
           ",\"rotacion\":" + anguloRotacion +
           ",\"u\":" + String(normU, 3) +
           ",\"v\":" + String(normV, 3) +
           ",\"laser\":" + (estadoLaser ? "true" : "false") +
           ",\"rect\":" + (modoRectangulo ? "true" : "false") +
           ",\"lado\":" + ladoActual +
           ",\"oled\":\"" + sanitizarASCII(textoOledPrincipal) + "\"" +
           ",\"wifi_ssid\":\"" + WiFi.SSID() + "\"" +
           ",\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("192.168.4.1")) + "\"" +
           ",\"cal\":{\"cTL\":" + cTL + ",\"rTL\":" + rTL +
           ",\"cTR\":" + cTR + ",\"rTR\":" + rTR +
           ",\"cBR\":" + cBR + ",\"rBR\":" + rBR +
           ",\"cBL\":" + cBL + ",\"rBL\":" + rBL + "}}";
}

void handleFiesta() {
    if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
        ledR = constrain(server.arg("r").toInt(), 0, 255);
        ledG = constrain(server.arg("g").toInt(), 0, 255);
        ledB = constrain(server.arg("b").toInt(), 0, 255);
        actualizarLaser(true);
    }
    bool activar = !server.hasArg("on") || server.arg("on") != "0";
    modoFiesta = activar;
    // Lectura ultrasónica cada 120ms
    if (millis() - ultimoPing >= 120) {
        ultimoPing = millis();
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);
        long dur = pulseIn(PIN_ECHO, HIGH, 15000);
        distanciaCM = (dur > 0) ? (int)(dur * 0.034f / 2.0f) : 999;

        if (distanciaCM > 1 && distanciaCM <= 10 && !modoDisparo) {
            modoDisparo = true;
            inicioDisparo = millis();
            ultimoPasoDisparo = 0;
            pasoDisparo = 0;
            baseCabeceoDisparo = constrain(anguloCabeceo, 98, 172);
        }
        static unsigned long ultimaOledMs = 0;
        if (millis() - ultimaOledMs >= 250) {
            ultimaOledMs = millis();
            dibujarRadarOled();
        }
    }

    if (modoDisparo) {
        if (millis() - inicioDisparo > 2200) {
            modoDisparo = false;
            noTone(PIN_BUZZER);
            digitalWrite(PIN_LASER, estadoLaser ? HIGH : LOW);
        } else if (millis() - ultimoPasoDisparo >= 65) {
            ultimoPasoDisparo = millis();
            // Ráfaga en frecuencias de resonancia sin mover servos
            int fase = pasoDisparo % 4;
            if (fase == 0) {
                digitalWrite(PIN_LASER, HIGH);
                tone(PIN_BUZZER, 2400, 60);
            } else if (fase == 1) {
                tone(PIN_BUZZER, 1900, 60);
            } else if (fase == 2) {
                digitalWrite(PIN_LASER, LOW);
                tone(PIN_BUZZER, 1400, 55);
            } else {
                noTone(PIN_BUZZER);
            }
            pasoDisparo++;
        }
    }

    if (modoFiesta) {
        inicioFiesta = millis();
        pasoFiesta = 0;
        baseCabeceoFiesta = constrain(anguloCabeceo, 25, 155);
    } else {
        noTone(PIN_BUZZER);
    }
    server.send(200, "application/json", jsonEstado());
}

void handleDisparo() {
    modoDisparo = true;
    inicioDisparo = millis();
    ultimoPasoDisparo = 0;
    pasoDisparo = 0;
    server.send(200, "application/json", jsonEstado());
}

void handleReset() {
    server.send(200, "application/json", "{\"ok\":true}");
    delay(200);
    ESP.restart();
}

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handleStatus() { server.send(200, "application/json", jsonEstado()); }

void handleSetServo() {
    if (server.hasArg("cabeceo") && server.hasArg("rotacion")) {
        modoRectangulo = false;
        moverCabeceoHW(server.arg("cabeceo").toInt());
        moverRotacionHW(server.arg("rotacion").toInt());
    }
    server.send(200, "application/json", jsonEstado());
}

void handleMoveNorm() {
    if (server.hasArg("u") && server.hasArg("v")) {
        modoRectangulo = false;
        moverNormalizado(server.arg("u").toFloat(), server.arg("v").toFloat());
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
        // Area segura actualizada sin sobreescribir OLED
        oledNecesitaActualizar = true;
        iniciarBuzzer(1);
    }
    server.send(200, "application/json", jsonEstado());
}

void handleNotify() {
    ultimoNotifyMs = millis();
    if (server.hasArg("text")) {
        textoOledPrincipal = server.arg("text");
        oledNecesitaActualizar = true;
    }
    if (server.hasArg("beep")) {
        String b = server.arg("beep");
        if (b == "confirm") iniciarBuzzer(1);
        else if (b == "detect") iniciarBuzzer(2);
    }
    server.send(200, "application/json", jsonEstado());
}

void handleWifiConfig() {
    if (server.hasArg("ssid") && server.hasArg("pass")) {
        wifiSsidPrimary = server.arg("ssid");
        wifiPassPrimary = server.arg("pass");
        prefs.begin("trak_wifi", false);
        prefs.putString("ssid", wifiSsidPrimary);
        prefs.putString("pass", wifiPassPrimary);
        prefs.end();
        textoOledPrincipal = "WiFi: " + wifiSsidPrimary;
        oledNecesitaActualizar = true;
        iniciarBuzzer(1);
        server.send(200, "application/json", "{\"saved\":true,\"ssid\":\"" + wifiSsidPrimary + "\"}");
        delay(200);
        WiFi.disconnect();
        WiFi.begin(wifiSsidPrimary.c_str(), wifiPassPrimary.c_str());
        return;
    }
    server.send(400, "application/json", "{\"error\":\"Falta ssid o pass\"}");
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
    ledPuntero.begin();
    ledPuntero.setBrightness(255);
    pinMode(PIN_LASER, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    actualizarLaser(true);

    led48.begin();
    led38.begin();
    led21.begin();

    // Inicializar I2C y OLED 0.96" en GPIO 8 (SDA) y GPIO 9 (SCL)
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledDisponible = true;
    } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        oledDisponible = true;
    }

    // Configurar Buzzer LEDC en Canal 2 (GPIO 7)
    ledcSetup(CANAL_BUZZER, 2000, 8);
    ledcAttachPin(PIN_BUZZER, CANAL_BUZZER);
    ledcWriteTone(CANAL_BUZZER, 0);

    cargarCalibracion();

    // Configurar Servos LEDC
    ledcSetup(CANAL_CABECEO, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_CABECEO, CANAL_CABECEO);

    ledcSetup(CANAL_ROTACION, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_ROTACION, CANAL_ROTACION);

    moverNormalizado(0.5f, 0.5f);
    ponerColorLed(0, 180, 255);

    // Wi-Fi por defecto: Hotspot del celular ("rafa") guardado en NVS
    prefs.begin("trak_wifi", true);
    wifiSsidPrimary = prefs.getString("ssid", "rafa");
    wifiPassPrimary = prefs.getString("pass", "12345678");
    prefs.end();

    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    WiFi.softAP(AP_SSID, AP_PASS);
    wifiMulti.addAP(wifiSsidPrimary.c_str(), wifiPassPrimary.c_str());
    wifiMulti.run();

    // Servidor Web
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/servo", HTTP_POST, handleSetServo);
    server.on("/api/fiesta", HTTP_POST, handleFiesta);
    server.on("/api/disparo", HTTP_POST, handleDisparo);
    server.on("/api/reset", HTTP_POST, handleReset);
    server.on("/api/move_norm", HTTP_POST, handleMoveNorm);
    server.on("/api/center", HTTP_POST, handleCenter);
    server.on("/api/safe_area", HTTP_POST, handleSafeArea);
    server.on("/api/notify", HTTP_POST, handleNotify);
    server.on("/api/wifi_config", HTTP_POST, handleWifiConfig);
    server.on("/api/laser/toggle", HTTP_POST, handleToggleLaser);
    server.on("/api/rect/toggle", HTTP_POST, handleToggleRect);
    server.begin();

    MDNS.begin("esp32-laser");
    ArduinoOTA.setHostname("esp32-laser");
    ArduinoOTA.begin();

    dibujarOled();
    iniciarBuzzer(1); // Pitido corto de arranque listo
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    actualizarBuzzer();
    actualizarLedGradienteRespiracion();

    if (WiFi.status() != WL_CONNECTED && millis() - ultimoCheckWifi > 10000) {
        ultimoCheckWifi = millis();
        wifiMulti.run();
    }

    actualizarServosSuaves();

    // Lectura ultrasónica cada 120ms
    if (millis() - ultimoPing >= 120) {
        ultimoPing = millis();
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);
        long dur = pulseIn(PIN_ECHO, HIGH, 15000);
        distanciaCM = (dur > 0) ? (int)(dur * 0.034f / 2.0f) : 999;

        if (distanciaCM > 1 && distanciaCM <= 10 && !modoDisparo) {
            modoDisparo = true;
            inicioDisparo = millis();
            ultimoPasoDisparo = 0;
            pasoDisparo = 0;
            baseCabeceoDisparo = constrain(anguloCabeceo, 98, 172);
        }
        static unsigned long ultimaOledMs = 0;
        if (millis() - ultimaOledMs >= 250) {
            ultimaOledMs = millis();
            dibujarRadarOled();
        }
    }

    if (modoDisparo) {
        if (millis() - inicioDisparo > 2200) {
            modoDisparo = false;
            noTone(PIN_BUZZER);
            digitalWrite(PIN_LASER, estadoLaser ? HIGH : LOW);
        } else if (millis() - ultimoPasoDisparo >= 65) {
            ultimoPasoDisparo = millis();
            // Ráfaga en frecuencias de resonancia sin mover servos
            int fase = pasoDisparo % 4;
            if (fase == 0) {
                digitalWrite(PIN_LASER, HIGH);
                tone(PIN_BUZZER, 2400, 60);
            } else if (fase == 1) {
                tone(PIN_BUZZER, 1900, 60);
            } else if (fase == 2) {
                digitalWrite(PIN_LASER, LOW);
                tone(PIN_BUZZER, 1400, 55);
            } else {
                noTone(PIN_BUZZER);
            }
            pasoDisparo++;
        }
    }

    if (modoFiesta) {
        if (millis() - inicioFiesta > 5000) {
            modoFiesta = false;
            noTone(PIN_BUZZER);
            moverCabeceoHW(baseCabeceoFiesta);
        } else if (millis() - ultimoPasoFiesta >= 180) {
            ultimoPasoFiesta = millis();
            const int notas[] = {523, 659, 784, 1046, 784, 659, 880, 1046};
            tone(PIN_BUZZER, notas[pasoFiesta % 8], 140);
            int offset = (pasoFiesta % 2 == 0) ? 12 : -12;
            moverCabeceoHW(baseCabeceoFiesta + offset);
            pasoFiesta++;
        }
    }

    if (oledNecesitaActualizar) {
        oledNecesitaActualizar = false;
    }

    // Botón físico deshabilitado para evitar activación accidental de modoRectangulo
    /*
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
    */

    if (modoRectangulo) {
        if (millis() - ultimoPasoRect >= 25) {
            ultimoPasoRect = millis();
            pasoPerimetro += 0.02f;
            if (pasoPerimetro >= 1.0f) {
                pasoPerimetro = 0.0f;
                ladoActual = (ladoActual % 4) + 1;
            }

            switch (ladoActual) {
                case 1: moverNormalizado(pasoPerimetro, 0.0f); break;
                case 2: moverNormalizado(1.0f, pasoPerimetro); break;
                case 3: moverNormalizado(1.0f - pasoPerimetro, 1.0f); break;
                case 4: moverNormalizado(0.0f, 1.0f - pasoPerimetro); break;
            }
        }
    }
}
