#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Arduino_GFX_Library.h>

const char* AP_SSID = "ESP32-GPS";
const char* AP_PASS = "12345678";

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
#define GPS_BAUD 9600

#define LCD_MOSI 45
#define LCD_SCLK 40
#define LCD_CS 42
#define LCD_DC 41
#define LCD_RST 39
#define LCD_BL 46
#define LCD_WIDTH 172
#define LCD_HEIGHT 320

#define COLOR_BLACK 0x000000
#define COLOR_WHITE 0xFFFFFF

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
Arduino_DataBus *displayBus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(displayBus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 34, 0, 34, 0);
WebServer server(80);

bool haveFix = false;
double lat = 0.0;
double lng = 0.0;
double altitude = 0.0;
int satellites = 0;
double hdop = 0.0;
unsigned long lastFixMs = 0;

String coordsJson() {
  JsonDocument doc;
  doc["valid"] = haveFix;
  doc["lat"] = serialized(String(lat, 7));
  doc["lng"] = serialized(String(lng, 7));
  doc["alt"] = altitude;
  doc["sat"] = satellites;
  doc["hdop"] = hdop;
  doc["age_ms"] = haveFix ? (long)(millis() - lastFixMs) : -1;

  String out;
  serializeJson(doc, out);
  return out;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", R"HTML(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 GNSS</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
body{margin:0;padding:12px;background:#111;color:#eee;font-family:system-ui,sans-serif}
.card{background:#1c1c1c;border-radius:12px;padding:14px;margin-bottom:12px}
.coords{font-family:monospace;font-size:18px;word-break:break-all;margin:10px 0}
button{padding:10px 14px;border:0;border-radius:8px;background:#2f81f7;color:#fff;font-size:15px}
#map{height:55vh;border-radius:12px}
.small{color:#aaa;font-size:13px}
</style>
</head>
<body>
<div class="card">
<h2>Координаты ESP32</h2>
<div id="status" class="small">Ожидание данных…</div>
<div id="coords" class="coords">—</div>
<button onclick="copyCoords()">Скопировать координаты</button>
</div>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const map=L.map('map').setView([59.9386,30.3141],11);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
let marker=null;
async function refresh(){
try{
const r=await fetch('/coords');
const d=await r.json();
if(!d.valid){document.getElementById('status').textContent='Нет GPS-фикса';return;}
const text=d.lat+', '+d.lng;
document.getElementById('coords').textContent=text;
document.getElementById('status').textContent='Спутников: '+d.sat+' | HDOP: '+d.hdop+' | обновлено '+Math.round(d.age_ms/1000)+' с назад';
if(!marker){marker=L.marker([d.lat,d.lng]).addTo(map);map.setView([d.lat,d.lng],16);}else{marker.setLatLng([d.lat,d.lng]);map.panTo([d.lat,d.lng]);}
}catch(e){}
}
function copyCoords(){
const text=document.getElementById('coords').textContent;
if(text!=='—')navigator.clipboard.writeText(text);
}
setInterval(refresh,2000);
refresh();
</script>
</body>
</html>
)HTML");
}

void handleCoords() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", coordsJson());
}

void updateDisplay() {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 20);

  if (!haveFix) {
    gfx->println("GPS:");
    gfx->println("no fix");

    gfx->setTextSize(1);
    gfx->setCursor(10, 100);
    gfx->println("Wi-Fi:");
    gfx->println(AP_SSID);
    gfx->println(WiFi.softAPIP().toString());
    return;
  }

  gfx->setTextSize(2);
  gfx->print("LAT\n");
  gfx->println(lat, 6);
  gfx->print("LON\n");
  gfx->println(lng, 6);

  gfx->setTextSize(1);
  gfx->setCursor(10, 250);
  gfx->print("SAT: ");
  gfx->println(satellites);
  gfx->print("ALT: ");
  gfx->print(altitude, 0);
  gfx->println(" m");
}

void setup() {
  Serial.begin(115200);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  if (!gfx->begin()) {
    Serial.println("ST7789 initialization failed");
  } else {
    Serial.println("ST7789 initialized");
  }

  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(10, 30);
  gfx->println("ESP32-S3");
  gfx->println("Starting...");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/coords", HTTP_GET, handleCoords);
  server.begin();

  Serial.println();
  Serial.print("Wi-Fi: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  while (gpsSerial.available()) {
    char c = gpsSerial.read();

    if (gps.encode(c)) {
      if (gps.location.isValid() && gps.location.isUpdated()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        haveFix = true;
        lastFixMs = millis();
      }

      if (gps.altitude.isValid()) {
        altitude = gps.altitude.meters();
      }

      if (gps.satellites.isValid()) {
        satellites = gps.satellites.value();
      }

      if (gps.hdop.isValid()) {
        hdop = gps.hdop.hdop();
      }
    }
  }

  static unsigned long lastUi = 0;
  if (millis() - lastUi > 500) {
    lastUi = millis();
    updateDisplay();
  }
}
