#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const char* AP_SSID = "ESP32-GPS";
const char* AP_PASS = "12345678";

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C
#define MPU_ADDR 0x68

#define BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_NOTIFY_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_COMMAND_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_MPU6050 mpu;
WebServer server(80);
Preferences preferences;

bool haveFix = false;
double lat = 0.0;
double lng = 0.0;
double altitude = 0.0;
int satellites = 0;
double hdop = 0.0;
unsigned long lastFixMs = 0;

bool mpuReady = false;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

bool displayTestActive = false;
unsigned long displayTestUntil = 0;

BLEServer* bleServer = nullptr;
BLECharacteristic* bleNotify = nullptr;
bool bleConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    Serial.println("BLE connected");
  }

  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    Serial.println("BLE disconnected");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = String(characteristic->getValue().c_str());
    value.trim();

    if (value == "DISPLAY_TEST") {
      startDisplayTest();
    } else if (value == "IMU_CALIBRATE") {
      calibrateImuZero();
    } else if (value == "IMU_RESET") {
      resetImuCalibration();
    }
  }
};

void startDisplayTest() {
  displayTestActive = true;
  displayTestUntil = millis() + 5000;
  Serial.println("Display test started");
}

void calibrateImuZero() {
  if (!mpuReady) return;

  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(5);
  }

  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;

  preferences.putFloat("gx", gyroOffsetX);
  preferences.putFloat("gy", gyroOffsetY);
  preferences.putFloat("gz", gyroOffsetZ);

  Serial.println("IMU zero calibrated");
}

void resetImuCalibration() {
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;

  preferences.putFloat("gx", 0);
  preferences.putFloat("gy", 0);
  preferences.putFloat("gz", 0);

  Serial.println("IMU calibration reset");
}

void readImu() {
  if (!mpuReady) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x - gyroOffsetX;
  gy = g.gyro.y - gyroOffsetY;
  gz = g.gyro.z - gyroOffsetZ;
}

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

String imuJson() {
  JsonDocument doc;
  doc["ready"] = mpuReady;
  doc["ax"] = ax;
  doc["ay"] = ay;
  doc["az"] = az;
  doc["gx"] = gx;
  doc["gy"] = gy;
  doc["gz"] = gz;
  doc["calibrated"] = gyroOffsetX != 0 || gyroOffsetY != 0 || gyroOffsetZ != 0;

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
<title>ESP32 GNSS + IMU</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
body{margin:0;padding:12px;background:#111;color:#eee;font-family:system-ui,sans-serif}
.tabs{display:flex;gap:8px;margin-bottom:12px}
.tab{padding:10px 16px;border-radius:8px;background:#333;color:#eee;border:0;font-size:15px}
.tab.active{background:#2f81f7;color:#fff}
.card{background:#1c1c1c;border-radius:12px;padding:14px;margin-bottom:12px}
.coords,.imu{font-family:monospace;font-size:17px;word-break:break-all;white-space:pre-wrap}
button{padding:10px 14px;border:0;border-radius:8px;background:#2f81f7;color:#fff;font-size:15px;margin:4px 4px 0 0}
button.gray{background:#555}
#map{height:50vh;border-radius:12px}
.small{color:#aaa;font-size:13px}
.hidden{display:none}
</style>
</head>
<body>
<div class="tabs">
<button id="tabGpsBtn" class="tab active" onclick="showTab('gps')">GPS</button>
<button id="tabImuBtn" class="tab" onclick="showTab('imu')">IMU</button>
</div>

<div id="gpsTab">
<div class="card">
<h2>Координаты ESP32</h2>
<div id="status" class="small">Ожидание данных…</div>
<div id="coords" class="coords">—</div>
<button onclick="copyCoords()">Скопировать координаты</button>
<button class="gray" onclick="testDisplay()">Тест экрана</button>
<button class="gray" onclick="connectBle()">Подключить Bluetooth</button>
<div id="bleStatus" class="small"></div>
</div>
<div id="map"></div>
</div>

<div id="imuTab" class="hidden">
<div class="card">
<h2>Акселерометр и гироскоп</h2>
<div id="imuStatus" class="small">Ожидание данных…</div>
<div id="imuData" class="imu">—</div>
<button onclick="calibrateZero()">Откалибровать 0</button>
<button class="gray" onclick="resetCalibration()">Сбросить калибровку</button>
</div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
function showTab(name){
document.getElementById('gpsTab').classList.toggle('hidden',name!=='gps');
document.getElementById('imuTab').classList.toggle('hidden',name!=='imu');
document.getElementById('tabGpsBtn').classList.toggle('active',name==='gps');
document.getElementById('tabImuBtn').classList.toggle('active',name==='imu');
}

const map=L.map('map').setView([55.7558,37.6173],12);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
let marker=null;

async function refreshGps(){
try{
const d=await (await fetch('/coords')).json();
if(!d.valid){document.getElementById('status').textContent='Нет GPS-фикса';return;}
const text=d.lat+', '+d.lng;
document.getElementById('coords').textContent=text;
document.getElementById('status').textContent='Спутников: '+d.sat+' | HDOP: '+d.hdop+' | обновлено '+Math.round(d.age_ms/1000)+' с назад';
if(!marker){marker=L.marker([d.lat,d.lng]).addTo(map);map.setView([d.lat,d.lng],16);}else{marker.setLatLng([d.lat,d.lng]);map.panTo([d.lat,d.lng]);}
}catch(e){}
}

async function refreshImu(){
try{
const d=await (await fetch('/imu')).json();
if(!d.ready){document.getElementById('imuStatus').textContent='MPU6050 не найден';document.getElementById('imuData').textContent='—';return;}
document.getElementById('imuStatus').textContent=d.calibrated?'Калибровка нуля активна':'Калибровка не задана';
document.getElementById('imuData').textContent='AX '+d.ax.toFixed(2)+' m/s2\nAY '+d.ay.toFixed(2)+' m/s2\nAZ '+d.az.toFixed(2)+' m/s2\n\nGX '+d.gx.toFixed(3)+' rad/s\nGY '+d.gy.toFixed(3)+' rad/s\nGZ '+d.gz.toFixed(3)+' rad/s';
}catch(e){}
}

async function testDisplay(){
await fetch('/display/test',{method:'POST'});
}

async function calibrateZero(){
await fetch('/imu/calibrate',{method:'POST'});
}

async function resetCalibration(){
await fetch('/imu/reset',{method:'POST'});
}

function copyCoords(){
const text=document.getElementById('coords').textContent;
if(text!=='—')navigator.clipboard.writeText(text);
}

async function connectBle(){
try{
const device=await navigator.bluetooth.requestDevice({filters:[{namePrefix:'ESP32-GNSS'}],optionalServices:['6e400001-b5a3-f393-e0a9-e50e24dcca9e']});
const server=await device.gatt.connect();
const service=await server.getPrimaryService('6e400001-b5a3-f393-e0a9-e50e24dcca9e');
const characteristic=await service.getCharacteristic('6e400002-b5a3-f393-e0a9-e50e24dcca9e');
await characteristic.startNotifications();
characteristic.addEventListener('characteristicvaluechanged',e=>{
const value=new TextDecoder().decode(e.target.value);
document.getElementById('bleStatus').textContent='BLE: '+value;
});
document.getElementById('bleStatus').textContent='Bluetooth подключён';
}catch(e){
document.getElementById('bleStatus').textContent='Bluetooth: '+e.message;
}
}

setInterval(()=>{if(!document.getElementById('gpsTab').classList.contains('hidden'))refreshGps();},2000);
setInterval(()=>{if(!document.getElementById('imuTab').classList.contains('hidden'))refreshImu();},200);
refreshGps();
</script>
</body>
</html>
)HTML");
}

void handleCoords() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", coordsJson());
}

void handleImu() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", imuJson());
}

void handleDisplayTest() {
  startDisplayTest();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrate() {
  calibrateImuZero();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleResetCalibration() {
  resetImuCalibration();
  server.send(200, "application/json", "{\"ok\":true}");
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (displayTestActive) {
    display.setTextSize(2);
    display.println("DISPLAY");
    display.println("TEST");
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("Seconds left: ");
    display.println((displayTestUntil - millis()) / 1000 + 1);
    display.drawRect(0, 0, 127, 63, SSD1306_WHITE);
    display.display();
    return;
  }

  if (!haveFix) {
    display.println("GPS: нет фикса");
    display.println();
    display.println("Wi-Fi:");
    display.println(AP_SSID);
    display.println(WiFi.softAPIP().toString());
    display.println();
    display.print("BLE: ");
    display.println(bleConnected ? "ON" : "OFF");
    display.display();
    return;
  }

  display.print("SAT: ");
  display.println(satellites);
  display.print("HDOP: ");
  display.println(hdop, 1);

  display.setCursor(0, 26);
  display.print("LAT ");
  display.println(lat, 6);
  display.print("LON ");
  display.println(lng, 6);

  display.setCursor(0, 54);
  display.print("ALT ");
  display.print(altitude, 0);
  display.print(" m");

  display.display();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED SSD1306 not found");
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32 GNSS + IMU");
  display.println("Starting...");
  display.display();

  preferences.begin("imu", false);
  gyroOffsetX = preferences.getFloat("gx", 0);
  gyroOffsetY = preferences.getFloat("gy", 0);
  gyroOffsetZ = preferences.getFloat("gz", 0);

  if (mpu.begin(MPU_ADDR, &Wire)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuReady = true;
    Serial.println("MPU6050 ready");
  } else {
    Serial.println("MPU6050 not found");
  }

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/coords", HTTP_GET, handleCoords);
  server.on("/imu", HTTP_GET, handleImu);
  server.on("/display/test", HTTP_POST, handleDisplayTest);
  server.on("/imu/calibrate", HTTP_POST, handleCalibrate);
  server.on("/imu/reset", HTTP_POST, handleResetCalibration);
  server.begin();

  BLEDevice::init("ESP32-GNSS");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);
  bleNotify = service->createCharacteristic(
    BLE_NOTIFY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleNotify->addDescriptor(new BLE2902());

  BLECharacteristic* command = service->createCharacteristic(
    BLE_COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  command->setCallbacks(new CommandCallbacks());

  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->start();

  Serial.println();
  Serial.print("Wi-Fi: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("BLE: ESP32-GNSS");
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

  static unsigned long lastImu = 0;
  if (millis() - lastImu > 100) {
    lastImu = millis();
    readImu();
  }

  static unsigned long lastBle = 0;
  if (bleConnected && millis() - lastBle > 1000) {
    lastBle = millis();

    String payload = coordsJson();
    payload += "|";
    payload += imuJson();
    bleNotify->setValue((uint8_t*)payload.c_str(), payload.length());
    bleNotify->notify();
  }

  if (displayTestActive && millis() > displayTestUntil) {
    displayTestActive = false;
  }

  static unsigned long lastUi = 0;
  if (millis() - lastUi > 200) {
    lastUi = millis();
    updateDisplay();
  }
}
