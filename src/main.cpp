#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <ImuDrv.hpp>
#include <SD_MMC.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

const char* AP_SSID = "ESP32-GPS";
const char* AP_PASS = "12345678";

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
#define GPS_BAUD 57600

#define LCD_MOSI 45
#define LCD_SCLK 40
#define LCD_CS 42
#define LCD_DC 41
#define LCD_RST 39
#define LCD_BL 46
#define LCD_WIDTH 172
#define LCD_HEIGHT 320

#define IMU_SDA 48
#define IMU_SCL 47

#define SD_MOUNT "/sdcard"

#define BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_NOTIFY_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_COMMAND_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
Arduino_DataBus *displayBus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(displayBus, LCD_RST, 1, true, LCD_WIDTH, LCD_HEIGHT, 34, 0, 34, 0);
SensorQMI8658 imu;
WebServer server(80);
Preferences preferences;

bool haveFix = false;
double lat = 0.0;
double lng = 0.0;
double altitude = 0.0;
int satellites = 0;
double hdop = 0.0;
unsigned long lastFixMs = 0;

bool imuReady = false;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

bool displayTestActive = false;
unsigned long displayTestUntil = 0;

BLEServer* bleServer = nullptr;
BLECharacteristic* bleNotify = nullptr;
bool bleConnected = false;

void startDisplayTest();
void calibrateImuZero();
void resetImuCalibration();

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
  if (!imuReady) return;

  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 0;
  const int targetSamples = 100;

  while (samples < targetSamples) {
    if (imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::GYRO))) {
      GyroscopeData gyro;
      imu.readGyro(gyro);

      sumX += gyro.dps.x;
      sumY += gyro.dps.y;
      sumZ += gyro.dps.z;
      samples++;
    }

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
  if (!imuReady) return;

  if (imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::ACCEL))) {
    AccelerometerData accel;
    imu.readAccel(accel);

    ax = accel.mps2.x;
    ay = accel.mps2.y;
    az = accel.mps2.z;
  }

  if (imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::GYRO))) {
    GyroscopeData gyro;
    imu.readGyro(gyro);

    gx = gyro.dps.x - gyroOffsetX;
    gy = gyro.dps.y - gyroOffsetY;
    gz = gyro.dps.z - gyroOffsetZ;
  }
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
  doc["ready"] = imuReady;
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

String mapsJson() {
  JsonDocument doc;
  JsonArray maps = doc["maps"].to<JsonArray>();

  File root = SD_MMC.open("/maps");
  if (root) {
    File item = root.openNextFile();

    while (item) {
      if (item.isDirectory()) {
        maps.add(String(item.name()));
      }
      item.close();
      item = root.openNextFile();
    }

    root.close();
  }

  String out;
  serializeJson(doc, out);
  return out;
}

bool validMapName(const String& name) {
  if (name.length() == 0 || name.length() > 32) return false;

  for (unsigned int i = 0; i < name.length(); i++) {
    char c = name[i];
    if (!isalnum(c) && c != '_' && c != '-') return false;
  }

  return true;
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
select{padding:8px;border-radius:6px;background:#222;color:#eee;border:1px solid #444}
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

<div class="card">
<h2>Карта</h2>
<div class="small">По умолчанию используется онлайн-карта. При наличии microSD можно выбрать сохранённую карту.</div>
<select id="mapSelect">
<option value="online:osm">OpenStreetMap — онлайн</option>
<option value="online:esri">Esri Satellite — онлайн</option>
<option value="online:topo">OpenTopoMap — онлайн</option>
<option value="online:yandex">Яндекс Спутник — онлайн</option>
</select>
<div id="mapStatus" class="small">Загрузка списка офлайн-карт…</div>
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
let currentMap='';
let tileLayer=null;

function showTab(name){
document.getElementById('gpsTab').classList.toggle('hidden',name!=='gps');
document.getElementById('imuTab').classList.toggle('hidden',name!=='imu');
document.getElementById('tabGpsBtn').classList.toggle('active',name==='gps');
document.getElementById('tabImuBtn').classList.toggle('active',name==='imu');
}

const map=L.map('map').setView([59.9386,30.3141],11);

async function loadMaps(){
  const select=document.getElementById('mapSelect');
  try{
    const d=await (await fetch('/maps')).json();
    if(d.maps && d.maps.length){
      const group=document.createElement('optgroup');
      group.label='Офлайн-карты microSD';
      d.maps.forEach(name=>{
        const option=document.createElement('option');
        option.value='sd:'+name;
        option.textContent=name;
        group.appendChild(option);
      });
      select.appendChild(group);
      document.getElementById('mapStatus').textContent='Найдено офлайн-карт: '+d.maps.length;
    }else{
      document.getElementById('mapStatus').textContent='На microSD нет карт в каталоге /maps';
    }
  }catch(e){
    document.getElementById('mapStatus').textContent='microSD недоступна; онлайн-карты доступны при наличии интернета';
  }
  selectMap('online:osm');
}
function selectMap(value){
  if(tileLayer){map.removeLayer(tileLayer);tileLayer=null;}
  if(value==='online:osm'){
    tileLayer=L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'});
  }else if(value==='online:esri'){
    tileLayer=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:19,attribution:'Tiles &copy; Esri'});
  }else if(value==='online:topo'){
    tileLayer=L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',{maxZoom:17,attribution:'&copy; OpenTopoMap'});
  }else if(value==='online:yandex'){
    tileLayer=L.tileLayer('https://tiles.maps.yandex.net/tiles?l=sat&x={x}&y={y}&z={z}&scale=1&lang=ru_RU',{maxZoom:19,attribution:'&copy; Яндекс'});
  }else if(value.startsWith('sd:')){
    const name=value.substring(3);
    tileLayer=L.tileLayer('/tiles?map='+encodeURIComponent(name)+'&z={z}&x={x}&y={y}',{maxZoom:19,minZoom:3,attribution:'Offline map'});
  }
  if(tileLayer)tileLayer.addTo(map);
  document.getElementById('mapStatus').textContent=value.startsWith('sd:')?'Офлайн-карта: '+value.substring(3):'Онлайн-карта: '+value.substring(7).toUpperCase();
}
async function refreshGps(){
try{
const d=await (await fetch('/coords')).json();
if(!d.valid){document.getElementById('status').textContent='Нет GPS-фикса';return;}
const text=d.lat+', '+d.lng;
document.getElementById('coords').textContent=text;
document.getElementById('status').textContent='Спутников: '+d.sat+' | HDOP: '+d.hdop+' | обновлено '+Math.round(d.age_ms/1000)+' с назад';
}catch(e){}
}

async function refreshImu(){
try{
const d=await (await fetch('/imu')).json();
if(!d.ready){document.getElementById('imuStatus').textContent='QMI8658 не найден';document.getElementById('imuData').textContent='—';return;}
document.getElementById('imuStatus').textContent=d.calibrated?'Калибровка нуля активна':'Калибровка не задана';
document.getElementById('imuData').textContent='AX '+d.ax.toFixed(2)+' m/s2\nAY '+d.ay.toFixed(2)+' m/s2\nAZ '+d.az.toFixed(2)+' m/s2\n\nGX '+d.gx.toFixed(3)+' deg/s\nGY '+d.gy.toFixed(3)+' deg/s\nGZ '+d.gz.toFixed(3)+' deg/s';
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
const status=document.getElementById('bleStatus');
if(!navigator.bluetooth){
status.textContent='Web Bluetooth не поддерживается этим браузером. Используйте Chrome или Edge.';
return;
}
try{
const device=await navigator.bluetooth.requestDevice({filters:[{namePrefix:'ESP32-GNSS'}],optionalServices:['6e400001-b5a3-f393-e0a9-e50e24dcca9e']});
const server=await device.gatt.connect();
const service=await server.getPrimaryService('6e400001-b5a3-f393-e0a9-e50e24dcca9e');
const characteristic=await service.getCharacteristic('6e400002-b5a3-f393-e0a9-e50e24dcca9e');
await characteristic.startNotifications();
characteristic.addEventListener('characteristicvaluechanged',e=>{
const value=new TextDecoder().decode(e.target.value);
status.textContent='BLE: '+value;
});
status.textContent='Bluetooth подключён';
}catch(e){
status.textContent='Bluetooth: '+e.message;
}
}

document.getElementById('mapSelect').addEventListener('change',e=>selectMap(e.target.value));
setInterval(()=>{if(!document.getElementById('gpsTab').classList.contains('hidden'))refreshGps();},2000);
setInterval(()=>{if(!document.getElementById('imuTab').classList.contains('hidden'))refreshImu();},200);
loadMaps();
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

void handleMaps() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", mapsJson());
}

void handleTile() {
  if (!server.hasArg("map") || !server.hasArg("z") || !server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  String mapName = server.arg("map");
  String z = server.arg("z");
  String x = server.arg("x");
  String y = server.arg("y");

  if (!validMapName(mapName)) {
    server.send(400, "text/plain", "Invalid map");
    return;
  }

  String path = String(SD_MOUNT) + "/maps/" + mapName + "/" + z + "/" + x + "/" + y + ".png";

  if (!SD_MMC.exists(path)) {
    server.send(404, "text/plain", "Tile not found");
    return;
  }

  File tile = SD_MMC.open(path);
  if (!tile) {
    server.send(500, "text/plain", "Cannot open tile");
    return;
  }

  server.sendHeader("Cache-Control", "max-age=86400");
  server.streamFile(tile, "image/png");
  tile.close();
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

void drawDisplayTest() {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(25, 40);
  gfx->println("DISPLAY");
  gfx->println("TEST");

  gfx->fillRect(10, 150, 152, 35, COLOR_RED);
  gfx->fillRect(10, 195, 152, 35, COLOR_GREEN);
  gfx->fillRect(10, 240, 152, 35, COLOR_BLUE);

  gfx->setTextSize(1);
  gfx->setCursor(10, 290);
  gfx->print("Seconds left: ");
  gfx->println((displayTestUntil - millis()) / 1000 + 1);

  gfx->drawRect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, COLOR_WHITE);
}

void updateDisplay() {
  if (displayTestActive) {
    drawDisplayTest();
    return;
  }

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
    gfx->println();
    gfx->print("BLE: ");
    gfx->println(bleConnected ? "ON" : "OFF");
    gfx->println();
    gfx->print("IMU: ");
    gfx->println(imuReady ? "OK" : "NOT FOUND");
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

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);

  preferences.begin("imu", false);
  gyroOffsetX = preferences.getFloat("gx", 0);
  gyroOffsetY = preferences.getFloat("gy", 0);
  gyroOffsetZ = preferences.getFloat("gz", 0);

  if (imu.begin(Wire, QMI8658_H_SLAVE_ADDRESS, IMU_SDA, IMU_SCL)) {
    imu.configAccel(AccelFullScaleRange::FS_8G, 250.0f, SensorQMI8658::LpfMode::MODE_0);
    imu.configGyro(GyroFullScaleRange::FS_1000_DPS, 250.0f, SensorQMI8658::LpfMode::MODE_0);
    imu.enableAccel();
    imu.enableGyro();
    imuReady = true;
    Serial.println("QMI8658 ready");
  } else {
    Serial.println("QMI8658 not found");
  }

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  SD_MMC.setPins(14, 15, 16, 18, 17, 21); // CLK, CMD, D0, D1, D2, D3
  if (SD_MMC.begin(SD_MOUNT, true)) {
    Serial.println("SD card mounted");
  } else {
    Serial.println("SD card not mounted");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/coords", HTTP_GET, handleCoords);
  server.on("/imu", HTTP_GET, handleImu);
  server.on("/maps", HTTP_GET, handleMaps);
  server.on("/tiles", HTTP_GET, handleTile);
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
  if (millis() - lastImu > 20) {
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
