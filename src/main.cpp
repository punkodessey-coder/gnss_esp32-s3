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

#define GPS_RX_PIN 44
#define GPS_TX_PIN 43
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
HardwareSerial simulatorSerial(1); // NMEA test output: TX GPIO8, RX GPIO9 (unused)
int gpsBaud = GPS_BAUD;
String savedPointsJson = "[]";
String lastUbxFrameHex = "";
String ubxLastEvent = "Ожидание UBX";
uint8_t ubxFrame[256];
size_t ubxFramePos = 0, ubxFrameLength = 0;

TinyGPSPlus gps;
bool nmeaSimulatorEnabled = false;
unsigned long lastSimulatorSentenceMs = 0;
Arduino_DataBus *displayBus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(displayBus, LCD_RST, 1, true, LCD_WIDTH, LCD_HEIGHT, 34, 0, 34, 0);
SensorQMI8658 imu;
WebServer server(80);
Preferences preferences;

bool haveFix = false;
double lat = 0.0;
double lng = 0.0;
double altitude = 0.0;
int satellites = 0; // satellites used by a valid GGA fix
int satellitesVisible = 0; // satellites reported by GSV
double hdop = 0.0;
unsigned long lastFixMs = 0;
unsigned long lastGgaMs = 0;
unsigned long lastGsvMs = 0;
uint8_t fixTimeoutSeconds = 7;
unsigned long simulatorStartMs = 0;
double simulatorLat = 54.922700;
double simulatorLng = 43.344800;
unsigned long uartLastByteMs = 0, uartWindowStartMs = 0, uartBytesWindow = 0;
unsigned long uartBytesPerSecond = 0, uartTotalBytes = 0, uartInvalidSentences = 0;
String uartCurrentLine, uartLastLine, uartRecentLines;
bool uartLineOverflow = false;

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
void sendNmeaSimulatorFrame();
void handleSimulatorStart();
void updateSimulatorPosition();
void handleSimulatorStop();
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

String uartJson() {
  JsonDocument doc;
  doc["baud"] = gpsBaud;
  doc["rx_pin"] = GPS_RX_PIN;
  doc["tx_pin"] = GPS_TX_PIN;
  doc["bytes_total"] = uartTotalBytes;
  doc["bytes_per_second"] = uartBytesPerSecond;
  doc["last_byte_age_ms"] = uartLastByteMs ? (long)(millis() - uartLastByteMs) : -1;
  doc["invalid_sentences"] = uartInvalidSentences;
  doc["checksum_passed"] = gps.passedChecksum();
  doc["checksum_failed"] = gps.failedChecksum();
  doc["chars_processed"] = gps.charsProcessed();
  doc["last_sentence"] = uartLastLine;
  doc["recent"] = uartRecentLines;
  doc["line_overflow"] = uartLineOverflow;
  String out; serializeJson(doc, out); return out;
}

String coordsJson() {
  JsonDocument doc;
  doc["valid"] = haveFix;
  doc["lat"] = serialized(String(lat, 7));
  doc["lng"] = serialized(String(lng, 7));
  doc["alt"] = altitude;
  doc["sat"] = satellites;
  doc["sat_visible"] = satellitesVisible;
  doc["fix_timeout_s"] = fixTimeoutSeconds;
  doc["hdop"] = hdop;
  doc["age_ms"] = haveFix ? (long)(millis() - lastFixMs) : -1;

  String out;
  serializeJson(doc, out);
  return out;
}


String bytesToHex(const uint8_t* data, size_t len) {
  String out;
  for (size_t i=0;i<len;i++) { if(i) out += ' '; char b[3]; snprintf(b,sizeof(b),"%02X",data[i]); out += b; }
  return out;
}
bool parseHexPayload(const String& hex, uint8_t* out, size_t& len) {
  len=0; int high=-1;
  for(size_t i=0;i<hex.length();i++) {
    char c=hex[i]; if(c==' '||c==','||c==':'||c=='\n'||c=='\r') continue;
    int v=(c>='0'&&c<='9')?c-'0':(c>='A'&&c<='F')?c-'A'+10:(c>='a'&&c<='f')?c-'a'+10:-1;
    if(v<0)return false;
    if(high<0) high=v; else { if(len>=220)return false; out[len++]=(high<<4)|v; high=-1; }
  }
  return high<0;
}
void sendUbx(uint8_t cls,uint8_t id,const uint8_t* payload,size_t len) {
  uint8_t a=0,b=0;
  gpsSerial.write(0xB5); gpsSerial.write(0x62); gpsSerial.write(cls); gpsSerial.write(id);
  uint8_t l1=len&0xFF,l2=(len>>8)&0xFF;
  gpsSerial.write(l1);gpsSerial.write(l2);
  a+=cls;b+=a;a+=id;b+=a;a+=l1;b+=a;a+=l2;b+=a;
  for(size_t i=0;i<len;i++){gpsSerial.write(payload[i]);a+=payload[i];b+=a;}
  gpsSerial.write(a);gpsSerial.write(b);
}
void processUbxByte(uint8_t c) {
  if(ubxFramePos==0 && c!=0xB5)return;
  if(ubxFramePos==1 && c!=0x62){ubxFramePos=(c==0xB5)?1:0;return;}
  if(ubxFramePos<sizeof(ubxFrame)) ubxFrame[ubxFramePos++]=c; else {ubxFramePos=0;return;}
  if(ubxFramePos==6){ubxFrameLength=(size_t)ubxFrame[4]|((size_t)ubxFrame[5]<<8);if(ubxFrameLength>248){ubxFramePos=0;return;}}
  if(ubxFramePos>=8 && ubxFrameLength && ubxFramePos==ubxFrameLength+8){
    uint8_t a=0,b=0;for(size_t i=2;i<ubxFramePos-2;i++){a+=ubxFrame[i];b+=a;}
    if(a==ubxFrame[ubxFramePos-2]&&b==ubxFrame[ubxFramePos-1]){
      lastUbxFrameHex=bytesToHex(ubxFrame,ubxFramePos);
      uint8_t cls=ubxFrame[2],id=ubxFrame[3];
      if(cls==0x05&&(id==0x01||id==0x00))ubxLastEvent=(id==1?"UBX ACK":"UBX NAK");
      else ubxLastEvent="Ответ UBX "+String(cls,HEX)+":"+String(id,HEX);
    }
    ubxFramePos=0;ubxFrameLength=0;
  }
}

String nmeaField(const String& line, uint8_t fieldIndex) {
  int start = 0;
  uint8_t index = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ',' || line[i] == '*') {
      if (index == fieldIndex) return line.substring(start, i);
      index++;
      start = i + 1;
    }
  }
  return "";
}

bool validNmeaChecksum(const String& line) {
  int star = line.indexOf('*');
  if (star < 0) return true; // Some receivers can be configured without checksums.
  uint8_t checksum = 0;
  for (int i = 1; i < star; i++) checksum ^= (uint8_t)line[i];
  if (star + 2 >= (int)line.length()) return false;
  char expected[3] = { line[star + 1], line[star + 2], 0 };
  return checksum == (uint8_t)strtoul(expected, nullptr, 16);
}

void processNmeaLine(const String& line) {
  if (line.length() < 7 || line[0] != '$' || !validNmeaChecksum(line)) return;
  String type = line.substring(3, 6);
  if (type == "GGA") {
    const int fixQuality = nmeaField(line, 6).toInt();
    const int used = nmeaField(line, 7).toInt();
    satellites = (fixQuality > 0) ? constrain(used, 0, 99) : 0;
    lastGgaMs = millis();
  } else if (type == "GSV") {
    const int visible = nmeaField(line, 3).toInt();
    satellitesVisible = constrain(visible, 0, 99);
    lastGsvMs = millis();
  }
}

String savedPointsResponse() {
  String value = preferences.getString("points", "[]");
  if (value.length() == 0) value = "[]";
  return value;
}

String imuJson() {
  JsonDocument doc;
  doc["ready"] = imuReady;
  doc["ax"] = ax; doc["ay"] = ay; doc["az"] = az;
  doc["gx"] = gx; doc["gy"] = gy; doc["gz"] = gz;
  doc["accel_magnitude"] = sqrtf(ax*ax + ay*ay + az*az);
  doc["gyro_magnitude"] = sqrtf(gx*gx + gy*gy + gz*gz);
  doc["roll_deg"] = atan2f(ay, az) * 57.2957795f;
  doc["pitch_deg"] = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
  doc["gyro_offset_x"] = gyroOffsetX; doc["gyro_offset_y"] = gyroOffsetY; doc["gyro_offset_z"] = gyroOffsetZ;
  doc["calibrated"] = gyroOffsetX != 0 || gyroOffsetY != 0 || gyroOffsetZ != 0;
  doc["sample_period_ms"] = 20;
  doc["i2c_address"] = 0x6A;
  doc["accel_range_g"] = 8;
  doc["gyro_range_dps"] = 1000;
  doc["sample_rate_hz"] = 250;
  String out; serializeJson(doc, out); return out;
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
.card{background:#1c1c1c;border-radius:14px;padding:16px;margin-bottom:12px}
.metric{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid #333;color:#aaa}
.metric strong{color:#fff;font:600 18px ui-monospace,monospace}
.gps-card .coords{margin-top:14px;color:#8ab4ff}
.coords,.imu{font-family:monospace;font-size:15px;word-break:break-word;white-space:pre-wrap;line-height:1.5}\n#uartStats,#uartLast,#uartRecent{font-size:13px;line-height:1.55;white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word}\n#uartRecent{max-height:52vh;overflow:auto;background:#151515;border-radius:8px;padding:10px}
button{padding:10px 14px;border:0;border-radius:8px;background:#2f81f7;color:#fff;font-size:15px;margin:4px 4px 0 0}
button.gray{background:#555}
#map{height:50vh;border-radius:12px}
.small{color:#aaa;font-size:13px}
.hidden{display:none}
select,input,textarea{padding:8px;border-radius:6px;background:#222;color:#eee;border:1px solid #444;max-width:100%;box-sizing:border-box}
.ubx-layout{display:grid;grid-template-columns:minmax(150px,220px) 1fr;gap:14px;margin-top:14px}
.ubx-nav{display:flex;flex-direction:column;gap:5px}.ubx-nav-item{text-align:left;background:#292929;margin:0}.ubx-nav-item.active{background:#245ca5}
.ubx-panel{min-width:0;background:#171717;border-radius:10px;padding:14px}
.ubx-field{display:grid;grid-template-columns:minmax(130px,1fr) minmax(120px,1fr);gap:12px;align-items:center;padding:9px 0;border-bottom:1px solid #333}
.ubx-field label{color:#bbb}.ubx-field input,.ubx-field select{width:100%}.ubx-checks{display:flex;flex-wrap:wrap;gap:14px}.ubx-checks label{display:flex;gap:6px;align-items:center}
.ubx-actions{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0}
@media(max-width:650px){.ubx-layout{grid-template-columns:1fr}.ubx-nav{display:grid;grid-template-columns:repeat(2,minmax(0,1fr))}.ubx-nav-item{font-size:12px;padding:8px}.ubx-field{grid-template-columns:1fr}}

</style>
</head>
<body>
<div class="tabs">
<button id="tabGpsBtn" class="tab active" onclick="showTab('gps')">GPS</button>
<button id="tabImuBtn" class="tab" onclick="showTab('imu')">IMU</button>
<button id="tabUartBtn" class="tab" onclick="showTab('uart')">UART</button>
<button id="tabUbxBtn" class="tab" onclick="showTab('ubx')">u-blox</button>
<button id="tabPointsBtn" class="tab" onclick="showTab('points')">Точки</button>
</div>

<div id="gpsTab">
<div class="card gps-card">
<h2>GNSS — текущее положение</h2>
<div id="status" class="small">Ожидание данных…</div>
<div class="metric"><span>Широта</span><strong id="latValue">0.000000</strong></div>
<div class="metric"><span>Долгота</span><strong id="lngValue">0.000000</strong></div>
<div class="metric"><span>Используется в решении</span><strong id="satValue">0</strong></div>
<div class="metric"><span>Видно по GSV</span><strong id="satVisibleValue">0</strong></div>
<div class="coords" id="coords">0.000000, 0.000000</div>
<button onclick="copyCoords()">Скопировать координаты</button>
<button class="gray" onclick="saveCurrentPoint()">Сохранить текущую точку</button>

<button class="gray" onclick="testDisplay()">Тест экрана</button>
<button class="gray" onclick="connectBle()">Подключить Bluetooth</button>
<div class="card" style="margin-top:12px">
<h3>Тестовый NMEA-генератор</h3>
<div class="small">Имитирует движение по замкнутому маршруту в центре Сарова: один полный круг за 60 секунд. NMEA выходит через TX GPIO8 (57600 бод); для физического loopback подключите GPIO8 → GPS RX GPIO44 и общий GND.</div>
<button onclick="startNmeaSimulator()">Включить имитацию</button>
<button class="gray" onclick="stopNmeaSimulator()">Выключить</button>
<div id="simulatorStatus" class="small">Имитатор выключен</div>
</div>
<div id="bleStatus" class="small"></div>
</div>

<div class="card">
<h2>Карта</h2>
<div class="small">Онлайн-карты требуют интернет на телефоне/ноутбуке. Яндекс Гибрид подключается через официальный JavaScript API и требует API-ключ.</div>
<select id="mapSelect">
<option value="online:osm">OpenStreetMap — онлайн</option>
<option value="online:esri">Esri Satellite — онлайн</option>
<option value="online:topo">OpenTopoMap — онлайн</option>
<option value="online:yandex">Яндекс — гибрид</option>
</select>
<div class="ubx-field"><label for="yandexApiKey">API-ключ Яндекс.Карт</label><input id="yandexApiKey" type="password" autocomplete="off" placeholder="Вставьте API-ключ"></div>
<button class="gray" onclick="saveYandexKey()">Сохранить ключ</button>
<div class="ubx-field"><label for="fixTimeoutSeconds">Скрывать точку после потери координат (сек.)</label><input id="fixTimeoutSeconds" type="number" min="1" max="60" value="7"></div>
<button class="gray" onclick="saveFixTimeout()">Сохранить тайм-аут</button>
<div id="mapStatus" class="small">Загрузка списка офлайн-карт…</div>
</div>

<div id="map"></div>
</div>

<div id="uartTab" class="hidden">
<div class="card"><h2>Монитор UART GPS</h2>
<div id="uartStatus" class="small">Ожидание данных UART…</div>
<label for="uartBaud">Скорость UART GPS</label>
<select id="uartBaud" onchange="changeUartBaud(this.value)">
<option>9600</option><option>19200</option><option>38400</option><option selected>57600</option><option>115200</option><option>230400</option>
</select>
<pre id="uartStats" class="coords">—</pre>
<h3>Последняя строка NMEA</h3><pre id="uartLast" class="coords">—</pre>
<h3>Последние строки NMEA</h3><pre id="uartRecent" class="coords">—</pre>
</div></div>
<div id="ubxTab" class="hidden">
<div class="card"><h2>u-blox — Configuration View</h2>
<div class="small">Configuration View для u-blox 6 / NEO-6M. Поля и длины UBX-CFG пакетов соответствуют протоколу u-blox 6. GNSS: NEO-6M — GPS-приёмник; CFG-GNSS у этого поколения отсутствует, поэтому доступен статус GPS и настройка SBAS. «Считать» отправляет Poll, «Записать» — корректный Set.</div>
<div class="ubx-layout">
<nav id="ubxNav" class="ubx-nav">
<button class="ubx-nav-item active" data-page="gnss" onclick="selectUbxPage('gnss')">GNSS — Systems</button>
<button class="ubx-nav-item" data-page="msg" onclick="selectUbxPage('msg')">MSG — Messages</button>
<button class="ubx-nav-item" data-page="nav5" onclick="selectUbxPage('nav5')">NAV5 — Navigation</button>
<button class="ubx-nav-item" data-page="prt" onclick="selectUbxPage('prt')">PRT — Ports</button>
<button class="ubx-nav-item" data-page="rate" onclick="selectUbxPage('rate')">RATE — Rates</button>
<button class="ubx-nav-item" data-page="tp5" onclick="selectUbxPage('tp5')">TP5 — Timepulse</button>
<button class="ubx-nav-item" data-page="sbas" onclick="selectUbxPage('sbas')">SBAS — Augmentation</button>
<button class="ubx-nav-item" data-page="nmea" onclick="selectUbxPage('nmea')">NMEA — Protocol</button>
<button class="ubx-nav-item" data-page="cfg" onclick="selectUbxPage('cfg')">CFG — Save/Load</button>
</nav>
<section class="ubx-panel">
<h3 id="ubxPageTitle">MSG — Message rates</h3>
<div id="ubxFields"></div>
<div class="ubx-actions"><button onclick="pollUbxPage()">Считать (Poll)</button><button onclick="writeUbxPage()">Записать (Send)</button><button class="gray" onclick="pollUbxPage()">Обновить</button></div>
<div id="ubxStatus" class="small">Подключите TX GPS к RX ESP32 и RX GPS к TX ESP32 для двусторонней настройки.</div>
<details><summary>Последний ответ приёмника (диагностика)</summary><pre id="ubxFrame" class="coords">—</pre></details>
</section></div></div></div>
<div id="pointsTab" class="hidden"><div class="card"><h2>Сохранённые точки</h2><div class="small">Название и координаты сохранённых местоположений</div><div id="savedPointsList" class="small">Загрузка…</div></div></div>
<div id="imuTab" class="hidden">
<div class="card"><h2>QMI8658 — состояние и идентификация</h2><div id="imuStatus" class="small">Ожидание данных…</div><pre id="imuData" class="imu">—</pre></div>
<div class="card"><h3>Калибровка гироскопа</h3><div class="small">Положите плату неподвижно на ровную поверхность во время калибровки.</div><button onclick="calibrateZero()">Откалибровать смещение</button><button class="gray" onclick="resetCalibration()">Сбросить калибровку</button></div>
<div class="card"><h3>Доступные измерения</h3><div class="small">QMI8658 — 6-осевой IMU: три оси акселерометра и три оси гироскопа. Магнитометра и встроенного компаса на этой плате нет. Roll/Pitch вычисляются по вектору ускорения; yaw без магнитометра/внешней привязки не определяется.</div></div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
let currentMap='';
let tileLayer=null;
let map=null, gpsMarker=null, currentFix=false;
let currentLat=54.9227,currentLng=43.3448;
let yandexMap=null,yandexCircle=null,yandexApiKey='';

function showTab(name){
document.getElementById('gpsTab').classList.toggle('hidden',name!=='gps');
document.getElementById('imuTab').classList.toggle('hidden',name!=='imu');
document.getElementById('uartTab').classList.toggle('hidden',name!=='uart');
document.getElementById('ubxTab').classList.toggle('hidden',name!=='ubx');
document.getElementById('pointsTab').classList.toggle('hidden',name!=='points');
document.getElementById('tabGpsBtn').classList.toggle('active',name==='gps');
document.getElementById('tabImuBtn').classList.toggle('active',name==='imu');
document.getElementById('tabUartBtn').classList.toggle('active',name==='uart');
document.getElementById('tabUbxBtn').classList.toggle('active',name==='ubx');
document.getElementById('tabPointsBtn').classList.toggle('active',name==='points');
if(name==='gps'){
  if(map) setTimeout(()=>map.invalidateSize(),150);
  if(yandexMap) setTimeout(()=>yandexMap.container.fitToViewport(),150);
}
if(name==='ubx' && !document.getElementById('ubxFields').children.length) selectUbxPage('gnss');
}

function initMap(){
  if(typeof L==='undefined'){document.getElementById('mapStatus').textContent='Leaflet не загрузилась — проверьте доступ к интернету';return false;}
  if(!map) map=L.map('map').setView([54.9227,43.3448],15);
  return true;
}
async function loadYandexKey(){
  try{
    const d=await (await fetch('/maps/yandex-key')).json();
    yandexApiKey=d.key||'';
    if(yandexApiKey) document.getElementById('yandexApiKey').value=yandexApiKey;
  }catch(e){}
}
async function saveYandexKey(){
  const key=document.getElementById('yandexApiKey').value.trim();
  try{
    const r=await fetch('/maps/yandex-key',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key})});
    if(!r.ok) throw new Error('HTTP '+r.status);
    yandexApiKey=key;
    document.getElementById('mapStatus').textContent=key?'Ключ сохранён. Загружаю Яндекс Гибрид…':'Ключ удалён';
    if(key) await selectMap('online:yandex');
  }catch(e){document.getElementById('mapStatus').textContent='Не удалось сохранить ключ Яндекса: '+e.message;}
}
async function loadYandexApi(){
  if(window.ymaps) { await new Promise(resolve=>window.ymaps.ready(resolve)); return; }
  if(!window.__yandexMapsPromise){
    window.__yandexMapsPromise=new Promise((resolve,reject)=>{
      const script=document.createElement('script');
      script.src='https://api-maps.yandex.ru/2.1/?apikey='+encodeURIComponent(yandexApiKey)+'&lang=ru_RU';
      script.onload=()=>resolve();
      script.onerror=()=>reject(new Error('Не удалось загрузить JavaScript API Яндекса'));
      document.head.appendChild(script);
    });
  }
  await window.__yandexMapsPromise;
  await new Promise(resolve=>window.ymaps.ready(resolve));
}
async function loadMaps(){
  const select=document.getElementById('mapSelect');
  try{
    const d=await (await fetch('/maps')).json();
    if(d.maps && d.maps.length){
      const group=document.createElement('optgroup'); group.label='Офлайн-карты microSD';
      d.maps.forEach(name=>{const option=document.createElement('option');option.value='sd:'+name;option.textContent=name;group.appendChild(option);});
      select.appendChild(group);
      document.getElementById('mapStatus').textContent='Найдено офлайн-карт: '+d.maps.length;
    }else document.getElementById('mapStatus').textContent='На microSD нет карт в каталоге /maps';
  }catch(e){document.getElementById('mapStatus').textContent='microSD недоступна; онлайн-карты требуют интернет на клиентском устройстве';}
  await loadYandexKey();
  selectMap('online:osm');
}
async function selectMap(value){
  currentMap=value;
  if(tileLayer && map){map.removeLayer(tileLayer);tileLayer=null;}
  if(yandexMap){yandexMap.destroy();yandexMap=null;yandexCircle=null;}
  if(value==='online:yandex'){
    if(!yandexApiKey){document.getElementById('mapStatus').textContent='Для Яндекс Гибрид нужен API-ключ. Введите его выше и нажмите «Сохранить ключ».';return;}
    if(map){map.remove();map=null;gpsMarker=null;}
    try{
      await loadYandexApi();
      yandexMap=new ymaps.Map('map',{center:currentFix?[currentLat,currentLng]:[54.9227,43.3448],zoom:16,controls:['zoomControl']});
      yandexMap.setType('yandex#hybrid');
      if(currentFix) updateYandexPoint(currentLat,currentLng);
      document.getElementById('mapStatus').textContent='Яндекс Гибрид загружен';
    }catch(e){document.getElementById('mapStatus').textContent='Ошибка Яндекс.Карт: '+e.message;}
    return;
  }
  if(map){map.remove();map=null;gpsMarker=null;}
  if(!initMap())return;
  if(value==='online:osm'){
    tileLayer=L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'});
  }else if(value==='online:esri'){
    tileLayer=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:19,attribution:'Tiles &copy; Esri'});
  }else if(value==='online:topo'){
    tileLayer=L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',{maxZoom:17,attribution:'&copy; OpenTopoMap'});
  }else if(value.startsWith('sd:')){
    const name=value.substring(3);
    tileLayer=L.tileLayer('/tiles?map='+encodeURIComponent(name)+'&z={z}&x={x}&y={y}',{maxZoom:19,minZoom:3,attribution:'Offline map'});
  }
  if(tileLayer)tileLayer.addTo(map);
  if(currentFix) updateLeafletPoint(currentLat,currentLng);
  document.getElementById('mapStatus').textContent=value.startsWith('sd:')?'Офлайн-карта: '+value.substring(3):'Онлайн-карта: '+value.substring(7).toUpperCase();
}
function updateLeafletPoint(lat,lng){
  if(!map)return;
  if(!gpsMarker){
    gpsMarker=L.circleMarker([lat,lng],{radius:6,color:'#ffffff',weight:2,fillColor:'#1677ff',fillOpacity:1}).addTo(map);
    map.setView([lat,lng],16);
  }else gpsMarker.setLatLng([lat,lng]);
}
function updateYandexPoint(lat,lng){
  if(!yandexMap)return;
  const coords=[lat,lng];
  if(!yandexCircle){
    yandexCircle=new ymaps.Circle([coords,5],{}, {fillColor:'#1677ffCC',strokeColor:'#ffffff',strokeWidth:2});
    yandexMap.geoObjects.add(yandexCircle);
    yandexMap.setCenter(coords,16);
  }else yandexCircle.geometry.setCoordinates(coords);
}
function removeCurrentMapPoint(){
  if(gpsMarker && map){map.removeLayer(gpsMarker);gpsMarker=null;}
  if(yandexCircle && yandexMap){yandexMap.geoObjects.remove(yandexCircle);yandexCircle=null;}
}
async function saveFixTimeout(){
  const input=document.getElementById('fixTimeoutSeconds');
  const seconds=Number(input.value);
  if(!Number.isInteger(seconds)||seconds<1||seconds>60){alert('Укажите от 1 до 60 секунд');return;}
  try{
    const r=await fetch('/settings/fix-timeout?seconds='+seconds,{method:'POST'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    document.getElementById('mapStatus').textContent='Тайм-аут потери координат: '+seconds+' с';
  }catch(e){document.getElementById('mapStatus').textContent='Не удалось сохранить тайм-аут: '+e.message;}
}
async function refreshSavedPoints(){async function refreshSavedPoints(){
 try{
  const points=await (await fetch('/points')).json();
  const list=document.getElementById('savedPointsList');
  list.innerHTML='';
  points.forEach(p=>{
   const row=document.createElement('div'); row.className='point-row';
   const title=document.createElement('strong'); title.textContent=p.name;
   const coords=document.createElement('div'); coords.className='small'; coords.textContent=Number(p.lat).toFixed(6)+', '+Number(p.lng).toFixed(6);
   const edit=document.createElement('button'); edit.className='gray'; edit.textContent='Переименовать'; edit.onclick=()=>editPoint(p.id,p.name);
   const del=document.createElement('button'); del.className='gray'; del.textContent='Удалить'; del.onclick=()=>deletePoint(p.id);
   row.append(title,coords,edit,del); list.appendChild(row);
  });
  if(!points.length) list.textContent='Сохранённых точек пока нет';
 }catch(e){document.getElementById('savedPointsList').textContent='Ошибка загрузки точек';}
}
async function saveCurrentPoint(){
 if(!currentFix){alert('Нет актуального GPS-фикса');return;}
 const name=prompt('Название точки:','Новая точка'); if(!name||!name.trim())return;
 const r=await fetch('/points/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:name.trim()})});
 if(r.ok)refreshSavedPoints();else alert('Не удалось сохранить точку');
}
async function editPoint(id,oldName){
 const name=prompt('Новое название точки:',oldName); if(!name||!name.trim())return;
 await fetch('/points/update',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id,name:name.trim()})}); refreshSavedPoints();
}
async function deletePoint(id){
 if(!confirm('Удалить сохранённую точку?'))return;
 await fetch('/points/delete?id='+encodeURIComponent(id),{method:'POST'}); refreshSavedPoints();
}

async function refreshGps(){
try{
  const d=await (await fetch('/coords?ts='+Date.now())).json();
  const lat=d.valid?Number(d.lat):0;
  const lng=d.valid?Number(d.lng):0;
  document.getElementById('latValue').textContent=lat.toFixed(6);
  document.getElementById('lngValue').textContent=lng.toFixed(6);
  document.getElementById('satValue').textContent=String(d.sat??0);
  document.getElementById('satVisibleValue').textContent=String(d.sat_visible??0);
  document.getElementById('coords').textContent=lat.toFixed(6)+', '+lng.toFixed(6);
  currentFix=!!d.valid;
  if(d.fix_timeout_s && document.activeElement!==document.getElementById('fixTimeoutSeconds')){
    document.getElementById('fixTimeoutSeconds').value=d.fix_timeout_s;
  }
  if(!d.valid){
    document.getElementById('status').textContent='Нет актуального фикса';
    removeCurrentMapPoint();
    return;
  }
  currentLat=lat; currentLng=lng;
  document.getElementById('status').textContent='Используется: '+(d.sat??0)+' | видно: '+(d.sat_visible??0)+' | HDOP: '+d.hdop+' | обновлено '+Math.round(d.age_ms/1000)+' с назад';
  if(yandexMap) updateYandexPoint(lat,lng);
  else if(map) updateLeafletPoint(lat,lng);
}catch(e){}
}
async function changeUartBaud(value){async function changeUartBaud(value){
 const r=await fetch('/uart/baud?baud='+encodeURIComponent(value),{method:'POST'});
 document.getElementById('uartStatus').textContent=r.ok?'Скорость UART изменена на '+value+' бод':'Не удалось изменить скорость UART';
 refreshUart();
}

let ubxPage='gnss',ubxLastFrame='';
const ubxPageInfo={
  gnss:{title:'GNSS — GPS / SBAS configuration',id:0x16},
  msg:{title:'MSG — Message output rate',id:0x01},
  nav5:{title:'NAV5 — Navigation engine settings',id:0x24},
  prt:{title:'PRT — UART port configuration',id:0x00},
  rate:{title:'RATE — Measurement and navigation rate',id:0x08},
  tp5:{title:'TP5 — Timepulse configuration',id:0x31},
  sbas:{title:'SBAS — Satellite Based Augmentation',id:0x16},
  nmea:{title:'NMEA — Protocol settings',id:0x17},
  cfg:{title:'CFG — Save/Load receiver configuration',id:0x09}
};
function field(id,label,value,type='number',extra=''){return '<div class="ubx-field"><label for="'+id+'">'+label+'</label><input id="'+id+'" type="'+type+'" value="'+value+'" '+extra+'></div>';}
function selectField(id,label,value,options){return '<div class="ubx-field"><label for="'+id+'">'+label+'</label><select id="'+id+'">'+options.map(o=>'<option value="'+o[0]+'" '+(String(value)===String(o[0])?'selected':'')+'>'+o[1]+'</option>').join('')+'</select></div>';}
function checkField(id,label,checked){return '<div class="ubx-field"><label>'+label+'</label><label><input id="'+id+'" type="checkbox" '+(checked?'checked':'')+'> Включено</label></div>';}
function selectUbxPage(page){
  ubxPage=page;
  document.querySelectorAll('.ubx-nav-item').forEach(b=>b.classList.toggle('active',b.dataset.page===page));
  document.getElementById('ubxPageTitle').textContent=ubxPageInfo[page].title;
  const f=document.getElementById('ubxFields');
  if(page==='gnss'){
    f.innerHTML='<div class="ubx-field"><label>Приёмник</label><strong>u-blox NEO-6M</strong></div>'+
      '<div class="ubx-field"><label>Основная GNSS</label><strong>GPS — поддерживается аппаратно</strong></div>'+
      '<div class="ubx-field"><label>Другие созвездия</label><strong>GLONASS / Galileo / BeiDou — не поддерживаются NEO-6M</strong></div>'+
      '<p class="small">У u-blox 6 нет команды CFG-GNSS. Ниже доступны реальные параметры CFG-SBAS (EGNOS/WAAS/MSAS).</p>'+
      checkField('gnssSbasEnable','SBAS включён',true)+
      checkField('gnssSbasTest','Разрешить SBAS Test Mode',false)+
      checkField('gnssSbasRange','Использовать SBAS как источник измерений',true)+
      checkField('gnssSbasDiff','Применять дифференциальные поправки',true)+
      checkField('gnssSbasIntegrity','Использовать контроль целостности',true)+
      field('gnssSbasMax','Каналов SBAS (0–3)',3)+
      field('gnssSbasScan','Маска PRN SBAS (0 = автопоиск)',0,'number','min="0" max="4294967295"');
  }
  if(page==='msg'){
    f.innerHTML=selectField('msgClass','Класс сообщения','F0',[['F0','NMEA standard (F0)'],['F1','NMEA proprietary (F1)'],['01','NAV (01)'],['02','RXM (02)'],['0A','MON (0A)']])+
      selectField('msgId','Сообщение','00',[['00','GGA — Fix data'],['01','GLL — Geographic position'],['02','GSA — DOP and active satellites'],['03','GSV — Satellites in view'],['04','RMC — Recommended minimum'],['05','VTG — Course and speed'],['08','ZDA — Time and date']])+
      selectField('msgI2c','Частота DDC/I²C','0',[['0','0 — выключено'],['1','1 — каждое решение'],['2','2 — каждое второе'],['5','5 — каждое пятое']])+
      selectField('msgUart1','Частота UART1','1',[['0','0 — выключено'],['1','1 — каждое решение'],['2','2 — каждое второе'],['5','5 — каждое пятое']])+
      selectField('msgUart2','Частота UART2','0',[['0','0 — выключено'],['1','1 — каждое решение'],['2','2 — каждое второе'],['5','5 — каждое пятое']])+
      selectField('msgUsb','Частота USB','0',[['0','0 — выключено'],['1','1 — каждое решение'],['2','2 — каждое второе'],['5','5 — каждое пятое']])+
      selectField('msgSpi','Частота SPI','0',[['0','0 — выключено'],['1','1 — каждое решение'],['2','2 — каждое второе'],['5','5 — каждое пятое']])+
      '<p class="small">CFG-MSG u-blox 6: Poll = 2 байта (класс, ID); Set = 8 байт (класс, ID, DDC, UART1, UART2, USB, SPI, reserved).</p>';
  }
  if(page==='nav5'){
    f.innerHTML=selectField('navDyn','Dynamic platform model',0,[[0,'Portable'],[2,'Stationary'],[3,'Pedestrian'],[4,'Automotive'],[5,'Sea'],[6,'Airborne <1g'],[7,'Airborne <2g'],[8,'Airborne <4g']])+
      selectField('navFixMode','Position fix mode',3,[[1,'2D only'],[2,'3D only'],[3,'Auto 2D/3D']])+
      field('navFixedAlt','Fixed altitude (m)',0,'number','step="0.01"')+
      field('navFixedAltVar','Fixed altitude variance (m²)',0,'number','step="0.0001"')+
      field('navMinElev','Minimum satellite elevation (°)',5)+
      field('navDrLimit','Dead reckoning limit (s)',0)+
      field('navPDop','Position DOP mask',0,'number','step="0.1"')+
      field('navTDop','Time DOP mask',0,'number','step="0.1"')+
      field('navPAcc','Position accuracy mask (m)',100)+
      field('navTAcc','Time accuracy mask (m)',300)+
      field('navStaticHold','Static hold threshold (cm/s)',0)+
      field('navDgpsTimeout','DGPS timeout (s)',0)+
      '<p class="small">CFG-NAV5: fixed altitude передаётся с масштабом 0,01 м; DOP — 0,1. DGPS timeout зависит от версии firmware.</p>';
  }
  if(page==='prt'){
    f.innerHTML=selectField('prtPort','Порт','1',[['0','DDC / I²C'],['1','UART1'],['2','UART2'],['3','USB'],['4','SPI']])+
      field('prtBaud','Baud rate',57600)+
      selectField('prtDataBits','Data bits',8,[[7,'7'],[8,'8']])+
      selectField('prtParity','Parity','none',[['none','None'],['even','Even'],['odd','Odd']])+
      selectField('prtStopBits','Stop bits',1,[[1,'1'],[2,'2']])+
      checkField('prtInUbx','Input protocol: UBX',true)+checkField('prtInNmea','Input protocol: NMEA',true)+
      checkField('prtInRtcm','Input protocol: RTCM',false)+checkField('prtOutUbx','Output protocol: UBX',false)+
      checkField('prtOutNmea','Output protocol: NMEA',true)+checkField('prtOutRtcm','Output protocol: RTCM',false)+
      '<p class="small">UART mode (8N1 по умолчанию) и маски протоколов формируются по полям CFG-PRT u-blox 6.</p>';
  }
  if(page==='rate'){
    f.innerHTML=field('rateMeas','Measurement period (ms)',1000)+
      field('rateNav','Navigation rate (cycles)',1,'number','disabled')+
      selectField('rateRef','Time reference',1,[[0,'UTC'],[1,'GPS']])+
      '<p class="small">Для u-blox 6 navigation rate фиксирован равным 1 и не изменяется. 1000 мс соответствует 1 Гц.</p>';
  }
  if(page==='sbas'){
    f.innerHTML=checkField('gnssSbasEnable','SBAS enabled',true)+
      checkField('gnssSbasTest','Test mode',false)+
      checkField('gnssSbasRange','Use SBAS ranging',true)+
      checkField('gnssSbasDiff','Use differential corrections',true)+
      checkField('gnssSbasIntegrity','Use integrity',true)+
      field('gnssSbasMax','Maximum SBAS channels (0–3)',3)+
      field('gnssSbasScan','SBAS scan mask (0 = automatic)',0,'number','min="0" max="4294967295"');
  }
  if(page==='nmea'){
    f.innerHTML=selectField('nmeaVersion','Версия NMEA',35,[[35,'2.3'],[33,'2.1']])+
      field('nmeaMaxSv','Максимум спутников в NMEA (0 = без ограничения)',16)+
      '<p class="small">Фильтры u-blox 6 CFG-NMEA. Отмеченный флаг отключает соответствующую фильтрацию (кроме SBAS, где флаг включает фильтрацию SBAS).</p>'+
      checkField('nmeaPosFilter','Выводить позицию даже без фикса',false)+
      checkField('nmeaMaskedFilter','Выводить позицию за пределами масок точности',false)+
      checkField('nmeaTimeFilter','Выводить время даже при недостоверном времени',false)+
      checkField('nmeaDateFilter','Выводить дату даже при недостоверной дате',false)+
      checkField('nmeaSbasFilter','Фильтровать SBAS из NMEA',false)+
      checkField('nmeaTrackFilter','Отключить фильтрацию курса (COG)',false)+
      checkField('nmeaCompat','Compatibility mode',false)+
      checkField('nmeaConsider','Consider mode',false);
  }
  if(page==='cfg'){
    const section=(prefix,label,defaults)=>'<h4>'+label+'</h4>'+
      checkField(prefix+'Port','Порты I/O',defaults)+
      checkField(prefix+'Messages','Конфигурация сообщений',defaults)+
      checkField(prefix+'Info','INF-сообщения',defaults)+
      checkField(prefix+'Navigation','Навигация (NAV/RATE/SBAS/NMEA)',defaults)+
      checkField(prefix+'Rxm','Receiver Manager (RXM)',defaults)+
      checkField(prefix+'Inventory','Remote Inventory',false)+
      checkField(prefix+'Antenna','Антенна',false);
    f.innerHTML='<p>CFG-CFG работает с разделами конфигурации, а не выбирает BBR/Flash. Сохранение выполняется во все доступные энергонезависимые носители приёмника.</p>'+
      section('cfgSave','Сохранить текущие параметры',true)+
      section('cfgClear','Сбросить разделы к заводским настройкам',false)+
      section('cfgLoad','Загрузить сохранённые параметры',false)+
      '<p class="small">Оставь Clear и Load пустыми для обычного сохранения. Не отмечай Clear без необходимости: это сбрасывает выбранные разделы.</p>';
  }
  if(page==='tp5'){
    f.innerHTML=selectField('tpIdx','Timepulse output',0,[[0,'TIMEPULSE'],[1,'TIMEPULSE2']])+
      field('tpAntDelay','Antenna cable delay (ns)',0)+field('tpRfDelay','RF group delay (ns)',0,'number','disabled')+
      field('tpFreq','Frequency (Hz) / period (µs)',1000000)+field('tpFreqLock','Frequency / period when GPS locked',1000000)+
      field('tpPulse','Pulse length (µs) / duty cycle',100000)+field('tpPulseLock','Pulse length / duty cycle when locked',100000)+
      field('tpUserDelay','User configurable delay (ns)',0)+
      checkField('tpActive','Timepulse active',false)+checkField('tpLockGps','Synchronize to GPS time',true)+
      checkField('tpLockedOther','Use locked frequency/pulse values',false)+checkField('tpIsFreq','Frequency mode (off = period)',true)+
      checkField('tpIsLength','Pulse length mode (off = duty cycle)',true)+checkField('tpAlign','Align pulse to top of second',false)+
      checkField('tpPolarity','Rising edge at top of second',true)+selectField('tpGrid','Time grid',1,[[0,'UTC'],[1,'GPS']])+
      '<p class="small">RF group delay — read-only. CFG-TP5 использует 32-байтовый payload; единицы и флаги соответствуют протоколу u-blox 6.</p>';
  }
}
function hexBytes(hex){return hex.trim().split(/\s+/).filter(Boolean).map(x=>parseInt(x,16));}
function ubxFramePayload(){const b=hexBytes(ubxLastFrame);if(b.length<8||b[0]!==181||b[1]!==98)return null;return {cls:b[2],id:b[3],p:b.slice(6,-2)};}
function u16(v){v=Math.round(Number(v)||0);return [v&255,(v>>8)&255];}
function u32(v){v=Math.round(Number(v)||0);return [v&255,(v>>8)&255,(v>>16)&255,(v>>24)&255];}
function readU16(p,o){return p[o]|(p[o+1]<<8);}
function readU32(p,o){return (p[o]|(p[o+1]<<8)|(p[o+2]<<16)|(p[o+3]<<24))>>>0;}
function payloadForPage(){
  const n=id=>Number(document.getElementById(id).value)||0;
  const ck=id=>document.getElementById(id).checked;
  if(ubxPage==='gnss'){
    const mode=(ck('gnssSbasEnable')?1:0)|(ck('gnssSbasTest')?2:0);
    const usage=(ck('gnssSbasRange')?1:0)|(ck('gnssSbasDiff')?2:0)|(ck('gnssSbasIntegrity')?4:0);
    const prn=Math.max(0,Math.min(0xFFFFFFFF,n('gnssSbasScan')))>>>0;
    return [mode,usage,Math.max(0,Math.min(3,n('gnssSbasMax'))),0,...u32(prn)];
  }
  if(ubxPage==='msg')return [parseInt(document.getElementById('msgClass').value,16),parseInt(document.getElementById('msgId').value,16),n('msgI2c'),n('msgUart1'),n('msgUart2'),n('msgUsb'),n('msgSpi'),0];
  if(ubxPage==='prt'){
    const port=n('prtPort'),baud=n('prtBaud');
    const dataBits=n('prtDataBits')===7?2:3;
    const parity=document.getElementById('prtParity').value;
    const parityBits=parity==='even'?0:(parity==='odd'?1:4);
    const stopBits=n('prtStopBits')===2?2:0;
    const mode=(1<<4)|(dataBits<<6)|(parityBits<<9)|(stopBits<<12);
    const inMask=(ck('prtInUbx')?1:0)|(ck('prtInNmea')?2:0)|(ck('prtInRtcm')?4:0);
    const outMask=(ck('prtOutUbx')?1:0)|(ck('prtOutNmea')?2:0)|(ck('prtOutRtcm')?4:0);
    return [port,0,0,0,...u32(mode),...u32(baud),...u16(inMask),...u16(outMask),0,0,0,0];
  }
  if(ubxPage==='rate')return [...u16(n('rateMeas')),...u16(1),...u16(n('rateRef'))];
  if(ubxPage==='nav5'){
    const mask=0x0FFF;
    const fixedAlt=Math.round(n('navFixedAlt')*100);
    const fixedAltVar=Math.max(0,Math.round(n('navFixedAltVar')*10000));
    const pDop=Math.max(0,Math.round(n('navPDop')*10));
    const tDop=Math.max(0,Math.round(n('navTDop')*10));
    return [...u16(mask),n('navDyn'),n('navFixMode'),...u32(fixedAlt),...u32(fixedAltVar),n('navMinElev')&255,n('navDrLimit')&255,...u16(pDop),...u16(tDop),...u16(n('navPAcc')),...u16(n('navTAcc')),n('navStaticHold')&255,n('navDgpsTimeout')&255,0,0,0,0,0,0,0,0,0,0,0,0];
  }
  if(ubxPage==='sbas'){
    const mode=(ck('gnssSbasEnable')?1:0)|(ck('gnssSbasTest')?2:0);
    const usage=(ck('gnssSbasRange')?1:0)|(ck('gnssSbasDiff')?2:0)|(ck('gnssSbasIntegrity')?4:0);
    const prn=Math.max(0,Math.min(0xFFFFFFFF,n('gnssSbasScan')))>>>0;
    return [mode,usage,Math.max(0,Math.min(3,n('gnssSbasMax'))),0,...u32(prn)];
  }
  if(ubxPage==='nmea'){
    const filter=(ck('nmeaPosFilter')?1:0)|(ck('nmeaMaskedFilter')?2:0)|
      (ck('nmeaTimeFilter')?4:0)|(ck('nmeaDateFilter')?8:0)|
      (ck('nmeaSbasFilter')?16:0)|(ck('nmeaTrackFilter')?32:0);
    const flags=(ck('nmeaCompat')?1:0)|(ck('nmeaConsider')?2:0);
    return [filter,Number(document.getElementById('nmeaVersion').value),n('nmeaMaxSv'),flags];
  }
  if(ubxPage==='cfg'){
    const mask=prefix=>{
      let value=0;
      const bits={Port:0,Messages:1,Info:2,Navigation:3,Rxm:4,Inventory:9,Antenna:10};
      Object.keys(bits).forEach(name=>{
        const e=document.getElementById(prefix+name);
        if(e&&e.checked)value|=(1<<bits[name]);
      });
      return value;
    };
    return [...u32(mask('cfgClear')), ...u32(mask('cfgSave')), ...u32(mask('cfgLoad'))];
  }
  if(ubxPage==='tp5'){
    const flags=(ck('tpActive')?1:0)|(ck('tpLockGps')?2:0)|(ck('tpLockedOther')?4:0)|(ck('tpIsFreq')?8:0)|(ck('tpIsLength')?16:0)|(ck('tpAlign')?32:0)|(ck('tpPolarity')?64:0)|((Number(document.getElementById('tpGrid').value)&0x0F)<<7);
    return [n('tpIdx'),0,0,0,...u16(n('tpAntDelay')),...u16(n('tpRfDelay')),...u32(n('tpFreq')),...u32(n('tpFreqLock')),...u32(n('tpPulse')),...u32(n('tpPulseLock')),...u32(n('tpUserDelay')),...u32(flags)];
  }
  return [];
}
async function sendUbxPacket(id,payload){
  const h=payload.map(v=>(v&255).toString(16).padStart(2,'0')).join(' ');
  return fetch('/ubx/send?cls=06&id='+id.toString(16).padStart(2,'0')+'&payload='+encodeURIComponent(h),{method:'POST'});
}
async function pollUbxPage(){
  if(ubxPage==='cfg'){
    document.getElementById('ubxStatus').textContent='CFG-CFG — командная операция, Poll для неё не предусмотрен. Используйте «Записать» для выполнения выбранных Clear/Save/Load.';
    return;
  }
  let q=[];
  if(ubxPage==='msg')q=[parseInt(document.getElementById('msgClass').value,16),parseInt(document.getElementById('msgId').value,16)];
  if(ubxPage==='prt')q=[Number(document.getElementById('prtPort').value)];
  if(ubxPage==='tp5')q=[Number(document.getElementById('tpIdx').value)];
  if(ubxPage==='sbas'||ubxPage==='cfg'||ubxPage==='nmea')q=[];
  const r=await sendUbxPacket(ubxPageInfo[ubxPage].id,q);
  document.getElementById('ubxStatus').textContent=r.ok?'Poll отправлен. Ожидаю ответ приёмника…':'Ошибка Poll';
  setTimeout(refreshUbxStatus,350);
}
async function writeUbxPage(){
  const p=payloadForPage(),r=await sendUbxPacket(ubxPageInfo[ubxPage].id,p);
  document.getElementById('ubxStatus').textContent=r.ok?'Параметры отправлены. Ожидаю ACK/NAK…':'Ошибка отправки';
  setTimeout(refreshUbxStatus,350);
}
function decodeUbxPage(frame){
  const p=frame.p;
  if(frame.cls!==6||frame.id!==ubxPageInfo[ubxPage].id)return;
  const set=(id,v)=>{const e=document.getElementById(id);if(e)e.value=v;};
  const check=(id,v)=>{const e=document.getElementById(id);if(e)e.checked=!!v;};
  if((ubxPage==='gnss'||ubxPage==='sbas')&&p.length>=8){
    check('gnssSbasEnable',p[0]&1);check('gnssSbasTest',p[0]&2);
    check('gnssSbasRange',p[1]&1);check('gnssSbasDiff',p[1]&2);check('gnssSbasIntegrity',p[1]&4);
    set('gnssSbasMax',p[2]);set('gnssSbasScan',readU32(p,4));
  }
  if(ubxPage==='msg'&&p.length>=8){set('msgI2c',p[3]);set('msgUart1',p[4]);set('msgUart2',p[5]);set('msgUsb',p[6]);set('msgSpi',p[7]);}
  if(ubxPage==='prt'&&p.length>=20){
    set('prtPort',p[0]);set('prtBaud',readU32(p,8));
    const mode=readU32(p,4),bits=(mode>>6)&3,parity=(mode>>9)&7,stop=(mode>>12)&3;
    set('prtDataBits',bits===2?7:8);set('prtParity',parity===0?'even':parity===1?'odd':'none');set('prtStopBits',stop===2?2:1);
    const im=readU16(p,12),om=readU16(p,14);
    check('prtInUbx',im&1);check('prtInNmea',im&2);check('prtInRtcm',im&4);
    check('prtOutUbx',om&1);check('prtOutNmea',om&2);check('prtOutRtcm',om&4);
  }
  if(ubxPage==='rate'&&p.length>=6){set('rateMeas',readU16(p,0));set('rateNav',readU16(p,2));set('rateRef',readU16(p,4));}
  if(ubxPage==='nav5'&&p.length>=36){
    set('navDyn',p[2]);set('navFixMode',p[3]);set('navFixedAlt',(readU32(p,4)|0)/100);
    set('navFixedAltVar',readU32(p,8)/10000);set('navMinElev',p[12]);set('navDrLimit',p[13]);
    set('navPDop',readU16(p,14)/10);set('navTDop',readU16(p,16)/10);
    set('navPAcc',readU16(p,18));set('navTAcc',readU16(p,20));set('navStaticHold',p[22]);set('navDgpsTimeout',p[23]);
  }
  if(ubxPage==='nmea'&&p.length>=4){
    const filter=p[0],flags=p[3];
    check('nmeaPosFilter',filter&1);check('nmeaMaskedFilter',filter&2);
    check('nmeaTimeFilter',filter&4);check('nmeaDateFilter',filter&8);
    check('nmeaSbasFilter',filter&16);check('nmeaTrackFilter',filter&32);
    set('nmeaVersion',p[1]);set('nmeaMaxSv',p[2]);
    check('nmeaCompat',flags&1);check('nmeaConsider',flags&2);
  }
  if(ubxPage==='tp5'&&p.length>=32){
    set('tpIdx',p[0]);set('tpAntDelay',(p[4]|p[5]<<8)<<16>>16);set('tpRfDelay',(p[6]|p[7]<<8)<<16>>16);
    set('tpFreq',readU32(p,8));set('tpFreqLock',readU32(p,12));set('tpPulse',readU32(p,16));set('tpPulseLock',readU32(p,20));
    set('tpUserDelay',readU32(p,24)|0);
    const flags=readU32(p,28);
    check('tpActive',flags&1);check('tpLockGps',flags&2);check('tpLockedOther',flags&4);
    check('tpIsFreq',flags&8);check('tpIsLength',flags&16);check('tpAlign',flags&32);check('tpPolarity',flags&64);
    set('tpGrid',(flags>>7)&0x0F);
  }
}
async function refreshUbxStatus(){
  try{
    const d=await (await fetch('/ubx/status?ts='+Date.now())).json();
    document.getElementById('ubxFrame').textContent=d.frame||'Ответ ещё не получен';
    if(d.frame&&d.frame!==ubxLastFrame){
      ubxLastFrame=d.frame;
      const fr=ubxFramePayload();
      if(fr){decodeUbxPage(fr);document.getElementById('ubxStatus').textContent='Получен UBX '+fr.cls.toString(16).toUpperCase()+':'+fr.id.toString(16).toUpperCase();}
    }else if(d.event)document.getElementById('ubxStatus').textContent=d.event;
  }catch(e){}
}
async function refreshUart(){async function refreshUart(){
 const status=document.getElementById('uartStatus');
 try{
  // Query token avoids stale responses and avoids Safari RequestInit/cache quirks.
  const apiUrl=window.location.origin+'/uart?ts='+Date.now();
  const response=await fetch(apiUrl);
  const body=await response.text();
  if(!response.ok) throw new Error('HTTP '+response.status+': '+body.slice(0,120));
  let d;
  try{d=JSON.parse(body);}catch(parseError){throw new Error('Некорректный JSON от /uart: '+body.slice(0,120));}
  if(!d || typeof d.baud!=='number' || typeof d.rx_pin!=='number' || typeof d.tx_pin!=='number') throw new Error('В ответе /uart отсутствуют обязательные поля');
  const age=d.last_byte_age_ms<0?'байтов ещё не было':(d.last_byte_age_ms<1500?'приём идёт':'последний байт '+(d.last_byte_age_ms/1000).toFixed(1)+' с назад');
  status.textContent=age+' | '+d.baud+' бод | RX GPIO'+d.rx_pin+' / TX GPIO'+d.tx_pin;
  document.getElementById('uartStats').textContent='Всего байтов: '+d.bytes_total+'\nСкорость: '+d.bytes_per_second+' байт/с\nChecksum OK: '+d.checksum_passed+'\nОшибок checksum: '+d.checksum_failed+'\nОбработано символов: '+d.chars_processed+'\nОшибок/переполнений строк: '+d.invalid_sentences;
  document.getElementById('uartLast').textContent=d.last_sentence||'Полная строка NMEA ещё не получена';
  document.getElementById('uartRecent').textContent=d.recent||'—';
 }catch(e){status.textContent='Ошибка UART API: '+(e && e.message ? e.message : String(e));}
}
async function refreshImu(){
try{
const d=await (await fetch('/imu')).json();
if(!d.ready){document.getElementById('imuStatus').textContent='QMI8658 не найден или не инициализирован';document.getElementById('imuData').textContent='—';return;}
document.getElementById('imuStatus').textContent='Датчик доступен | ACC: '+(d.accel_enabled?'ON':'OFF')+' | GYRO: '+(d.gyro_enabled?'ON':'OFF')+' | I2C 0x'+Number(d.i2c_address).toString(16).toUpperCase();
const f=(v,n=3)=>Number(v).toFixed(n);
document.getElementById('imuData').textContent=
'КОНФИГУРАЦИЯ\nI2C address: 0x'+Number(d.i2c_address).toString(16).toUpperCase()+'\nАкселерометр: ±'+d.accel_range_g+' g | Гироскоп: ±'+d.gyro_range_dps+' °/s\nЧастота настройки датчиков: '+d.sample_rate_hz+' Hz | Опрос: '+d.sample_period_ms+' ms'+
'\n\nАКСЕЛЕРОМЕТР (m/s²)\nX: '+f(d.ax)+' | Y: '+f(d.ay)+' | Z: '+f(d.az)+'\nМодуль ускорения: '+f(d.accel_magnitude)+' m/s²'+
'\n\nГИРОСКОП (°/s)\nX: '+f(d.gx)+' | Y: '+f(d.gy)+' | Z: '+f(d.gz)+'\nМодуль угловой скорости: '+f(d.gyro_magnitude)+' °/s'+
'\n\nРАСЧЁТНАЯ ОРИЕНТАЦИЯ\nRoll: '+f(d.roll_deg,2)+'° | Pitch: '+f(d.pitch_deg,2)+'°\nYaw: недоступен без магнитометра/внешней привязки'+
'\n\nКАЛИБРОВКА ГИРОСКОПА\nX: '+f(d.gyro_offset_x,4)+' | Y: '+f(d.gyro_offset_y,4)+' | Z: '+f(d.gyro_offset_z,4)+' °/s\nСтатус: '+(d.calibrated?'смещение сохранено':'смещение не задано')+
'\n\nОбновление опроса: '+d.sample_period_ms+' ms';
}catch(e){document.getElementById('imuStatus').textContent='Ошибка чтения IMU: '+e.message;}
}

async function startNmeaSimulator(){
 try{
  const r=await fetch('/simulator/start',{method:'POST'});
  const d=await r.json();
  document.getElementById('simulatorStatus').textContent=d.enabled?'Имитация включена: кольцевой маршрут в Сарове, 60 секунд на круг':'Не удалось включить имитатор';
 }catch(e){document.getElementById('simulatorStatus').textContent='Ошибка запуска имитатора';}
}
async function stopNmeaSimulator(){
 try{
  await fetch('/simulator/stop',{method:'POST'});
  document.getElementById('simulatorStatus').textContent='Имитатор выключен';
 }catch(e){document.getElementById('simulatorStatus').textContent='Ошибка выключения имитатора';}
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

async function copyCoords(){
 const text=document.getElementById('coords').textContent.trim();
 try{
  if(navigator.clipboard && window.isSecureContext){await navigator.clipboard.writeText(text);}
  else{
   const field=document.createElement('textarea');field.value=text;field.style.position='fixed';field.style.opacity='0';document.body.appendChild(field);field.focus();field.select();
   const ok=document.execCommand('copy');document.body.removeChild(field);if(!ok)throw new Error('copy command failed');
  }
  const button=document.querySelector('button[onclick="copyCoords()"]');if(button){const old=button.textContent;button.textContent='Скопировано';setTimeout(()=>button.textContent=old,1200);}
 }catch(e){prompt('Скопируйте координаты:',text);}
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
setInterval(()=>{if(!document.getElementById('gpsTab').classList.contains('hidden'))refreshGps();},1000);
setInterval(()=>{if(!document.getElementById('imuTab').classList.contains('hidden'))refreshImu();},200);
setInterval(()=>{if(!document.getElementById('uartTab').classList.contains('hidden'))refreshUart();},500);
setInterval(()=>{if(!document.getElementById('ubxTab').classList.contains('hidden'))refreshUbxStatus();},500);
refreshUart();
selectUbxPage('gnss');
loadMaps();
refreshGps();
refreshSavedPoints();
</script>
</body>
</html>
)HTML");
}

void handleSavedPoints() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", savedPointsResponse());
}

void handleSavePoint() {
  JsonDocument input;
  if (deserializeJson(input, server.arg("plain"))) { server.send(400, "text/plain", "Invalid JSON"); return; }
  String name = input["name"] | "";
  name.trim();
  if (!name.length() || name.length() > 48 || !haveFix) { server.send(400, "text/plain", "Valid name and current fix required"); return; }
  JsonDocument doc;
  deserializeJson(doc, savedPointsResponse());
  JsonArray points = doc.as<JsonArray>();
  JsonObject item = points.add<JsonObject>();
  item["id"] = String((uint32_t)esp_random(), HEX);
  item["name"] = name;
  item["lat"] = lat;
  item["lng"] = lng;
  String out; serializeJson(doc, out);
  preferences.putString("points", out);
  server.send(200, "application/json", out);
}

void handleUpdatePoint() {
  JsonDocument input;
  if (deserializeJson(input, server.arg("plain"))) { server.send(400, "text/plain", "Invalid JSON"); return; }
  String id = input["id"] | "";
  String name = input["name"] | "";
  name.trim();
  if (!id.length() || !name.length() || name.length() > 48) { server.send(400, "text/plain", "Invalid id/name"); return; }
  JsonDocument doc;
  deserializeJson(doc, savedPointsResponse());
  for (JsonObject item : doc.as<JsonArray>()) if (String((const char*)item["id"]) == id) item["name"] = name;
  String out; serializeJson(doc, out); preferences.putString("points", out);
  server.send(200, "application/json", out);
}

void handleDeletePoint() {
  String id = server.arg("id");
  JsonDocument source, target;
  deserializeJson(source, savedPointsResponse());
  JsonArray result = target.to<JsonArray>();
  for (JsonObject item : source.as<JsonArray>()) if (String((const char*)item["id"]) != id) result.add(item);
  String out; serializeJson(target, out); preferences.putString("points", out);
  server.send(200, "application/json", out);
}

void handleSetBaud() {
  int baud = server.arg("baud").toInt();
  const int allowed[] = {9600, 19200, 38400, 57600, 115200, 230400};
  bool ok = false;
  for (int value : allowed) if (value == baud) ok = true;
  if (!ok) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"Unsupported baud\"}"); return; }
  gpsSerial.updateBaudRate(baud);
  gpsBaud = baud;
  simulatorSerial.updateBaudRate(baud);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleUbxSend() {
  uint8_t cls=(uint8_t)strtoul(server.arg("cls").c_str(),nullptr,16);
  uint8_t id=(uint8_t)strtoul(server.arg("id").c_str(),nullptr,16);
  uint8_t payload[220]; size_t len=0;
  if(!parseHexPayload(server.arg("payload"),payload,len)){server.send(400,"application/json","{\"ok\":false}");return;}
  ubxLastEvent="Команда отправлена"; sendUbx(cls,id,payload,len);
  server.send(200,"application/json","{\"ok\":true}");
}
void handleUbxStatus() {
  JsonDocument d; d["event"]=ubxLastEvent;d["frame"]=lastUbxFrameHex;
  String out;serializeJson(d,out);server.send(200,"application/json",out);
}

void handleSetFixTimeout() {
  int seconds = server.arg("seconds").toInt();
  if (seconds < 1 || seconds > 60) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"seconds must be 1..60\"}");
    return;
  }
  fixTimeoutSeconds = (uint8_t)seconds;
  preferences.putUChar("fix_timeout", fixTimeoutSeconds);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleYandexKeyGet() {
  JsonDocument doc;
  doc["key"] = preferences.getString("yandex_key", "");
  String out; serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleYandexKeySet() {
  JsonDocument input;
  if (deserializeJson(input, server.arg("plain"))) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid JSON\"}");
    return;
  }
  String key = input["key"] | "";
  key.trim();
  if (key.length() > 160) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"key too long\"}");
    return;
  }
  preferences.putString("yandex_key", key);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCoords() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", coordsJson());
}

void handleUart() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", uartJson());
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

  String path = "/maps/" + mapName + "/" + z + "/" + x + "/" + y + ".png";

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

void handleSimulatorStart() {
  nmeaSimulatorEnabled = true;
  simulatorStartMs = millis();
  lastSimulatorSentenceMs = 0;
  satellites = 8; satellitesVisible = 8; hdop = 0.9;
  updateSimulatorPosition();
  server.send(200, "application/json", "{\"ok\":true,\"enabled\":true,\"route_seconds\":60}");
}

void handleSimulatorStop() {
  nmeaSimulatorEnabled = false;
  // Keep the last simulated point until the configurable stale-fix timeout expires.
  if (haveFix) lastFixMs = millis();
  satellites = 0;
  satellitesVisible = 0;
  server.send(200, "application/json", "{\"ok\":true,\"enabled\":false}");
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
  if (displayTestActive) { drawDisplayTest(); return; }
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setTextSize(2); gfx->setCursor(10, 8); gfx->println("GPS");
  gfx->setTextSize(1); gfx->setCursor(10, 38);
  gfx->print("STATUS: "); gfx->println(haveFix ? "FIX OK" : "NO FIX");
  gfx->setTextSize(2); gfx->setCursor(10, 62); gfx->println("LAT");
  gfx->setTextSize(1); gfx->setCursor(10, 86); gfx->println(haveFix ? String(lat, 6) : String("0.000000"));
  gfx->setTextSize(2); gfx->setCursor(10, 112); gfx->println("LON");
  gfx->setTextSize(1); gfx->setCursor(10, 136); gfx->println(haveFix ? String(lng, 6) : String("0.000000"));
  gfx->setTextSize(2); gfx->setCursor(10, 166); gfx->print("SAT: "); gfx->println(satellitesVisible);
  gfx->setTextSize(1); gfx->setCursor(10, 198);
  gfx->print("USED: "); gfx->println(satellites);
  gfx->print("ALT: "); gfx->print(altitude, 0); gfx->println(" m");
  gfx->println(); gfx->print("BLE: "); gfx->println(bleConnected ? "ON" : "OFF");
  gfx->print("IMU: "); gfx->println(imuReady ? "OK" : "NOT FOUND");
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
  Wire.setClock(100000);

  preferences.begin("imu", false);
  if (!preferences.isKey("points")) preferences.putString("points", "[]");
  fixTimeoutSeconds = preferences.getUChar("fix_timeout", 7);
  if (fixTimeoutSeconds < 1 || fixTimeoutSeconds > 60) fixTimeoutSeconds = 7;
  gyroOffsetX = preferences.getFloat("gx", 0);
  gyroOffsetY = preferences.getFloat("gy", 0);
  gyroOffsetZ = preferences.getFloat("gz", 0);

  uint8_t imuAddress = 0;
  for (uint8_t address = 0x08; address < 0x78; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("I2C device found at 0x%02X\n", address);
      if (address == 0x6A || address == 0x6B) imuAddress = address;
    }
  }
  if (imuAddress && imu.begin(Wire, imuAddress, IMU_SDA, IMU_SCL)) {
    imu.configAccel(AccelFullScaleRange::FS_8G, 250.0f, SensorQMI8658::LpfMode::MODE_0);
    imu.configGyro(GyroFullScaleRange::FS_1000_DPS, 250.0f, SensorQMI8658::LpfMode::MODE_0);
    imu.enableAccel();
    imu.enableGyro();
    imuReady = true;
    Serial.println("QMI8658 ready");
  } else {
    Serial.println("QMI8658 not found");
  }

  gpsSerial.begin(gpsBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  simulatorSerial.begin(gpsBaud, SERIAL_8N1, 9, 8);
  Serial.println("NMEA simulator UART1 ready: TX GPIO8, RX GPIO9, 57600 baud");

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
  server.on("/points", HTTP_GET, handleSavedPoints);
  server.on("/points/save", HTTP_POST, handleSavePoint);
  server.on("/points/update", HTTP_POST, handleUpdatePoint);
  server.on("/points/delete", HTTP_POST, handleDeletePoint);
  server.on("/uart/baud", HTTP_POST, handleSetBaud);
  server.on("/ubx/send", HTTP_POST, handleUbxSend);
  server.on("/ubx/status", HTTP_GET, handleUbxStatus);
  server.on("/imu", HTTP_GET, handleImu);
  server.on("/uart", HTTP_GET, handleUart);
  server.on("/maps", HTTP_GET, handleMaps);
  server.on("/maps/yandex-key", HTTP_GET, handleYandexKeyGet);
  server.on("/maps/yandex-key", HTTP_POST, handleYandexKeySet);
  server.on("/settings/fix-timeout", HTTP_POST, handleSetFixTimeout);
  server.on("/tiles", HTTP_GET, handleTile);
  server.on("/display/test", HTTP_POST, handleDisplayTest);
  server.on("/simulator/start", HTTP_POST, handleSimulatorStart);
  server.on("/simulator/stop", HTTP_POST, handleSimulatorStop);
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

String nmeaWithChecksum(const String& payload) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < payload.length(); i++) checksum ^= (uint8_t)payload[i];
  char suffix[6];
  snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  return "$" + payload + String(suffix) + "\r\n";
}

String toNmeaCoordinate(double value, bool isLatitude, char& hemisphere) {
  hemisphere = value < 0 ? (isLatitude ? 'S' : 'W') : (isLatitude ? 'N' : 'E');
  double absoluteValue = fabs(value);
  int degrees = (int)absoluteValue;
  double minutes = (absoluteValue - degrees) * 60.0;
  char buffer[20];
  snprintf(buffer, sizeof(buffer), isLatitude ? "%02d%07.4f" : "%03d%07.4f", degrees, minutes);
  return String(buffer);
}

void updateSimulatorPosition() {
  if (!nmeaSimulatorEnabled) return;
  const double centerLat = 54.922700;
  const double centerLng = 43.344800;
  const double radiusLat = 0.00045; // about 50 m
  const double radiusLng = 0.00065; // about 42 m at Sarov latitude
  const double pi = 3.14159265358979323846;
  double phase = (double)(millis() - simulatorStartMs) / 60000.0;
  phase -= floor(phase);
  double angle = phase * 2.0 * pi;
  simulatorLat = centerLat + radiusLat * cos(angle);
  simulatorLng = centerLng + radiusLng * sin(angle);
  lat = simulatorLat;
  lng = simulatorLng;
  altitude = 125.0;
  satellites = 8;
  satellitesVisible = 8;
  hdop = 0.9;
  haveFix = true;
  lastFixMs = millis();
}

void sendNmeaSimulatorFrame() {
  updateSimulatorPosition();
  const double pi = 3.14159265358979323846;
  double phase = (double)(millis() - simulatorStartMs) / 60000.0;
  phase -= floor(phase);
  double angle = phase * 2.0 * pi;
  double east = 0.00065 * cos(angle) * cos(simulatorLat * pi / 180.0);
  double north = -0.00045 * sin(angle);
  double course = atan2(east, north) * 180.0 / pi;
  if (course < 0) course += 360.0;
  double circumference = 2.0 * pi * sqrt((pow(50.0, 2.0) + pow(42.0, 2.0)) / 2.0);
  double speedKnots = (circumference / 60.0) * 1.943844;
  uint32_t seconds = (millis() / 1000UL) % 86400UL;
  char timeText[16], dateText[8];
  snprintf(timeText, sizeof(timeText), "%02lu%02lu%02lu.00",
    (unsigned long)(seconds / 3600UL),
    (unsigned long)((seconds / 60UL) % 60UL),
    (unsigned long)(seconds % 60UL));
  snprintf(dateText, sizeof(dateText), "151126");
  char latHem, lonHem;
  String nmeaLat = toNmeaCoordinate(simulatorLat, true, latHem);
  String nmeaLng = toNmeaCoordinate(simulatorLng, false, lonHem);
  String rmc = "GPRMC," + String(timeText) + ",A," + nmeaLat + "," + latHem + "," +
    nmeaLng + "," + lonHem + "," + String(speedKnots, 1) + "," + String(course, 1) + "," + dateText + ",,,A";
  String gga = "GPGGA," + String(timeText) + "," + nmeaLat + "," + latHem + "," +
    nmeaLng + "," + lonHem + ",1,08,0.9,125.0,M,0.0,M,,";
  simulatorSerial.print(nmeaWithChecksum(rmc));
  simulatorSerial.print(nmeaWithChecksum(gga));
  simulatorSerial.print(nmeaWithChecksum("GPGSA,A,3,01,02,03,04,05,06,07,08,,,,,1.2,0.9,0.8"));
  simulatorSerial.print(nmeaWithChecksum("GPGSV,2,1,08,01,45,120,40,02,35,230,38,03,25,310,36,04,60,050,42"));
  simulatorSerial.print(nmeaWithChecksum("GPGSV,2,2,08,05,30,180,39,06,20,270,35,07,50,020,41,08,15,330,32"));
  simulatorSerial.print(nmeaWithChecksum("GPVTG," + String(course, 1) + ",T,,M," + String(speedKnots, 1) + ",N," + String(speedKnots * 1.852, 1) + ",K,A"));
}

void loop() {
  server.handleClient();

  if (haveFix && millis() - lastFixMs > (unsigned long)fixTimeoutSeconds * 1000UL) {
    haveFix = false;
    lat = 0.0; lng = 0.0; altitude = 0.0;
    satellites = 0; satellitesVisible = 0; hdop = 0.0;
  }
  if (lastGgaMs && millis() - lastGgaMs > 3000) satellites = 0;
  if (lastGsvMs && millis() - lastGsvMs > 5000) satellitesVisible = 0;

  if (nmeaSimulatorEnabled && millis() - lastSimulatorSentenceMs >= 1000) {
    lastSimulatorSentenceMs = millis();
    sendNmeaSimulatorFrame();
  }

  unsigned long now = millis();
  if (now - uartWindowStartMs >= 1000) {
    uartBytesPerSecond = uartBytesWindow; uartBytesWindow = 0; uartWindowStartMs = now;
  }
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    processUbxByte((uint8_t)c);
    uartTotalBytes++; uartBytesWindow++; uartLastByteMs = millis();
    if (c == '\n') {
      uartCurrentLine.trim();
      if (uartCurrentLine.length()) processNmeaLine(uartCurrentLine);
      if (uartCurrentLine.length()) {
        uartLastLine = uartCurrentLine;
        uartRecentLines += uartCurrentLine + "\n";
        if (uartRecentLines.length() > 1200) uartRecentLines.remove(0, uartRecentLines.length()-1200);
      }
      uartCurrentLine = ""; uartLineOverflow = false;
    } else if (c != '\r') {
      if (uartCurrentLine.length() < 160) uartCurrentLine += c;
      else { uartLineOverflow = true; uartInvalidSentences++; uartCurrentLine = ""; }
    }
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

      if (gps.hdop.isValid()) {
        hdop = gps.hdop.hdop();
      }
    }
  }

  if (nmeaSimulatorEnabled) updateSimulatorPosition();

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
