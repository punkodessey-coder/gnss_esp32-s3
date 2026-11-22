#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include "ai_ml.h"
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <ImuDrv.hpp>
#include <SD_MMC.h>
#include <sqlite3.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_image_format.h>
#include <esp_app_desc.h>
#include <esp_system.h>
#include <time.h>
#include "firmware_build_info.h"

#ifndef FIRMWARE_GIT_SHA
#define FIRMWARE_GIT_SHA "local"
#endif

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

// SQLite + WebServer + ArduinoJson inside the handler overflow the default 8 KB loopTask
// stack when an MBTiles archive is opened ("Stack canary watchpoint" -> ESP32 reboots and
// the Wi-Fi AP/web UI disappears). 16 KB is enough.
#ifdef SET_LOOP_TASK_STACK_SIZE
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

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
bool firmwareUploadOk = false;
String firmwareUploadError;
size_t firmwareUploadBytes = 0;
bool firmwareUploadIsZip = false, firmwareUploadStarted = false;
uint8_t firmwareZipHeader[512] = {};
size_t firmwareZipHeaderBytes = 0, firmwareZipHeaderExpected = 30;
uint32_t firmwareZipRemaining = 0, firmwareZipExpectedCrc = 0, firmwareZipCrc = 0xFFFFFFFFUL;
uint32_t situationLogEntryNumber = 0;
Preferences preferences;

bool haveFix = false;
bool coordinatesAvailable = false;
uint32_t lastCoordinateMs = 0;
String coordinateSource = "";
int timezoneOffsetMinutes = 0;
int16_t wifiScanStatus = WIFI_SCAN_FAILED;
double lat = 0.0;
double lng = 0.0;
double altitude = 0.0;
int satellites = 0; // satellites used by a valid GGA fix
int satellitesVisible = 0; // satellites reported by GSV
double hdop = 0.0;
float avgGsvSnr = 0.0f;
uint8_t gsvSnrCount = 0;
struct SatelliteInfo { char system[3]; uint16_t prn; int8_t elevation; int16_t azimuth; int8_t snr; bool snrValid; bool used; uint32_t lastSeen; };
static SatelliteInfo satelliteInfo[32];
static uint8_t satelliteInfoCount = 0;
static bool ntpConfigured = false;
static unsigned long lastNtpRetryMs = 0;
GnssAiMonitor aiMonitor;
bool sdReady = false;
String mbtilesLastError = "База ещё не открывалась";
int mbtilesLastRc = SQLITE_OK;
static String mbtilesDiagnosticLog;
static void logMbtilesDiagnostic(const String& message) {
  String line = "[" + String(millis()) + " ms] " + message + "\n";
  Serial.print("[MBTiles] "); Serial.print(line);
  mbtilesDiagnosticLog += line;
  if (mbtilesDiagnosticLog.length() > 6000)
    mbtilesDiagnosticLog.remove(0, mbtilesDiagnosticLog.length() - 6000);
}
static sqlite3* mbtilesDb = nullptr;
static sqlite3_stmt* mbtilesTileStmt = nullptr;
static String mbtilesOpenName;
static int mbtilesMinZoom = -1, mbtilesMaxZoom = -1;
static uint64_t mbtilesFileSize = 0;
bool aiLoggingEnabled = false;
bool situationLoggingEnabled = false;
bool aiReferenceEnabled = false;
double aiReferenceLat = 0.0, aiReferenceLon = 0.0;
float aiReferenceRadiusM = 50.0f;
uint16_t reportIntervalSeconds = 300;
unsigned long lastSituationLogMs = 0;
unsigned long lastBleMessageMs = 0;
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
uint16_t gsvCycleSnrSum = 0;
uint8_t gsvCycleSnrCount = 0, gsvCycleExpected = 0, gsvCycleSeen = 0;
uint8_t gsvCycleVisible = 0;

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
  doc["coordinates_available"] = coordinatesAvailable;
  doc["coordinate_source"] = coordinateSource;
  doc["coordinate_age_ms"] = coordinatesAvailable ? (long)(millis() - lastCoordinateMs) : -1;
  doc["lat"] = serialized(String(lat, 7));
  doc["lng"] = serialized(String(lng, 7));
  doc["alt"] = altitude;
  doc["sat"] = satellites;
  doc["sat_visible"] = satellitesVisible;
  doc["fix_timeout_s"] = fixTimeoutSeconds;
  doc["hdop"] = hdop;
  doc["age_ms"] = haveFix ? (long)(millis() - lastFixMs) : -1;
  doc["gsv_snr_avg"] = gsvSnrCount ? avgGsvSnr : 0.0f;
  doc["gsv_snr_count"] = gsvSnrCount;
  doc["gsv_age_ms"] = lastGsvMs ? (long)(millis() - lastGsvMs) : -1;
  doc["gga_age_ms"] = lastGgaMs ? (long)(millis() - lastGgaMs) : -1;

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

bool parseNmeaCoordinate(const String& raw, const String& hemisphere, bool latitude, double& result) {
  if (!raw.length() || hemisphere.length() != 1) return false;
  const double packed = raw.toDouble();
  if (!isfinite(packed) || packed <= 0.0) return false;
  const int degrees = (int)(packed / 100.0);
  const double minutes = packed - degrees * 100.0;
  const int maxDegrees = latitude ? 90 : 180;
  if (degrees > maxDegrees || minutes < 0.0 || minutes >= 60.0 || (degrees == maxDegrees && minutes > 0.0)) return false;
  const char h = hemisphere[0];
  if (latitude && h != 'N' && h != 'S') return false;
  if (!latitude && h != 'E' && h != 'W') return false;
  result = degrees + minutes / 60.0;
  if (h == 'S' || h == 'W') result = -result;
  return true;
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
  const String type = line.substring(3, 6);
  if (type == "GGA") {
    const int fixQuality = nmeaField(line, 6).toInt();
    const int used = nmeaField(line, 7).toInt();
    satellites = (fixQuality > 0) ? constrain(used, 0, 99) : 0;
    double nmeaLat = 0.0, nmeaLng = 0.0;
    const bool ggaCoordsUpdated = parseNmeaCoordinate(nmeaField(line, 2), nmeaField(line, 3), true, nmeaLat) &&
        parseNmeaCoordinate(nmeaField(line, 4), nmeaField(line, 5), false, nmeaLng);
    if (ggaCoordsUpdated) {
      lat = nmeaLat; lng = nmeaLng; coordinatesAvailable = true;
      lastCoordinateMs = millis(); coordinateSource = "GGA";
    }
    if (fixQuality > 0 && ggaCoordsUpdated) { haveFix = true; lastFixMs = millis(); }
    else if (fixQuality == 0) haveFix = false;
    lastGgaMs = millis();
  } else if (type == "RMC") {
    const String status = nmeaField(line, 2);
    double nmeaLat = 0.0, nmeaLng = 0.0;
    if (parseNmeaCoordinate(nmeaField(line, 3), nmeaField(line, 4), true, nmeaLat) &&
        parseNmeaCoordinate(nmeaField(line, 5), nmeaField(line, 6), false, nmeaLng)) {
      lat = nmeaLat; lng = nmeaLng; coordinatesAvailable = true;
      lastCoordinateMs = millis(); coordinateSource = "RMC";
      if (status == "A") { haveFix = true; lastFixMs = millis(); }
      else if (status == "V") haveFix = false;
    }
  } else if (type == "GSA") {
    String system = line.substring(1, 3);
    for (uint8_t i = 0; i < satelliteInfoCount; ++i) {
      if (system == "GN" || system == satelliteInfo[i].system) satelliteInfo[i].used = false;
    }
    for (uint8_t field = 3; field <= 14; ++field) {
      int prn = nmeaField(line, field).toInt();
      if (prn <= 0) continue;
      for (uint8_t i = 0; i < satelliteInfoCount; ++i) {
        if (satelliteInfo[i].prn == prn &&
            (system == "GN" || system == satelliteInfo[i].system)) satelliteInfo[i].used = true;
      }
    }
  } else if (type == "GSV") {
    String system = line.substring(1, 3);
    const int totalMessages = constrain(nmeaField(line, 1).toInt(), 1, 9);
    const int messageNumber = constrain(nmeaField(line, 2).toInt(), 1, 9);
    const int visible = constrain(nmeaField(line, 3).toInt(), 0, 99);
    if (messageNumber == 1 || gsvCycleExpected != totalMessages || messageNumber != gsvCycleSeen + 1) {
      gsvCycleSnrSum = 0; gsvCycleSnrCount = 0;
      gsvCycleExpected = totalMessages; gsvCycleSeen = 0;
      gsvCycleVisible = visible;
    }
    for (uint8_t field = 4; field <= 16; field += 4) {
      String prnText = nmeaField(line, field);
      if (!prnText.length()) continue;
      int prn = prnText.toInt();
      if (prn <= 0 || prn > 999) continue;
      int slot = -1;
      for (uint8_t i = 0; i < satelliteInfoCount; ++i)
        if (satelliteInfo[i].prn == prn && system == satelliteInfo[i].system) { slot = i; break; }
      if (slot < 0 && satelliteInfoCount < 32) slot = satelliteInfoCount++;
      if (slot >= 0) {
        SatelliteInfo& sat = satelliteInfo[slot];
        strncpy(sat.system, system.c_str(), 2); sat.system[2] = 0; sat.prn = prn;
        sat.elevation = (int8_t)constrain(nmeaField(line, field + 1).toInt(), 0, 90);
        sat.azimuth = (int16_t)constrain(nmeaField(line, field + 2).toInt(), 0, 359);
        String snrText = nmeaField(line, field + 3);
        int snr = snrText.toInt();
        sat.snrValid = snrText.length() && snr > 0 && snr <= 99;
        sat.snr = sat.snrValid ? (int8_t)snr : -1;
        sat.lastSeen = millis();
      }
      String snrText = nmeaField(line, field + 3);
      int snr = snrText.toInt();
      if (snrText.length() && snr > 0 && snr <= 99) {
        gsvCycleSnrSum += snr; if (gsvCycleSnrCount < 99) ++gsvCycleSnrCount;
      }
    }
    gsvCycleSeen = messageNumber; gsvCycleVisible = visible; lastGsvMs = millis();
    if (messageNumber >= totalMessages) {
      satellitesVisible = gsvCycleVisible;
      gsvSnrCount = gsvCycleSnrCount;
      avgGsvSnr = gsvSnrCount ? (float)gsvCycleSnrSum / gsvSnrCount : 0.0f;
    }
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

void logAiDatasetRow() {
  if (!aiLoggingEnabled || !sdReady) return;
  File file = SD_MMC.open("/ai_dataset.csv", FILE_APPEND);
  if (!file) return;
  if (file.size() == 0) {
    file.println("millis,fix,lat,lon,speed_mps,hdop,sat_used,sat_visible,gsv_snr_avg,accel_magnitude,gyro_magnitude,profile_score,anomaly,anomaly_code,risk,motion");
  }
  const float speed = gps.speed.isValid() ? gps.speed.mps() : NAN;
  const float accelMagnitude = imuReady ? sqrtf(ax*ax + ay*ay + az*az) : NAN;
  const float gyroMagnitude = imuReady ? sqrtf(gx*gx + gy*gy + gz*gz) : NAN;
  file.printf("%lu,%u,%.7f,%.7f,%.3f,%.2f,%d,%d,%.2f,%.3f,%.3f,%d,%u,%s,%s,%s\n",
    (unsigned long)millis(), haveFix ? 1U : 0U, lat, lng, speed, hdop,
    satellites, satellitesVisible, gsvSnrCount ? avgGsvSnr : NAN,
    accelMagnitude, gyroMagnitude, aiMonitor.qualityScore(),
    aiMonitor.anomaly() ? 1U : 0U, aiMonitor.anomalyCode(),
    aiMonitor.risk(), aiMonitor.motion());
  file.close();
}

float aiDistanceMeters(double lat1,double lon1,double lat2,double lon2) {
  const double rad=0.017453292519943295;
  const double dLat=(lat2-lat1)*rad,dLon=(lon2-lon1)*rad;
  const double a=sin(dLat/2)*sin(dLat/2)+cos(lat1*rad)*cos(lat2*rad)*sin(dLon/2)*sin(dLon/2);
  return (float)(6371000.0*2.0*atan2(sqrt(a),sqrt(fmax(0.0,1.0-a))));
}
String aiJson() {
  JsonDocument doc;
  doc["enabled"] = true;
  doc["model"] = "online-statistical-v1";
  doc["model_type"] = "Welford online anomaly detector";
  doc["trained"] = aiMonitor.trained();
  doc["offline_model_loaded"] = aiMonitor.offlineModelLoaded();
  doc["offline_model_training_rows"] = aiMonitor.trainingSamples();
  doc["model_training_mode"] = aiMonitor.offlineModelLoaded() ? "offline_robust_median_mad" : "online_welford_ram";
  doc["profile_role"] = aiMonitor.profileRole();
  doc["samples"] = aiMonitor.samples();
  doc["quality_score"] = aiMonitor.qualityScore();
  doc["risk"] = aiMonitor.risk();
  doc["motion"] = aiMonitor.motion();
  doc["anomaly"] = aiMonitor.anomaly();
  doc["anomaly_code"] = aiMonitor.anomalyCode();
  doc["max_z_score"] = aiMonitor.maxZ();
  doc["gnss_speed_position_mismatch"] = aiMonitor.mismatch();
  doc["speed_difference_mps"] = aiMonitor.mismatchSpeed();
  doc["hdop_trend"] = aiMonitor.hdopTrend();
  doc["snr_trend"] = aiMonitor.cnoTrend();
  doc["avg_gsv_snr"] = avgGsvSnr;
  doc["snr_satellites"] = gsvSnrCount;
  doc["fix"] = haveFix;
  doc["coordinates_available"] = coordinatesAvailable;
  doc["logging_enabled"] = aiLoggingEnabled;
  doc["reference_enabled"] = aiReferenceEnabled;
  doc["reference_lat"] = aiReferenceEnabled ? aiReferenceLat : 0.0;
  doc["reference_lon"] = aiReferenceEnabled ? aiReferenceLon : 0.0;
  doc["reference_radius_m"] = aiReferenceRadiusM;
  const float referenceDistance=(aiReferenceEnabled&&haveFix&&coordinatesAvailable)?aiDistanceMeters(aiReferenceLat,aiReferenceLon,lat,lng):-1.0f;
  doc["reference_distance_m"] = referenceDistance;
  doc["outside_reference"] = referenceDistance>=0.0f&&referenceDistance>aiReferenceRadiusM;
  doc["coordinate_source"] = coordinateSource;
  doc["coordinate_age_ms"] = coordinatesAvailable ? (long)(millis() - lastCoordinateMs) : -1;
  doc["hdop"] = hdop;
  doc["accel_magnitude_mps2"] = imuReady ? sqrtf(ax*ax + ay*ay + az*az) : 0.0f;
  doc["linear_accel_estimate_mps2"] = imuReady ? fabsf(sqrtf(ax*ax + ay*ay + az*az) - 9.80665f) : 0.0f;
  String situation, verdict, reasons;
  if (aiMonitor.mismatch()) {
    verdict = "ОБНАРУЖЕН ПОДОЗРИТЕЛЬНЫЙ ПРИЗНАК";
    situation = "Скорость, сообщаемая GNSS, не согласуется с перемещением координат.";
    reasons = "Это может указывать на нарушение целостности навигационных данных, но само по себе не доказывает спуффинг.";
  } else if (String(aiMonitor.risk()) == "spoofing_signature_match") {
    verdict = "СОВПАДЕНИЕ С ПРОФИЛЕМ СПУФИНГА";
    situation = "Параметры GNSS похожи на ранее сохранённую сигнатуру спуфинга.";
    reasons = "Это сходство с историческим профилем, а не самостоятельное доказательство текущей подмены координат.";
  } else if (aiMonitor.anomaly()) {
    verdict = "ОБНАРУЖЕНО СТАТИСТИЧЕСКОЕ ОТКЛОНЕНИЕ";
    situation = "Один или несколько параметров GNSS заметно отличаются от накопленного профиля.";
    reasons = "Код отклонения: " + String(aiMonitor.anomalyCode()) + ". Требуется наблюдение за динамикой показателей.";
  } else if (haveFix && (satellites < 4 || hdop > 3.0f)) {
    verdict = "НИЗКОЕ КАЧЕСТВО НАВИГАЦИОННОГО РЕШЕНИЯ";
    situation = "Фикс получен, но геометрия спутников или HDOP ограничивают надёжность координат.";
    reasons = "Спутников в решении: " + String(satellites) + "; HDOP: " + String(hdop, 2) + ".";
  } else if (String(aiMonitor.risk()) == "degrading") {
    verdict = "КАЧЕСТВО GNSS УХУДШАЕТСЯ";
    situation = "Наблюдается ухудшение тренда HDOP/SNR.";
    reasons = "HDOP trend: " + String(aiMonitor.hdopTrend(), 2) + "; SNR trend: " + String(aiMonitor.cnoTrend(), 1) + ".";
  } else if (!aiMonitor.trained()) {
    verdict = haveFix ? "НАКОПЛЕНИЕ БАЗОВОГО ПРОФИЛЯ" : "НЕТ ФИКСА — НАКОПЛЕНИЕ ПРОДОЛЖАЕТСЯ";
    situation = "Система анализирует все доступные данные независимо от наличия фикса.";
    reasons = "Для обучения используются доступные уровни сигналов, видимые спутники и измерения IMU. Координатные проверки выполняются только при FIX.";
  } else {
    verdict = haveFix ? "ЯВНЫХ ОТКЛОНЕНИЙ НЕ ОБНАРУЖЕНО" : "АНАЛИЗ БЕЗ ФИКСА: ЯВНЫХ ОТКЛОНЕНИЙ НЕ ВЫЯВЛЕНО";
    situation = "По доступным телеметрическим данным сейчас не выявлено статистических отклонений.";
    reasons = "Видно спутников: " + String(satellitesVisible) + "; средний GSV SNR: " + String(avgGsvSnr, 1) + "; проверка скорости и перемещения: " + String(haveFix ? "доступна" : "недоступна без FIX") + ". Это не гарантирует отсутствие помех или спуфинга.";
  }
  if (imuReady) reasons += " IMU доступен, но акселерометр и гироскоп не откалиброваны и исключены из оценки GNSS-целостности.";
  else reasons += " IMU недоступен и не участвует в оценке.";
  String classification=String(aiMonitor.risk());
  if(aiReferenceEnabled&&haveFix&&coordinatesAvailable){
    if(referenceDistance>aiReferenceRadiusM){
      if(satellites>=4&&avgGsvSnr>=25.0f){classification="spoofing_suspected";verdict="ПОДОЗРЕНИЕ НА ПОДМЕНУ КООРДИНАТ";situation="FIX получен, но позиция вне контрольного радиуса при относительно сохранном уровне сигналов.";reasons="Отклонение "+String(referenceDistance,1)+" м при допуске "+String(aiReferenceRadiusM,1)+" м. Это подозрительный признак, но не доказательство спуфинга.";}
      else{classification="position_deviation_with_weak_signal";verdict="ПОЗИЦИЯ ВНЕ КОНТРОЛЬНОЙ ЗОНЫ — СЛАБЫЙ ПРИЁМ";situation="Координаты вышли за допустимый радиус, качество приёма снижено.";reasons="Отклонение "+String(referenceDistance,1)+" м; спутников в решении: "+String(satellites)+"; средний SNR: "+String(avgGsvSnr,1)+" дБ.";}
    }else{classification="inside_reference";if(!aiMonitor.anomaly()){verdict="НОРМА — КООРДИНАТЫ В ДОПУСТИМОЙ ЗОНЕ";situation="Текущая позиция находится в пределах контрольного радиуса.";reasons="Отклонение "+String(referenceDistance,1)+" м из допустимых "+String(aiReferenceRadiusM,1)+" м. Качество сигналов оценивается отдельно.";}}
  }else if(!haveFix&&satellitesVisible>0&&avgGsvSnr>0&&avgGsvSnr<20.0f){classification="interference_possible";verdict="ВОЗМОЖНЫ ПОМЕХИ GNSS";situation="Фикс отсутствует, а наблюдаемый уровень сигналов низкий.";reasons="Возможны помехи или слабое покрытие; по NMEA нельзя доказать глушение.";}
  doc["classification"]=classification;
  doc["situation"] = situation;
  doc["verdict"] = verdict;
  doc["reasons"] = reasons;
  doc["imu_used_for_integrity"] = false;
  doc["report_interval_seconds"] = reportIntervalSeconds;
  doc["situation_logging"] = situationLoggingEnabled;
  doc["satellites_used"] = satellites;
  doc["satellites_visible"] = satellitesVisible;
  doc["gps_speed_mps"] = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
  doc["imu_ready"] = imuReady;
  doc["note"] = "Statistical anomaly score; not proof of jamming or spoofing.";
  String out; serializeJson(doc, out); return out;
}

bool validMapName(const String& name);

String mapsJson() {
  JsonDocument doc;
  JsonArray maps = doc["maps"].to<JsonArray>();

  File root;
  if (sdReady) root = SD_MMC.open("/maps");
  if (root && root.isDirectory()) {
    File item = root.openNextFile();

    while (item) {
      String itemName = String(item.name());
      int slash = itemName.lastIndexOf('/');
      if (slash >= 0) itemName = itemName.substring(slash + 1);
      if (item.isDirectory()) {
        if (validMapName(itemName)) maps.add(itemName);
      } else if (itemName.endsWith(".mbtiles")) {
        itemName.remove(itemName.length() - 8);
        if (validMapName(itemName)) maps.add(itemName);
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
<style>
body{margin:0;padding:64px 12px 12px;background:#111;color:#eee;font-family:system-ui,sans-serif}
.app-shell{display:block;min-width:0}.app-content{width:100%;min-width:0}.top-menu{position:fixed;z-index:2000;right:12px;top:10px}.menu-toggle{width:44px;height:44px;border-radius:12px;background:#2f81f7;color:white;font-size:21px;padding:0;box-shadow:0 4px 16px #0008}.tabs{display:none;position:absolute;right:0;top:50px;width:210px;max-height:calc(100vh - 70px);overflow-y:auto;flex-direction:column;gap:6px;padding:10px;background:#1c1c1c;border:1px solid #444;border-radius:12px;box-shadow:0 8px 28px #000b}.tabs.open{display:flex}.tab{width:100%;text-align:left;padding:11px 12px;border-radius:8px;background:#333;color:#eee;border:0;font-size:14px}.tab.active{background:#2f81f7;color:#fff}.fix-badge{display:inline-flex;align-items:center;gap:8px;padding:8px 12px;border-radius:8px;font-weight:700;margin:8px 0}.fix-badge:before{content:'';width:10px;height:10px;border-radius:50%;background:currentColor}.fix-badge.has-fix{color:#37d67a;background:#123522}.fix-badge.no-fix{color:#ff6262;background:#3b1919}.map-actions{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.map-status{min-height:22px;padding:9px 10px;margin-top:8px;border-radius:8px;background:#252525;color:#ddd;font-size:13px}.map-status.good{color:#58e18b}.map-status.error{color:#ff7777}.map-status.loading{color:#ffd166}
.card{background:#1c1c1c;border-radius:14px;padding:16px;margin-bottom:12px}
.metric{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid #333;color:#aaa}
.metric strong{color:#fff;font:600 18px ui-monospace,monospace}
.gps-card .coords{margin-top:14px;color:#8ab4ff}
.coords,.imu{font-family:monospace;font-size:15px;word-break:break-word;white-space:pre-wrap;line-height:1.5}
#uartStats,#uartLast,#uartRecent{font-size:13px;line-height:1.55;white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word}
#uartRecent{min-height:72px;max-height:220px;overflow:auto;background:#151515;border-radius:8px;padding:10px;box-sizing:border-box}#uartStats,#uartLast{min-height:0;max-height:180px;overflow:auto;box-sizing:border-box}.uart-monitor{min-height:0;max-height:none;overflow:visible;box-sizing:border-box;display:flex;flex-direction:column;gap:8px}.uart-monitor pre{flex:0 1 auto;width:100%;white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word}
button{padding:10px 14px;border:0;border-radius:8px;background:#2f81f7;color:#fff;font-size:15px;margin:4px 4px 0 0}
button.gray{background:#555}
.map-controls{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}.map-controls select{flex:1;min-width:180px}.map-progress{display:flex;gap:10px;align-items:center;font-size:12px;color:#bbb;margin:6px 0}.map-progress progress{flex:1;height:10px;min-width:80px}#map{height:50vh;border-radius:12px}
.small{color:#aaa;font-size:13px}
.hidden{display:none}
select,input,textarea{padding:8px;border-radius:6px;background:#222;color:#eee;border:1px solid #444;max-width:100%;box-sizing:border-box}
.ubx-layout{display:grid;grid-template-columns:minmax(150px,220px) 1fr;gap:14px;margin-top:14px}
.ubx-nav{display:flex;flex-direction:column;gap:5px}.ubx-nav-item{text-align:left;background:#292929;margin:0}.ubx-nav-item.active{background:#245ca5}
.ubx-panel{min-width:0;background:#171717;border-radius:10px;padding:14px}
.ubx-field{display:grid;grid-template-columns:minmax(130px,1fr) minmax(120px,1fr);gap:12px;align-items:center;padding:9px 0;border-bottom:1px solid #333}
.ubx-field label{color:#bbb}.ubx-field input,.ubx-field select{width:100%}.ubx-checks{display:flex;flex-wrap:wrap;gap:14px}.ubx-checks label{display:flex;gap:6px;align-items:center}
.ubx-actions{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0}
@media(max-width:650px){body{padding:6px;padding-top:62px}.top-menu{right:8px;top:8px}.tabs{width:min(240px,calc(100vw - 24px))}.ubx-layout{grid-template-columns:1fr}.ubx-nav{display:grid;grid-template-columns:repeat(2,minmax(0,1fr))}.ubx-nav-item{font-size:12px;padding:8px}.ubx-field{grid-template-columns:1fr}#map{height:42vh}}

</style>
</head>
<body>
<div class="app-shell">
<div class="top-menu"><button class="menu-toggle" id="menuToggle" aria-label="Открыть меню" onclick="toggleMenu()">☰</button>
<div class="tabs" id="mainMenu">
<button id="tabGpsBtn" class="tab active" onclick="showTab('gps')">Главная</button>
<button id="tabImuBtn" class="tab" onclick="showTab('imu')">IMU</button>
<button id="tabTestsBtn" class="tab" onclick="showTab('tests')">Тесты</button>
<button id="tabUbxBtn" class="tab" onclick="showTab('ubx')">u-blox</button>
<button id="tabPointsBtn" class="tab" onclick="showTab('points')">Точки</button>
<button id="tabAiBtn" class="tab" onclick="showTab('ai')">AI / ML</button>
<button id="tabSatBtn" class="tab" onclick="showTab('sat')">Спутники и сигнал</button>
<button id="tabDeviceBtn" class="tab" onclick="showTab('device')">Устройство</button>
</div></div>
<main class="app-content">
<div id="gpsTab">
<div class="card gps-card">
<h2>GNSS — текущее положение</h2>
<div id="fixBadge" class="fix-badge no-fix">НЕТ ФИКСА</div>
<div id="status" class="small">Ожидание данных…</div>
<div class="metric"><span>Широта</span><strong id="latValue">0.000000</strong></div>
<div class="metric"><span>Долгота</span><strong id="lngValue">0.000000</strong></div>
<div class="metric"><span>Используется в решении (GGA)</span><strong id="satValue">0</strong></div>
<div class="metric"><span>Видно по GSV</span><strong id="satVisibleValue">0</strong></div>
<div class="metric"><span>Средний SNR (GSV)</span><strong id="gsvSnrValue">—</strong></div>
<div class="coords" id="coords">0.000000, 0.000000</div>
<button onclick="copyCoords()">Скопировать координаты</button>
<button class="gray" onclick="saveCurrentPoint()">Сохранить текущую точку</button>
<button class="gray" onclick="connectBle()">Подключить Bluetooth</button>
<div id="bleStatus" class="small"></div>
</div>
<div class="card"><h2>Карта</h2>
<div class="map-controls"><select id="mapSelect"><option value="">— Выберите карту —</option><option value="online:osm">OpenStreetMap — онлайн</option></select><button onclick="loadSelectedMap()">Загрузить карту</button></div><div class="map-progress"><progress id="tileProgress" max="1" value="0"></progress><span id="tileProgressText">Ожидание тайлов</span></div><div id="map"></div><div class="small">OpenStreetMap — единственная онлайн-карта. Остальные карты загружаются с microSD.</div>
<div class="map-actions"><button class="gray" onclick="centerOnGps()">К текущей точке</button><span id="followStatus" class="small">Автоцентрирование включено</span></div>
</div>
<div class="card"><div class="ubx-field"><label for="fixTimeoutSeconds">Скрывать точку после потери координат (сек.)</label><input id="fixTimeoutSeconds" type="number" min="1" max="60" value="7"></div><button class="gray" onclick="saveFixTimeout()">Сохранить тайм-аут</button><div id="mapStatus" class="map-status">Загрузка карты…</div></div>

</div>

<div id="testsTab" class="hidden">
<div class="card"><h2>Монитор UART GPS</h2>
<div class="uart-monitor"><div id="uartStatus" class="small">Ожидание данных UART…</div>
<label for="uartBaud">Скорость UART GPS</label>
<select id="uartBaud" onchange="changeUartBaud(this.value)">
<option>9600</option><option>19200</option><option>38400</option><option selected>57600</option><option>115200</option><option>230400</option>
</select>
<pre id="uartStats" class="coords">—</pre>
<h3>Последняя строка NMEA</h3><pre id="uartLast" class="coords">—</pre>
<h3>Последние строки NMEA</h3><pre id="uartRecent" class="coords">—</pre></div>
</div>
<div class="card"><h2>Тест координат</h2><div class="small">Проверка актуальности фикса, координат, количества спутников и HDOP.</div><button onclick="testCoordinates()">Проверить координаты</button><pre id="coordinateTestResult" class="coords">Тест ещё не запускался</pre></div>
<div class="card"><h2>Тест чтения MBTiles</h2><div class="small">Проверяет открытие выбранной базы и чтение реального тайла в текущей точке. Если фикса нет, используется тестовая точка Сарова.</div><select id="mbtilesTestMap"><option value="">— Выберите офлайн-карту —</option></select><button onclick="testMbtiles()">Проверить чтение карты</button><button class="gray" onclick="downloadMbtilesLogs()">Скачать журнал MBTiles</button><pre id="mbtilesTestResult" class="coords">Тест ещё не запускался</pre></div>
<div class="card"><h2>Аппаратные тесты</h2><button onclick="testDisplay()">Тест экрана</button><div class="small">Тест экрана временно показывает цветовые полосы на дисплее.</div></div>
<div class="card"><h2>Тестовый NMEA-генератор</h2><div class="small">Имитирует движение по замкнутому маршруту в центре Сарова: один полный круг за 60 секунд. NMEA выходит через TX GPIO8 (57600 бод); для физического loopback подключите GPIO8 → GPS RX GPIO44 и общий GND.</div><button onclick="startNmeaSimulator()">Включить имитацию</button><button class="gray" onclick="stopNmeaSimulator()">Выключить</button><div id="simulatorStatus" class="small">Имитатор выключен</div></div>
</div>
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
<div id="satTab" class="hidden">
<div class="card"><h2>Небесная диаграмма спутников</h2><div class="small">Центр — зенит, внешний круг — горизонт; азимут от севера по часовой стрелке. Цвет — уровень SNR из NMEA GSV.</div>
<svg id="skyPlot" viewBox="0 0 320 320" style="width:100%;max-width:420px;display:block;margin:auto;background:#101820;border-radius:12px">
<circle cx="160" cy="160" r="140" fill="none" stroke="#64748b"/><circle cx="160" cy="160" r="93.3" fill="none" stroke="#475569"/><circle cx="160" cy="160" r="46.7" fill="none" stroke="#475569"/><path d="M20 160H300 M160 20V300 M61 61L259 259 M259 61L61 259" stroke="#475569"/>
<g fill="#cbd5e1" font-size="11"><text x="160" y="13" text-anchor="middle">N</text><text x="307" y="164">E</text><text x="160" y="315" text-anchor="middle">S</text><text x="5" y="164">W</text></g></svg><div id="satSummary" class="small">Ожидание GSV…</div></div>
<div class="card"><h3>Уровень сигнала по спутникам</h3><div class="small">SNR из NMEA GSV, не калиброванный C/N₀ и не RF-спектр.</div><svg id="snrBars" viewBox="0 0 360 205" style="width:100%;display:block;background:#101820;border-radius:10px"></svg></div>
<div class="card"><h3>Радиочастотный спектр</h3><div class="small">NEO-6M не выдаёт IQ-отсчёты или FFT. Настоящий спектр/водопад требует внешнего SDR или приёмника с IQ.</div></div>
<div class="card"><h3>Спутники</h3><div id="satelliteList" class="small">Ожидание данных…</div></div>
<div class="metric"><span>Соответствие базовому профилю</span><strong id="aiScore">—</strong></div>
<div class="metric"><span>Состояние модели</span><strong id="aiRisk">—</strong></div>
<div class="metric"><span>Обучающие интервалы</span><strong id="aiSamples">0</strong></div>
<div class="metric"><span>Аномалия</span><strong id="aiAnomaly">—</strong></div>
<div class="metric"><span>Несоответствие скорости и координат GNSS</span><strong id="aiMismatch">—</strong></div>
<div class="metric"><span>Класс движения</span><strong id="aiMotion">—</strong></div>
<div class="metric"><span>Средний GSV SNR</span><strong id="aiSnr">—</strong></div>
<div class="metric"><span>Тренд HDOP</span><strong id="aiHdopTrend">—</strong></div>
<div class="metric"><span>Тренд SNR</span><strong id="aiSnrTrend">—</strong></div>
<pre id="aiDetails" class="coords">Модель собирает исходный профиль приёма…</pre>
</div>
<div id="deviceTab" class="hidden">
<div class="card">
<h2>Устройство — сведения о прошивке</h2>
<div id="firmwareInfo" class="coords">Загрузка сведений…</div>
<button class="gray" onclick="loadFirmwareInfo()">Обновить сведения</button>
<button onclick="downloadCurrentFirmware()">Выгрузить установленную прошивку (.zip)</button>
<div class="small">Сохраняется точный образ запущенного приложения. Файл можно использовать как резервную копию.</div>
</div>
<div class="card">
<h2>Интернет и синхронизация времени</h2>
<p class="small">Точка доступа ESP32-GPS остаётся включённой. Для NTP подключите ESP32 к Wi-Fi роутеру с доступом в интернет.</p>
<label for="wifiNetworks">Найденные сети Wi-Fi</label><select id="wifiNetworks" style="width:100%"><option value="">Сначала выполните сканирование</option></select><button class="gray" onclick="scanWifiNetworks()">Сканировать сети</button><button class="gray" onclick="selectWifiNetwork()">Выбрать сеть</button>
<label for="wifiSsid">Имя Wi-Fi сети (SSID)</label><input id="wifiSsid" maxlength="32" autocomplete="off" placeholder="SSID">
<label for="wifiPassword">Пароль Wi-Fi</label><input id="wifiPassword" type="password" maxlength="63" autocomplete="new-password" placeholder="Пароль">
<button onclick="saveWifiNetwork()">Подключить и сохранить</button><div id="networkStatus" class="small">Проверка…</div>
</div>
<div class="card">
<h2>Загрузка новой прошивки</h2>
<p>Выберите BIN или ZIP-архив прошивки, скачанный из интерфейса устройства. ZIP должен содержать BIN без сжатия. Во время обновления не отключайте питание и не закрывайте страницу.</p>
<input id="firmwareFile" type="file" accept=".bin,.zip,application/octet-stream,application/zip">
<div><button onclick="uploadFirmware()">Загрузить и установить прошивку</button></div>
<progress id="firmwareProgress" max="100" value="0" style="width:100%;margin-top:12px"></progress>
<div id="firmwareUploadStatus" class="small">Ожидание файла</div>
</div>
<div class="card">
<h2>Перезагрузка</h2>
<p>Перезапуск контроллера ESP32-S3. Данные и файлы microSD не удаляются.</p>
<button class="danger" onclick="rebootDevice()">Перезагрузить ESP32</button>
<div id="rebootStatus" class="small">Контроллер работает</div>
</div>
</div>
<div id="aiTab" class="hidden">
<div class="card">
<h2>AI / ML — мониторинг навигации</h2>
<div id="aiStatus" class="small">Ожидание данных…</div>
<div class="card" style="background:#152235"><h3>Заключение по текущей обстановке</h3><strong id="aiVerdict">Ожидание данных…</strong><div id="aiSituation" class="coords">Сбор телеметрии…</div><div id="aiReasons" class="small"></div></div>

<button class="gray" onclick="resetAiModel()">Сбросить профиль модели</button>
<div class="card"><h3>Профиль: импорт / экспорт</h3><a href="/ai/profile" style="color:#8ab4ff">Скачать профиль JSON</a><p><input id="aiProfileFile" type="file" accept=".json,application/json"></p><button onclick="uploadAiProfile()">Загрузить профиль</button><div id="aiProfileStatus" class="small"></div></div>
<div class="card"><h3>Контрольная точка навигации</h3><div class="small">Сравнивает текущий FIX с реальной точкой. Координаты сохраняются в настройках устройства, но не в JSON-профиле и не в коде. Рекомендуемый радиус 30–50 м.</div><label>Широта</label><input id="trainRefLat" type="number" step="0.000001"><label>Долгота</label><input id="trainRefLon" type="number" step="0.000001"><label>Допуск, м</label><input id="trainRadius" type="number" min="5" max="5000" value="50"><button onclick="saveAiReference()">Сохранить контрольную точку</button><button class="gray" onclick="clearAiReference()">Отключить контрольную точку</button><div id="referenceStatus" class="small">Загрузка…</div><pre id="trainCommand" class="coords"></pre></div>
<div class="card"><h3>Журнал обстановки и интервал сообщений</h3><label for="reportInterval">Интервал (10 секунд — 30 минут)</label><select id="reportInterval"><option value="10">10 секунд</option><option value="30">30 секунд</option><option value="60">1 минута</option><option value="300" selected>5 минут — рекомендуется</option><option value="600">10 минут</option><option value="900">15 минут</option><option value="1800">30 минут</option></select><button onclick="saveReportInterval()">Применить интервал</button><div class="ubx-actions"><button onclick="setSituationLogging(true)">Начать журнал</button><button class="gray" onclick="setSituationLogging(false)">Остановить журнал</button><a href="/ai/situation-log" style="color:#8ab4ff;padding:10px">Скачать журнал</a></div><div id="situationLogStatus" class="small">Журнал остановлен</div></div>
<div class="ubx-actions"><button onclick="setAiLogging(true)">Начать подробную запись CSV</button><button class="gray" onclick="setAiLogging(false)">Остановить запись CSV</button><a href="/ai/dataset" style="color:#8ab4ff;padding:10px">Скачать CSV</a></div><div id="aiLogStatus" class="small">Запись набора данных выключена</div>
<div class="small">Локальная статистическая ML-модель с онлайн-обучением. Отклонения не доказывают глушение или spoofing. GSV SNR — приближённый уровень сигнала, не калиброванный C/N₀.</div>
</div>
</div>

<div id="imuTab" class="hidden">
<div class="card"><h2>QMI8658 — состояние и идентификация</h2><div id="imuStatus" class="small">Ожидание данных…</div><pre id="imuData" class="imu">—</pre></div>
<div class="card"><h3>Калибровка гироскопа</h3><div class="small">Положите плату неподвижно на ровную поверхность во время калибровки.</div><button onclick="calibrateZero()">Откалибровать смещение</button><button class="gray" onclick="resetCalibration()">Сбросить калибровку</button></div>
<div class="card"><h3>Доступные измерения</h3><div class="small">QMI8658 — 6-осевой IMU: три оси акселерометра и три оси гироскопа. Магнитометра и встроенного компаса на этой плате нет. Roll/Pitch вычисляются по вектору ускорения; yaw без магнитометра/внешней привязки не определяется.</div></div>
</div>
</main>
</div>

<script>
let currentMap='';
let tileLayer=null;
let map=null, gpsMarker=null, currentFix=false, currentCoordinatesAvailable=false;
let currentLat=59.9386,currentLng=30.3141;
let timezoneConfigured=false, wifiScanBusy=false;
let followGps=true, mapLoadToken=0, mapTilesLoaded=0, mapTilesFailed=0;

function toggleMenu(){document.getElementById('mainMenu').classList.toggle('open');}
function showTab(name){
document.getElementById('mainMenu').classList.remove('open');
document.getElementById('deviceTab').classList.toggle('hidden',name!=='device');
document.getElementById('tabDeviceBtn').classList.toggle('active',name==='device');
document.getElementById('gpsTab').classList.toggle('hidden',name!=='gps');
document.getElementById('imuTab').classList.toggle('hidden',name!=='imu');
document.getElementById('testsTab').classList.toggle('hidden',name!=='tests');
document.getElementById('ubxTab').classList.toggle('hidden',name!=='ubx');
document.getElementById('pointsTab').classList.toggle('hidden',name!=='points');
document.getElementById('aiTab').classList.toggle('hidden',name!=='ai');
document.getElementById('satTab').classList.toggle('hidden',name!=='sat');
document.getElementById('tabSatBtn').classList.toggle('active',name==='sat');
document.getElementById('tabGpsBtn').classList.toggle('active',name==='gps');
document.getElementById('tabImuBtn').classList.toggle('active',name==='imu');
document.getElementById('tabTestsBtn').classList.toggle('active',name==='tests');
document.getElementById('tabUbxBtn').classList.toggle('active',name==='ubx');
document.getElementById('tabPointsBtn').classList.toggle('active',name==='points');
document.getElementById('tabAiBtn').classList.toggle('active',name==='ai');
if(name==='gps'){
  if(map) setTimeout(()=>map.invalidateSize(),150);
}
if(name==='sat') refreshSatellites();
if(name==='device'){refreshNetworkStatus();scanWifiNetworks();}
if(name==='tests') refreshUart();
if(name==='device'){document.getElementById('rebootStatus').textContent='Контроллер работает';loadFirmwareInfo();}
if(name==='ubx' && !document.getElementById('ubxFields').children.length) selectUbxPage('gnss');
}

const BLANK_TILE='data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';
let leafletPromise=null;
function loadScriptTimeout(src,ms){return new Promise((resolve,reject)=>{const s=document.createElement('script');const t=setTimeout(()=>{s.remove();reject(new Error('timeout'));},ms);s.onload=()=>{clearTimeout(t);resolve();};s.onerror=()=>{clearTimeout(t);s.remove();reject(new Error('load error'));};s.src=src;document.head.appendChild(s);});}
// Leaflet is loaded lazily and never blocks the page: first from microSD (/lib/), then from the CDN
// (only works when the phone has internet). The ESP32 access point itself has no internet.
function ensureLeaflet(){
  if(typeof L!=='undefined')return Promise.resolve(true);
  if(!leafletPromise){leafletPromise=(async()=>{
    for(const base of ['/lib/','https://unpkg.com/leaflet@1.9.4/dist/']){
      try{await loadScriptTimeout(base+'leaflet.js',base==='/lib/'?5000:8000);const l=document.createElement('link');l.rel='stylesheet';l.href=base+'leaflet.css';document.head.appendChild(l);return typeof L!=='undefined';}catch(e){}
    }
    return false;
  })().then(ok=>{if(!ok)leafletPromise=null;return ok;});}
  return leafletPromise;
}
function initMap(){
  if(typeof L==='undefined'){document.getElementById('mapStatus').textContent='Leaflet не найдена: скопируйте leaflet.js/leaflet.css в /lib на microSD (maps/download_maps.ps1 делает это) или подключите телефон к интернету';return false;}
  if(!map){map=L.map('map').setView(currentCoordinatesAvailable?[currentLat,currentLng]:[59.9386,30.3141],currentCoordinatesAvailable?16:11);map.on('dragstart',disableGpsFollow);map.getContainer().addEventListener('wheel',disableGpsFollow,{passive:true});}
  return true;
}
async function loadMaps(){
  const select=document.getElementById('mapSelect');let mapNames=[];
  try{
    const d=await (await fetch('/maps')).json();mapNames=Array.isArray(d.maps)?d.maps:[];
    if(mapNames.length){
      const group=document.createElement('optgroup');group.label='Офлайн-карты microSD';
      mapNames.forEach(name=>{const option=document.createElement('option');option.value='sd:'+name;option.textContent=name;group.appendChild(option);});
      select.appendChild(group);setMapStatus('Найдено офлайн-карт: '+mapNames.length+'. Выберите карту и нажмите «Загрузить».','good');
    }else setMapStatus('На microSD нет карт в каталоге /maps','error');
  }catch(e){setMapStatus('microSD недоступна: '+e.message,'error');}
  const testSelect=document.getElementById('mbtilesTestMap');
  mapNames.forEach(name=>{const o=document.createElement('option');o.value=name;o.textContent=name;testSelect.appendChild(o);});
  const world=mapNames.find(n=>/world/i.test(n));const defaultValue=world?'sd:'+world:'online:osm';select.value=defaultValue;if(await ensureLeaflet()){initMap();await selectMap(defaultValue);}
}
async function loadSelectedMap(){
  const value=document.getElementById('mapSelect').value;
  if(!value){setMapStatus('Сначала выберите карту','error');return;}
  await selectMap(value);
}
function setMapStatus(message,state=''){
  const el=document.getElementById('mapStatus');el.textContent=message;el.className='map-status'+(state?' '+state:'');
}
function disableGpsFollow(){if(!followGps)return;followGps=false;document.getElementById('followStatus').textContent='Ручное управление картой';}
function centerOnGps(){
  if(!currentCoordinatesAvailable){setMapStatus('Координаты ещё не поступали из NMEA','error');return;}
  followGps=true;document.getElementById('followStatus').textContent='Автоцентрирование включено';
  if(map)map.setView([currentLat,currentLng],Math.max(map.getZoom(),14),{animate:false});
}
function latLonToTile(lat,lon,z){
  const n=Math.pow(2,z),r=Math.max(-85.05112878,Math.min(85.05112878,lat))*Math.PI/180;
  return {x:Math.floor((lon+180)/360*n),y:Math.floor((1-Math.asinh(Math.tan(r))/Math.PI)/2*n)};
}
function attachTileProgress(layer,token,label){
  mapTilesLoaded=0;mapTilesFailed=0;
  let pending=0;const progress=document.getElementById('tileProgress'),progressText=document.getElementById('tileProgressText');const paint=()=>{if(token!==mapLoadToken)return;const total=mapTilesLoaded+mapTilesFailed+pending;progress.max=Math.max(1,total);progress.value=mapTilesLoaded+mapTilesFailed;progressText.textContent='Загружено '+mapTilesLoaded+' · в очереди '+pending+' · ошибок '+mapTilesFailed;setMapStatus(label+': '+progressText.textContent,'loading');};layer.on('loading',()=>{pending=0;mapTilesLoaded=0;mapTilesFailed=0;paint();});layer.on('tileloadstart',()=>{pending++;paint();});layer.on('tileload',()=>{if(token!==mapLoadToken)return;pending=Math.max(0,pending-1);mapTilesLoaded++;paint();});
  layer.on('tileerror',async (event)=>{if(token!==mapLoadToken)return;pending=Math.max(0,pending-1);mapTilesFailed++;if(mapTilesFailed>1){setMapStatus(label+': загружено '+mapTilesLoaded+' · без тайла (вне покрытия): '+mapTilesFailed,'loading');return;}let detail='ошибка тайла';try{const response=await fetch(event.tile.src,{cache:'no-store'});const body=await response.text();detail='HTTP '+response.status+' '+body.slice(0,220);}catch(e){detail=e.message||String(e);}if(token===mapLoadToken)setMapStatus(label+': '+detail,'error');});
  layer.on('load',()=>{if(token!==mapLoadToken)return;pending=0;progress.max=Math.max(1,mapTilesLoaded+mapTilesFailed);progress.value=mapTilesLoaded;progressText.textContent='Завершено: '+mapTilesLoaded+' тайлов · ошибок '+mapTilesFailed;setMapStatus(mapTilesFailed?'Загрузка завершена с ошибками: '+mapTilesFailed+' тайлов не получено':'Карта загружена: '+mapTilesLoaded+' тайлов видимой области',mapTilesFailed?'error':'good');});
}
async function selectMap(value){
  const token=++mapLoadToken;currentMap=value;setMapStatus('Подготовка карты…','loading');
  if(tileLayer && map){map.removeLayer(tileLayer);tileLayer=null;}
  if(map){map.remove();map=null;gpsMarker=null;}
  setMapStatus('Загружаю библиотеку карты…','loading');
  if(!await ensureLeaflet()||token!==mapLoadToken){if(token===mapLoadToken)initMap();return;}
  if(!initMap())return;
  if(value==='online:osm')tileLayer=L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'});

  else if(value.startsWith('sd:')){
    const name=value.substring(3);
    try{
      const infoResponse=await fetch('/maps/info?map='+encodeURIComponent(name)+'&ts='+Date.now(),{cache:'no-store'});
      const infoBody=await infoResponse.text();let info={};try{info=JSON.parse(infoBody);}catch(e){}
      if(!infoResponse.ok||!info.db_open)throw new Error((info.sqlite_error||info.error||('HTTP '+infoResponse.status+' '+infoBody.slice(0,180)))+' | SQLite rc='+(info.sqlite_rc??'—'));
      const nativeZoom=Number.isInteger(info.max_zoom)?info.max_zoom:(Number.isInteger(info.maxZoom)?info.maxZoom:16);const nativeMin=Number.isInteger(info.min_zoom)&&info.min_zoom>0&&info.min_zoom<=nativeZoom?info.min_zoom:0;
      tileLayer=L.tileLayer('/tiles?map='+encodeURIComponent(name)+'&z={z}&x={x}&y={y}',{minZoom:0,maxZoom:19,minNativeZoom:nativeMin,maxNativeZoom:nativeZoom,errorTileUrl:BLANK_TILE,keepBuffer:1,updateWhenZooming:false,updateWhenIdle:true,attribution:'Offline map'});
      setMapStatus('MBTiles открыта · '+(Number(info.file_size_bytes||0)/1048576).toFixed(0)+' MiB · zoom '+info.min_zoom+'–'+nativeZoom+(info.warning?' · ⚠ '+info.warning:''),info.warning?'error':'good');
    }catch(e){setMapStatus('Ошибка MBTiles: '+e.message,'error');return;}
  }
  if(!tileLayer){setMapStatus('Неизвестный тип карты','error');return;}
  attachTileProgress(tileLayer,token,value.startsWith('sd:')?'Офлайн '+value.substring(3):'Онлайн '+value.substring(7).toUpperCase());
  tileLayer.addTo(map);
  if(currentCoordinatesAvailable)updateLeafletPoint(currentLat,currentLng);else {const selectedMapName=value.startsWith('sd:')?value.substring(3):'';map.setView([59.9386,30.3141],/world/i.test(selectedMapName)?8:11,{animate:false});}
  setMapStatus(value.startsWith('sd:')?'Открываю MBTiles '+value.substring(3)+'…':'Запрашиваю тайлы вокруг точки…','loading');
}
function updateLeafletPoint(lat,lng){
  if(!map)return;
  if(!gpsMarker)gpsMarker=L.circleMarker([lat,lng],{radius:6,color:'#ffffff',weight:2,fillColor:currentFix?'#1677ff':'#f59e0b',fillOpacity:1}).addTo(map);
  else {gpsMarker.setLatLng([lat,lng]);gpsMarker.setStyle({fillColor:currentFix?'#1677ff':'#f59e0b'});}
  if(followGps)map.panTo([lat,lng],{animate:false});
}
function removeCurrentMapPoint(){
  if(gpsMarker && map){map.removeLayer(gpsMarker);gpsMarker=null;}
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
async function refreshSavedPoints(){
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
 if(!currentCoordinatesAvailable){alert('Координаты ещё не поступали из NMEA');return;}
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
  const hasCoords=!!d.coordinates_available;
  const lat=hasCoords?Number(d.lat):0;
  const lng=hasCoords?Number(d.lng):0;
  document.getElementById('latValue').textContent=lat.toFixed(6);
  document.getElementById('lngValue').textContent=lng.toFixed(6);
  document.getElementById('satValue').textContent=String(d.sat??0);
  document.getElementById('satVisibleValue').textContent=String(d.sat_visible??0);
document.getElementById('gsvSnrValue').textContent=d.gsv_snr_count?Number(d.gsv_snr_avg).toFixed(1)+' дБ ('+d.gsv_snr_count+')':'—';
  document.getElementById('coords').textContent=hasCoords?lat.toFixed(6)+', '+lng.toFixed(6):'Координаты ещё не получены';
  currentFix=!!d.valid; currentCoordinatesAvailable=hasCoords;
  if(hasCoords){currentLat=lat;currentLng=lng;}
  const badge=document.getElementById('fixBadge');badge.textContent=d.valid?'GPS FIX':'НЕТ ФИКСА';badge.className='fix-badge '+(d.valid?'has-fix':'no-fix');
  if(d.fix_timeout_s && document.activeElement!==document.getElementById('fixTimeoutSeconds')){
    document.getElementById('fixTimeoutSeconds').value=d.fix_timeout_s;
  }
  if(!d.valid){
    document.getElementById('status').textContent=hasCoords?'Координаты из NMEA ('+(d.coordinate_source||'—')+') без подтверждённого фикса · возраст '+Math.round((d.coordinate_age_ms||0)/1000)+' с':'Нет фикса и координаты ещё не переданы';
    if(hasCoords && map) updateLeafletPoint(lat,lng); else if(!hasCoords) removeCurrentMapPoint();
    return;
  }
  document.getElementById('status').textContent='Фикс подтверждён · используется: '+(d.sat??0)+' | видно: '+(d.sat_visible??0)+' | HDOP: '+d.hdop+' | обновлено '+Math.round(d.age_ms/1000)+' с назад';
  if(map) updateLeafletPoint(lat,lng);
}catch(e){}
}
async function refreshNetworkStatus(){
 try{
 if(!timezoneConfigured){const localOffset=-new Date().getTimezoneOffset();await fetch('/network/timezone?offset='+localOffset,{method:'POST'});timezoneConfigured=true;}
 const d=await (await fetch('/network/status?ts='+Date.now(),{cache:'no-store'})).json();
 document.getElementById('networkStatus').textContent=(d.sta_connected?'Wi-Fi подключён: '+d.ssid+' · IP '+d.ip:'Wi-Fi не подключён')+' · NTP: '+(d.time_synced?'местное время '+d.local+' (UTC '+d.utc+')':'ожидание синхронизации');
 if(d.ssid)document.getElementById('wifiSsid').value=d.ssid;
 }catch(e){document.getElementById('networkStatus').textContent='Ошибка сети: '+e.message;}
}
async function scanWifiNetworks(){
 if(wifiScanBusy)return;wifiScanBusy=true;
 const status=document.getElementById('networkStatus'),select=document.getElementById('wifiNetworks');
 status.textContent='Сканирую доступные Wi-Fi сети…';
 try{
  const start=await (await fetch('/wifi/scan',{method:'POST'})).json();
  if(!start.started && start.status!==0)throw new Error(start.error||'Не удалось запустить сканирование');
  let result=null;
  for(let i=0;i<25;i++){
   await new Promise(resolve=>setTimeout(resolve,700));
   result=await (await fetch('/wifi/scan/status?ts='+Date.now(),{cache:'no-store'})).json();
   if(result.done||result.failed)break;
  }
  if(!result||!result.done)throw new Error(result&&result.failed?'Ошибка сканирования':'Сканирование заняло слишком много времени');
  select.innerHTML='';
  (result.networks||[]).sort((a,b)=>b.rssi-a.rssi).forEach(n=>{
   const option=document.createElement('option');option.value=n.ssid;option.textContent=(n.ssid||'(скрытая сеть)')+' · '+n.rssi+' dBm · канал '+n.channel+' · '+(n.secure?'защищена':'открытая');select.appendChild(option);
  });
  status.textContent='Найдено сетей: '+result.count+'. Выберите сеть из списка или введите SSID вручную.';
 }catch(e){status.textContent='Сканирование Wi-Fi: '+e.message;}
 finally{wifiScanBusy=false;}
}
function selectWifiNetwork(){const select=document.getElementById('wifiNetworks');if(select.value)document.getElementById('wifiSsid').value=select.value;}
async function saveWifiNetwork(){
 const ssid=document.getElementById('wifiSsid').value, password=document.getElementById('wifiPassword').value;
 const status=document.getElementById('networkStatus');status.textContent='Подключаюсь…';
 try{const body=new URLSearchParams({ssid,password});const r=await fetch('/network/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const d=await r.json();if(!r.ok)throw new Error(d.error||('HTTP '+r.status));status.textContent=d.message;setTimeout(refreshNetworkStatus,3000);}
 catch(e){status.textContent='Ошибка: '+e.message;}
}
async function refreshSatellites(){
 try{
 const d=await (await fetch('/satellites?ts='+Date.now(),{cache:'no-store'})).json(), sats=d.satellites||[];
 const svg=document.getElementById('skyPlot'), ns='http://www.w3.org/2000/svg';
 svg.querySelectorAll('.sat-point').forEach(e=>e.remove());
 sats.forEach(s=>{
 const r=140*(90-Number(s.elevation))/90,a=Number(s.azimuth)*Math.PI/180,x=160+r*Math.sin(a),y=160-r*Math.cos(a);
 const c=document.createElementNS(ns,'circle');c.setAttribute('cx',x);c.setAttribute('cy',y);c.setAttribute('r','7');c.setAttribute('fill',s.snr==null?'#94a3b8':s.snr>=38?'#22c55e':s.snr>=28?'#eab308':'#ef4444');c.setAttribute('stroke',s.used?'#fff':'#111827');c.setAttribute('stroke-width',s.used?'2.5':'1');c.setAttribute('class','sat-point');
 const title=document.createElementNS(ns,'title');title.textContent=s.system+s.prn+' · '+s.elevation+'° / '+s.azimuth+'° · SNR '+(s.snr??'—')+' · '+(s.used?'используется':'виден');c.appendChild(title);svg.appendChild(c);
 const label=document.createElementNS(ns,'text');label.setAttribute('x',x+8);label.setAttribute('y',y-7);label.setAttribute('fill','#fff');label.setAttribute('font-size','9');label.setAttribute('class','sat-point');label.textContent=s.prn;svg.appendChild(label);
 });
 document.getElementById('satSummary').textContent='Видно: '+sats.length+' · Используется по GGA: '+d.used_gga;
 const bars=document.getElementById('snrBars');bars.innerHTML='';
 const max=50,left=42,right=350,top=18,bottom=172,plotH=bottom-top,plotW=right-left;
 [0,10,20,30,40,50].forEach(v=>{const y=bottom-v/max*plotH;const line=document.createElementNS(ns,'line');line.setAttribute('x1',left);line.setAttribute('x2',right);line.setAttribute('y1',y);line.setAttribute('y2',y);line.setAttribute('stroke','#64748b');bars.appendChild(line);const t=document.createElementNS(ns,'text');t.setAttribute('x',left-7);t.setAttribute('y',y+3);t.setAttribute('fill','#cbd5e1');t.setAttribute('font-size','10');t.setAttribute('text-anchor','end');t.textContent=v;bars.appendChild(t);});
 const axis=document.createElementNS(ns,'text');axis.setAttribute('x','5');axis.setAttribute('y','12');axis.setAttribute('fill','#cbd5e1');axis.setAttribute('font-size','10');axis.textContent='SNR, дБ';bars.appendChild(axis);
 const bw=sats.length?Math.min(22,plotW/sats.length):18;sats.forEach((sat,i)=>{const x=left+(i+0.5)*(plotW/Math.max(1,sats.length)),v=sat.snr==null?0:Number(sat.snr),h=Math.max(0,Math.min(max,v))/max*plotH;const rect=document.createElementNS(ns,'rect');rect.setAttribute('x',x-bw/2);rect.setAttribute('y',bottom-h);rect.setAttribute('width',Math.max(4,bw-3));rect.setAttribute('height',h);rect.setAttribute('rx','2');rect.setAttribute('fill',sat.snr==null?'#64748b':v>=38?'#22c55e':v>=28?'#eab308':'#ef4444');bars.appendChild(rect);const t=document.createElementNS(ns,'text');t.setAttribute('x',x);t.setAttribute('y','188');t.setAttribute('fill','#cbd5e1');t.setAttribute('font-size','9');t.setAttribute('text-anchor','middle');t.textContent=sat.prn;bars.appendChild(t);});
 const list=document.getElementById('satelliteList');
 if(!sats.length){list.textContent='Спутники пока не получены. Ожидание GSV.';return;}
 list.innerHTML='<div style="display:grid;grid-template-columns:45px 40px 50px 50px 45px 1fr;gap:5px;font-size:12px"><b>Сист.</b><b>PRN</b><b>Выс.</b><b>Азим.</b><b>SNR</b><b>Статус</b>'+sats.map(s=>'<span>'+s.system+'</span><span>'+s.prn+'</span><span>'+s.elevation+'°</span><span>'+s.azimuth+'°</span><span>'+(s.snr??'—')+'</span><span>'+(s.used?'Используется':'Виден')+'</span>').join('')+'</div>';
 }catch(e){document.getElementById('satSummary').textContent='Ошибка: '+e.message;}
}
async function loadFirmwareInfo(){
 const out=document.getElementById('firmwareInfo');out.textContent='Чтение данных устройства…';
 try{
  const r=await fetch('/firmware/info?ts='+Date.now(),{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);
  const d=await r.json();
  const rows=[['Версия прошивки',d.firmware_version],['Дата сборки',d.build_date+' '+d.build_time],['Git commit',d.git_commit],['SHA-256 ELF',d.elf_sha256],['Чип',d.chip_model+' rev.'+d.chip_revision+' / '+d.cores+' ядра'],['Частота CPU',d.cpu_mhz+' МГц'],['Причина последнего сброса',d.reset_reason],['Время после запуска',Math.floor(d.uptime_ms/1000)+' с']];
  out.textContent=rows.map(x=>x[0]+': '+(x[1]===undefined||x[1]===null?'—':x[1])).join('\n');
 }catch(e){out.textContent='Не удалось получить сведения: '+e.message;}
}
function downloadCurrentFirmware(){window.location.href='/firmware/download';}
function uploadFirmware(){
 const input=document.getElementById('firmwareFile'),status=document.getElementById('firmwareUploadStatus'),progress=document.getElementById('firmwareProgress');
 if(!input.files||!input.files.length){status.textContent='Сначала выберите BIN или ZIP-файл';return;}
 const file=input.files[0],fileName=file.name.toLowerCase();if(!fileName.endsWith('.bin')&&!fileName.endsWith('.zip')){status.textContent='Выберите файл прошивки .bin или .zip';return;}
 if(!confirm('Установить '+file.name+' ('+file.size+' байт)? ESP32 перезагрузится.'))return;
 const data=new FormData();data.append('firmware',file,file.name);
 const xhr=new XMLHttpRequest();xhr.open('POST','/firmware/update');xhr.timeout=180000;
 xhr.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);progress.value=p;status.textContent='Передача: '+p+'% ('+e.loaded+' / '+e.total+' байт)';}};
 xhr.onload=()=>{let msg=xhr.responseText;try{const d=JSON.parse(msg);msg=d.message||d.error||msg;}catch(e){}if(xhr.status>=200&&xhr.status<300){progress.value=100;status.textContent='Прошивка проверена и записана. ESP32 перезагружается; переподключитесь через 15–30 секунд.';}else status.textContent='Ошибка обновления HTTP '+xhr.status+': '+msg;};
 xhr.onerror=()=>status.textContent='Потеряна связь во время загрузки. Проверьте устройство.';
 xhr.ontimeout=()=>status.textContent='Истекло время ожидания загрузки.';
 progress.value=0;status.textContent='Подготовка обновления…';xhr.send(data);
}
async function rebootDevice(){
 const out=document.getElementById('rebootStatus');
 if(!confirm('Перезагрузить ESP32-S3?'))return;
 out.textContent='Отправляю команду…';
 try{const r=await fetch('/system/reboot',{method:'POST'});const body=await r.text();if(!r.ok)throw new Error('HTTP '+r.status+': '+body);out.textContent='Команда принята. ESP32 перезагружается; точка доступа временно исчезнет.';}
 catch(e){out.textContent='Ответ потерян или ошибка: '+e.message+'. Подождите 10–20 секунд и подключитесь снова.';}
}
async function testCoordinates(){
 const out=document.getElementById('coordinateTestResult');out.textContent='Проверяю…';
 try{const r=await fetch('/coords?ts='+Date.now());if(!r.ok)throw new Error('HTTP '+r.status);const d=await r.json();out.textContent=d.valid?'PASS — актуальный фикс\nШирота: '+Number(d.lat).toFixed(7)+'\nДолгота: '+Number(d.lng).toFixed(7)+'\nСпутников в решении: '+d.sat+'\nВидно: '+d.sat_visible+'\nHDOP: '+d.hdop+'\nВозраст данных: '+Math.round(d.age_ms/1000)+' с':'FAIL — фикса нет или он устарел';}
 catch(e){out.textContent='ОШИБКА: '+e.message;}
}
async function downloadMbtilesLogs(){
 try{
  const r=await fetch('/maps/logs?ts='+Date.now(),{cache:'no-store'});
  const body=await r.text();if(!r.ok)throw new Error('HTTP '+r.status+': '+body);
  const data=JSON.parse(body);
  const textLog='ESP32 MBTiles diagnostics\nSD mounted: '+data.sd_ready+'\nMap open: '+data.map_open+'\nFile size: '+data.file_size_bytes+'\nSQLite rc: '+data.sqlite_rc+'\nLast error: '+data.sqlite_error+'\n\n'+(data.log||'(no log entries)');
  const url=URL.createObjectURL(new Blob([textLog],{type:'text/plain;charset=utf-8'}));
  const a=document.createElement('a');a.href=url;a.download='mbtiles-diagnostics.txt';document.body.appendChild(a);a.click();a.remove();URL.revokeObjectURL(url);
 }catch(e){document.getElementById('mbtilesTestResult').textContent+='\nНе удалось выгрузить журнал: '+e.message;}
}
async function testMbtiles(){
 const out=document.getElementById('mbtilesTestResult'),name=document.getElementById('mbtilesTestMap').value;
 if(!name){out.textContent='Сначала выберите офлайн-карту';return;}out.textContent='Проверяю открытие базы…';
 try{
  const ir=await fetch('/maps/info?map='+encodeURIComponent(name)+'&ts='+Date.now(),{cache:'no-store'});
  const infoText=await ir.text();let info={};try{info=JSON.parse(infoText);}catch(e){}
  if(!ir.ok||!info.db_open)throw new Error((info.sqlite_error||info.error||('Метаданные HTTP '+ir.status+' '+infoText.slice(0,160)))+' | SQLite rc='+(info.sqlite_rc??'—'));
  const z=Math.max(0,Math.min(Number(info.maxZoom)||9,name==='world'?9:11));
  const lat=currentFix?currentLat:54.9227,lon=currentFix?currentLng:43.3448,t=latLonToTile(lat,lon,z);
  const tr=await fetch('/tiles?map='+encodeURIComponent(name)+'&z='+z+'&x='+t.x+'&y='+t.y+'&ts='+Date.now(),{cache:'no-store'});
  if(!tr.ok){
    const errorBody=await tr.text();let errorInfo={};try{errorInfo=JSON.parse(errorBody);}catch(e){}
    let diagnostic='База обнаружена, но тайл не прочитан: HTTP '+tr.status+'\nОтвет ESP32: '+errorBody.slice(0,1200);
    if(errorInfo.sqlite_rc!==undefined)diagnostic+='\nSQLite rc: '+errorInfo.sqlite_rc;
    try{const lr=await fetch('/maps/logs?ts='+Date.now(),{cache:'no-store'});const ld=await lr.json();diagnostic+='\nПоследняя ошибка: '+(ld.sqlite_error||'—')+'\nЖурнал:\n'+(ld.log||'пусто');}catch(logError){diagnostic+='\nНе удалось получить журнал: '+logError.message;}
    throw new Error(diagnostic);
  }
  if(tr.headers.get('X-Tile-Missing'))throw new Error('Тайл отсутствует в базе для z/x/y — точка вне покрытия');const blob=await tr.blob();if(!blob.size)throw new Error('Получен пустой тайл');if(!blob.type.startsWith('image/'))throw new Error('Неизвестный формат тайла: '+blob.type);
  out.textContent='PASS — MBTiles открыта, тайл прочитан\nКарта: '+name+'\nZoom: '+z+' | X: '+t.x+' | Y: '+t.y+'\nФормат: '+blob.type+'\nРазмер: '+blob.size+' байт\nКоординаты: '+lat.toFixed(6)+', '+lon.toFixed(6);
 }catch(e){out.textContent='FAIL — '+e.message+'\nДля выгрузки нажмите «Скачать журнал MBTiles».';}
}
async function changeUartBaud(value){
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
  if(ubxPage==='msg'&&p.length>=8){set('msgI2c',p[2]);set('msgUart1',p[3]);set('msgUart2',p[4]);set('msgUsb',p[5]);set('msgSpi',p[6]);}
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
async function refreshUart(){
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
'\n\nАКСЕЛЕРОМЕТР (m/s²)\nX: '+f(d.ax)+' | Y: '+f(d.ay)+' | Z: '+f(d.az)+'\nПолный модуль: '+f(d.accel_magnitude)+' m/s² (в покое около 9.81 — гравитация)'+
'\n\nГИРОСКОП (°/s)\nX: '+f(d.gx)+' | Y: '+f(d.gy)+' | Z: '+f(d.gz)+'\nМодуль угловой скорости: '+f(d.gyro_magnitude)+' °/s'+
'\n\nРАСЧЁТНАЯ ОРИЕНТАЦИЯ\nRoll: '+f(d.roll_deg,2)+'° | Pitch: '+f(d.pitch_deg,2)+'°\nYaw: недоступен без магнитометра/внешней привязки'+
'\n\nКАЛИБРОВКА ГИРОСКОПА\nX: '+f(d.gyro_offset_x,4)+' | Y: '+f(d.gyro_offset_y,4)+' | Z: '+f(d.gyro_offset_z,4)+' °/s\nСтатус: '+(d.calibrated?'смещение сохранено':'смещение не задано')+
'\n\nОбновление опроса: '+d.sample_period_ms+' ms';
}catch(e){document.getElementById('imuStatus').textContent='Ошибка чтения IMU: '+e.message;}
}

async function refreshAi(){
  try{
    const d=await (await fetch('/ai/status?ts='+Date.now())).json();
    const labels={learning:'Накопление профиля',normal:'Норма',degrading:'Качество ухудшается',anomaly:'Статистическое отклонение',integrity_warning:'Несоответствие скорости и перемещения GNSS',spoofing_suspected:'Подозрение на спуфинг',position_deviation_with_weak_signal:'Отклонение позиции при слабом сигнале',interference_possible:'Возможны помехи GNSS',inside_reference:'В контрольной зоне',no_fix:'Нет фикса',limited_data:'Недостаточно данных',unknown:'Неизвестно'};
    const motion={moving:'Движение (по GNSS)',vibration:'Вибрация / динамика',rotation:'Вращение',stationary_or_low_dynamics:'Покой или низкая динамика',imu_unavailable:'IMU недоступен',uncertain:'Неопределённо',unknown:'Неизвестно'};
    document.getElementById('aiStatus').textContent=d.reference_enabled?'Контрольная зона активна · радиус '+Number(d.reference_radius_m).toFixed(0)+' м':'Модель активна · контрольная точка не задана';
    const refStatus=document.getElementById('referenceStatus');if(refStatus){refStatus.textContent=d.reference_enabled?'Задано · '+Number(d.reference_lat).toFixed(6)+', '+Number(d.reference_lon).toFixed(6)+' · радиус '+Number(d.reference_radius_m).toFixed(0)+' м':'Контрольная точка не задана';if(d.reference_enabled){document.getElementById('trainRefLat').value=d.reference_lat;document.getElementById('trainRefLon').value=d.reference_lon;document.getElementById('trainRadius').value=d.reference_radius_m;}}
    document.getElementById('aiScore').textContent=d.quality_score>=0?d.quality_score+'/100':'—';
    document.getElementById('aiRisk').textContent=labels[d.risk]||d.risk;
    document.getElementById('aiSamples').textContent=String(d.samples);
    document.getElementById('aiAnomaly').textContent=d.anomaly?d.anomaly_code:'Не обнаружена';
    document.getElementById('aiMismatch').textContent=d.gnss_speed_position_mismatch?'Есть расхождение':'Нет явного расхождения';
    document.getElementById('aiMotion').textContent=motion[d.motion]||d.motion;
    document.getElementById('aiSnr').textContent=d.avg_gsv_snr>0?Number(d.avg_gsv_snr).toFixed(1)+' дБ ('+d.snr_satellites+' спутн.)':'Нет данных';
    document.getElementById('aiHdopTrend').textContent=Number(d.hdop_trend).toFixed(2);
    document.getElementById('aiSnrTrend').textContent=Number(d.snr_trend).toFixed(1);
    document.getElementById('aiSituation').textContent=d.situation||'Сбор телеметрии…';
    document.getElementById('aiVerdict').textContent=d.verdict||'Недостаточно данных для заключения';
    document.getElementById('aiReasons').textContent=d.reasons||'';
    const interval=document.getElementById('reportInterval'); if(interval && !interval.dataset.loaded){interval.value=String(d.report_interval_seconds||300);interval.dataset.loaded='1';}
    document.getElementById('situationLogStatus').textContent=d.situation_logging?'Журнал ведётся':'Журнал остановлен';
    const csvStatus=document.getElementById('aiLogStatus');if(csvStatus)csvStatus.textContent=d.logging_enabled?'Запись включена: /ai_dataset.csv':'Запись остановлена';
    document.getElementById('aiDetails').textContent='Модель: '+d.model_type+
      '\nКод отклонения: '+d.anomaly_code+
      '\nМаксимальное отклонение: '+Number(d.max_z_score).toFixed(2)+' σ'+
      '\nРазница скоростей: '+Number(d.speed_difference_mps).toFixed(1)+' м/с'+
      '\nHDOP: '+Number(d.hdop).toFixed(2)+' | спутников в решении: '+d.satellites_used+
      '\nПримечание: '+d.note;
  }catch(e){document.getElementById('aiStatus').textContent='Ошибка AI-статуса: '+e.message;}
}
async function uploadAiProfile(){
  const input=document.getElementById('aiProfileFile'),status=document.getElementById('aiProfileStatus');
  if(!input.files.length){status.textContent='Выберите JSON-файл';return;}
  try{const body=await input.files[0].text();const r=await fetch('/ai/profile',{method:'POST',headers:{'Content-Type':'application/json'},body});const d=await r.json();if(!r.ok)throw new Error(d.error||'Ошибка');status.textContent=d.message||'Профиль загружен';refreshAi();}
  catch(e){status.textContent='Ошибка импорта: '+e.message;}
}
async function saveAiReference(){
 const a=document.getElementById('trainRefLat').value,b=document.getElementById('trainRefLon').value,r=Number(document.getElementById('trainRadius').value||50),status=document.getElementById('referenceStatus');
 if(!a||!b||!Number.isFinite(Number(a))||!Number.isFinite(Number(b))||Math.abs(Number(a))>90||Math.abs(Number(b))>180||r<5||r>5000){status.textContent='Проверьте координаты и радиус 5–5000 м';return;}
 try{const q=new URLSearchParams({lat:a,lon:b,radius:String(r)});const res=await fetch('/ai/reference?'+q,{method:'POST'});const d=await res.json();if(!res.ok)throw new Error(d.error||'Ошибка');status.textContent=d.message||'Сохранено';refreshAi();}catch(e){status.textContent='Ошибка: '+e.message;}
}
async function clearAiReference(){try{const r=await fetch('/ai/reference/clear',{method:'POST'});if(!r.ok)throw new Error('HTTP '+r.status);document.getElementById('referenceStatus').textContent='Контрольная точка отключена';refreshAi();}catch(e){document.getElementById('referenceStatus').textContent='Ошибка: '+e.message;}}
async function setAiLogging(enabled){
  try{
    const r=await fetch('/ai/logging?enabled='+(enabled?'1':'0'),{method:'POST'});
    const d=await r.json();
    if(!r.ok) throw new Error(d.error||('HTTP '+r.status));
    document.getElementById('aiLogStatus').textContent=d.enabled?'Запись включена: /ai_dataset.csv':'Запись остановлена';
  }catch(e){document.getElementById('aiLogStatus').textContent='Ошибка записи CSV: '+e.message;}
}
async function saveReportInterval(){try{const v=Number(document.getElementById('reportInterval').value);const r=await fetch('/ai/report-interval?seconds='+v,{method:'POST'});const d=await r.json();if(!r.ok)throw new Error(d.error||'Ошибка');document.getElementById('situationLogStatus').textContent='Интервал установлен: '+d.seconds+' с';}catch(e){document.getElementById('situationLogStatus').textContent='Ошибка интервала: '+e.message;}}
async function setSituationLogging(enabled){try{const r=await fetch('/ai/situation-logging?enabled='+(enabled?'1':'0'),{method:'POST'});const d=await r.json();if(!r.ok)throw new Error(d.error||'Ошибка');document.getElementById('situationLogStatus').textContent=d.enabled?'Журнал включён':'Журнал остановлен';}catch(e){document.getElementById('situationLogStatus').textContent='Ошибка журнала: '+e.message;}}
async function resetAiModel(){
  try{const r=await fetch('/ai/reset',{method:'POST'});if(!r.ok)throw new Error('HTTP '+r.status);refreshAi();}
  catch(e){document.getElementById('aiStatus').textContent='Ошибка сброса: '+e.message;}
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

document.getElementById('mapSelect').addEventListener('change',()=>setMapStatus('Карта выбрана. Нажмите «Загрузить выбранную карту».'));
setInterval(()=>{if(!document.getElementById('gpsTab').classList.contains('hidden'))refreshGps();},1000);
setInterval(()=>{if(!document.getElementById('imuTab').classList.contains('hidden'))refreshImu();},200);
setInterval(()=>{if(!document.getElementById('testsTab').classList.contains('hidden'))refreshUart();},500);
setInterval(()=>{if(!document.getElementById('ubxTab').classList.contains('hidden'))refreshUbxStatus();},500);
setInterval(()=>{if(!document.getElementById('aiTab').classList.contains('hidden')||!document.getElementById('satTab').classList.contains('hidden'))refreshAi();},1000);
setInterval(()=>{if(!document.getElementById('satTab').classList.contains('hidden'))refreshSatellites();},1500);
setInterval(()=>{if(!document.getElementById('deviceTab').classList.contains('hidden'))refreshNetworkStatus();},5000);
refreshNetworkStatus();
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
  if (!name.length() || name.length() > 48 || !coordinatesAvailable) { server.send(400, "text/plain; charset=utf-8", "Нужно имя и полученные координаты"); return; }
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

void handleAiStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", aiJson());
}
void handleAiReset() {
  aiMonitor.reset();
  server.send(200, "application/json", R"({"ok":true})");
}
void handleAiLogging() {
  const bool enabled = server.arg("enabled") == "1";
  if (enabled && !sdReady) { server.send(503,"application/json",R"({"enabled":false,"error":"microSD недоступна"})"); return; }
  if(enabled){
    if(SD_MMC.exists("/ai_dataset.csv"))SD_MMC.remove("/ai_dataset.csv");
    File fresh=SD_MMC.open("/ai_dataset.csv",FILE_WRITE);
    if(!fresh){server.send(500,"application/json",R"({"enabled":false,"error":"Не удалось создать новый CSV"})");return;}
    fresh.println("millis,fix,lat,lon,speed_mps,hdop,sat_used,sat_visible,gsv_snr_avg,accel_magnitude,gyro_magnitude,profile_score,anomaly,anomaly_code,risk,motion");fresh.close();
  }
  aiLoggingEnabled=enabled;preferences.putBool("ai_log_on",enabled);
  JsonDocument doc;doc["enabled"]=enabled;String out;serializeJson(doc,out);server.send(200,"application/json",out);
}
void handleAiReference(){
  if(!server.hasArg("lat")||!server.hasArg("lon")||!server.hasArg("radius")){server.send(400,"application/json",R"({"ok":false,"error":"Нужны lat, lon и radius"})");return;}
  double la=server.arg("lat").toDouble(),lo=server.arg("lon").toDouble();float radius=server.arg("radius").toFloat();
  if(!isfinite(la)||!isfinite(lo)||fabs(la)>90||fabs(lo)>180||!isfinite(radius)||radius<5||radius>5000){server.send(400,"application/json",R"({"ok":false,"error":"Некорректные координаты или радиус"})");return;}
  aiReferenceLat=la;aiReferenceLon=lo;aiReferenceRadiusM=radius;aiReferenceEnabled=true;
  preferences.putBool("ai_ref_on",true);preferences.putDouble("ai_ref_lat",la);preferences.putDouble("ai_ref_lon",lo);preferences.putFloat("ai_ref_rad",radius);
  server.send(200,"application/json",R"({"ok":true,"message":"Контрольная точка сохранена"})");
}
void handleAiReferenceClear(){aiReferenceEnabled=false;preferences.putBool("ai_ref_on",false);server.send(200,"application/json",R"({"ok":true})");}
void handleAiProfileDownload() {
  JsonDocument doc; doc["format"]="gnss-ai-profile-v1"; doc["role"]=aiMonitor.profileRole();
  doc["feature_mask"]=aiMonitor.profileMask(); doc["threshold"]=aiMonitor.profileThreshold();
  doc["training_samples"]=aiMonitor.trainingSamples();
  JsonArray c=doc["center"].to<JsonArray>(), z=doc["scale"].to<JsonArray>();
  for(uint8_t i=0;i<6;i++){c.add(aiMonitor.profileCenter(i));z.add(aiMonitor.profileScale(i));}
  String out;serializeJson(doc,out);
  server.sendHeader("Content-Disposition","attachment; filename=gnss-ai-profile.json");
  server.send(200,"application/json; charset=utf-8",out);
}
void handleAiProfileUpload() {
  if(!sdReady){server.send(503,"application/json",R"({"ok":false,"error":"microSD недоступна"})");return;}
  JsonDocument doc;DeserializationError e=deserializeJson(doc,server.arg("plain"));
  if(e||doc["format"]!="gnss-ai-profile-v1"||!doc["center"].is<JsonArray>()||!doc["scale"].is<JsonArray>()||
     doc["center"].as<JsonArray>().size()!=6||doc["scale"].as<JsonArray>().size()!=6){
    server.send(400,"application/json",R"({"ok":false,"error":"Некорректный JSON-профиль"})");return;
  }
  float c[6],z[6];for(uint8_t i=0;i<6;i++){c[i]=doc["center"][i].as<float>();z[i]=doc["scale"][i].as<float>();}
  const char* role=doc["role"]|"normal_baseline";
  if(!aiMonitor.setProfile(doc["feature_mask"].as<uint8_t>(),c,z,doc["threshold"].as<float>(),
      doc["training_samples"].as<uint32_t>(),role)){
    server.send(400,"application/json",R"({"ok":false,"error":"Параметры профиля не прошли проверку"})");return;
  }
  if(SD_MMC.exists("/ai_profile.json")) SD_MMC.remove("/ai_profile.json");
  File file=SD_MMC.open("/ai_profile.json",FILE_WRITE);
  if(!file){server.send(500,"application/json",R"({"ok":false,"error":"Не удалось сохранить профиль"})");return;}
  serializeJson(doc,file);file.close();
  server.send(200,"application/json",R"({"ok":true,"message":"Профиль активирован и сохранён на microSD"})");
}
void loadAiProfileFromSd() {
  if(!sdReady||!SD_MMC.exists("/ai_profile.json"))return;
  File file=SD_MMC.open("/ai_profile.json",FILE_READ);if(!file)return;
  JsonDocument doc;DeserializationError e=deserializeJson(doc,file);file.close();if(e)return;
  if(!doc["center"].is<JsonArray>()||!doc["scale"].is<JsonArray>()||
     doc["center"].as<JsonArray>().size()!=6||doc["scale"].as<JsonArray>().size()!=6)return;
  float c[6],z[6];for(uint8_t i=0;i<6;i++){c[i]=doc["center"][i].as<float>();z[i]=doc["scale"][i].as<float>();}
  const char* role=doc["role"]|"normal_baseline";
  aiMonitor.setProfile(doc["feature_mask"].as<uint8_t>(),c,z,doc["threshold"].as<float>(),
    doc["training_samples"].as<uint32_t>(),role);
}

void handleAiDataset() {
  if (!sdReady || !SD_MMC.exists("/ai_dataset.csv")) {
    server.send(404, "text/plain; charset=utf-8", "CSV пока не создан");
    return;
  }
  File file = SD_MMC.open("/ai_dataset.csv", FILE_READ);
  if (!file) { server.send(500, "text/plain", "Не удалось открыть CSV"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=ai_dataset.csv");
  server.streamFile(file, "text/csv");
  file.close();
}
void handleSituationLogging() {
  const bool enabled = server.arg("enabled") == "1";
  if (enabled && !sdReady) { server.send(503, "application/json", R"({"enabled":false,"error":"microSD недоступна"})"); return; }
  if (enabled) {
    situationLogEntryNumber = 0;
    if (SD_MMC.exists("/situation_log.csv")) SD_MMC.remove("/situation_log.csv");
    File fresh = SD_MMC.open("/situation_log.csv", FILE_WRITE);
    if (!fresh) { server.send(500, "application/json", R"({"enabled":false,"error":"Не удалось создать журнал"})"); return; }
    fresh.println("№;Дата и время;Время UTC;Время синхронизировано;Координаты получены;Широта;Долгота;Источник координат;FIX;Спутников в решении;Спутников видно;HDOP;Средний уровень сигнала, дБ-Гц;Скорость, м/с;Статистическое отклонение;Несоответствие скорости и перемещения;Заключение;Причины");
    fresh.close(); lastSituationLogMs = 0;
  }
  situationLoggingEnabled = enabled;
  JsonDocument doc; doc["enabled"] = enabled; String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}
void handleReportInterval() {
  int seconds = server.arg("seconds").toInt();
  if (seconds < 10 || seconds > 1800) { server.send(400, "application/json", R"({"error":"Интервал должен быть от 10 до 1800 секунд"})"); return; }
  reportIntervalSeconds = (uint16_t)seconds;
  preferences.putUShort("report_sec", reportIntervalSeconds);
  JsonDocument doc; doc["seconds"] = reportIntervalSeconds; String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}
void handleSituationLogDownload() {
  if (!sdReady || !SD_MMC.exists("/situation_log.csv")) { server.send(404, "text/plain; charset=utf-8", "Журнал пока не создан"); return; }
  File file = SD_MMC.open("/situation_log.csv", FILE_READ);
  if (!file) { server.send(500, "text/plain", "Не удалось открыть журнал"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=situation_log.csv");
  server.streamFile(file, "text/csv"); file.close();
}
void writeSituationLog() {
  if (!situationLoggingEnabled || !sdReady) return;
  const unsigned long now=millis();
  if(lastSituationLogMs&&now-lastSituationLogMs<(unsigned long)reportIntervalSeconds*1000UL)return;
  lastSituationLogMs=now;
  File file=SD_MMC.open("/situation_log.csv",FILE_APPEND);if(!file)return;
  const time_t epoch=time(nullptr);const bool timeValid=epoch>1700000000;
  char localStamp[48]="Нет синхронизации",utcStamp[24]="—";
  if(timeValid){struct tm local,utc;localtime_r(&epoch,&local);gmtime_r(&epoch,&utc);strftime(localStamp,sizeof(localStamp),"%d.%m.%Y %H:%M:%S",&local);strftime(utcStamp,sizeof(utcStamp),"%d.%m.%Y %H:%M:%S",&utc);}
  if(file.size()==0)file.println("№;Дата и время;Время UTC;Время синхронизировано;Координаты получены;Широта;Долгота;Источник координат;FIX;Спутников в решении;Спутников видно;HDOP;Средний уровень сигнала, дБ-Гц;Скорость, м/с;Статистическое отклонение;Несоответствие скорости и перемещения;Заключение;Причины");
  String verdict=haveFix?"FIX подтверждён":"Нет FIX",reasons;
  if(aiMonitor.mismatch()){verdict="Подозрительный признак целостности";reasons="Скорость не согласуется с перемещением координат";}
  else if(aiMonitor.anomaly()){verdict="Статистическое отклонение";reasons=String("Отклонение: ")+aiMonitor.anomalyCode();}
  else if(!haveFix){reasons="Навигационный FIX отсутствует; анализ доступных сигналов и IMU продолжается; координатные проверки недоступны";}
  else if(satellites<4||hdop>3.0f){verdict="Низкое качество GNSS";reasons="Мало спутников в решении или высокий HDOP";}
  else if(String(aiMonitor.risk())=="degrading"){verdict="Качество GNSS ухудшается";reasons="Ухудшается тренд HDOP/SNR";}
  else if(!aiMonitor.trained()){verdict="Накопление профиля";reasons="Недостаточно интервалов для статистического сравнения";}
  else{verdict="Явных отклонений не выявлено";reasons="Статистический анализ доступных данных не выявил выраженных отклонений; это не доказывает отсутствие помех или спуфинга";}
  verdict.replace(';',',');reasons.replace(';',',');++situationLogEntryNumber;
  file.printf("%lu;%s;%s;%s;%s;",(unsigned long)situationLogEntryNumber,localStamp,utcStamp,timeValid?"Да":"Нет",coordinatesAvailable?"Да":"Нет");
  if(coordinatesAvailable)file.printf("%.7f;%.7f;%s;",lat,lng,coordinateSource.c_str());else file.print(";;;");
  file.printf("%s;%d;%d;%.2f;%.2f;%.2f;%s;%s;\"%s\";\"%s\"\n",haveFix?"Да":"Нет",satellites,satellitesVisible,hdop,gsvSnrCount?avgGsvSnr:NAN,gps.speed.isValid()?gps.speed.mps():NAN,aiMonitor.anomaly()?"Да":"Нет",aiMonitor.mismatch()?"Да":"Нет",verdict.c_str(),reasons.c_str());
  file.close();
}
void handleMaps() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", mapsJson());
}

static int mbtilesQueryInt(const char* sql, int fallback) {
  sqlite3_stmt* stmt = nullptr;
  int value = fallback;
  if (sqlite3_prepare_v2(mbtilesDb, sql, -1, &stmt, nullptr) == SQLITE_OK &&
      sqlite3_step(stmt) == SQLITE_ROW && sqlite3_column_type(stmt, 0) != SQLITE_NULL) {
    value = sqlite3_column_int(stmt, 0);
  }
  if (stmt) sqlite3_finalize(stmt);
  return value;
}

// Reads the zoom range once, when the archive is opened. Never uses MIN()/MAX() over the whole
// `tiles` table: on a multi-GB file that can scan the card for minutes, and because WebServer is
// single-threaded the whole web UI freezes meanwhile.
static void mbtilesReadZoomRange() {
  mbtilesMinZoom = mbtilesQueryInt("SELECT value FROM metadata WHERE name='minzoom' LIMIT 1", -1);
  mbtilesMaxZoom = mbtilesQueryInt("SELECT value FROM metadata WHERE name='maxzoom' LIMIT 1", -1);
  // Fallbacks walk the (zoom_level, tile_column, tile_row) index and stop at the first row.
  if (mbtilesMinZoom < 0 || mbtilesMinZoom > 22)
    mbtilesMinZoom = mbtilesQueryInt("SELECT zoom_level FROM tiles ORDER BY zoom_level ASC LIMIT 1", 0);
  if (mbtilesMaxZoom < 0 || mbtilesMaxZoom > 22)
    mbtilesMaxZoom = mbtilesQueryInt("SELECT zoom_level FROM tiles ORDER BY zoom_level DESC LIMIT 1", 16);
  if (mbtilesMinZoom < 0 || mbtilesMinZoom > mbtilesMaxZoom) mbtilesMinZoom = 0;
}

static void closeMbtiles() {
  if (mbtilesTileStmt) { sqlite3_finalize(mbtilesTileStmt); mbtilesTileStmt = nullptr; }
  if (mbtilesDb) { sqlite3_close(mbtilesDb); mbtilesDb = nullptr; }
  mbtilesOpenName = "";
}

bool openMbtiles(const String& mapName) {
  if (!sdReady) { mbtilesLastRc = SQLITE_CANTOPEN; mbtilesLastError = "microSD не смонтирована"; logMbtilesDiagnostic(mbtilesLastError); return false; }
  if (!validMapName(mapName)) { mbtilesLastRc = SQLITE_MISUSE; mbtilesLastError = "Недопустимое имя карты"; logMbtilesDiagnostic(mbtilesLastError); return false; }
  if (mbtilesDb && mbtilesOpenName == mapName && mbtilesTileStmt) return true;
  closeMbtiles();
  const String sdPath = "/maps/" + mapName + ".mbtiles";
  if (!SD_MMC.exists(sdPath)) { mbtilesLastRc = SQLITE_CANTOPEN; mbtilesLastError = "Файл не найден на microSD: " + sdPath; logMbtilesDiagnostic(mbtilesLastError); return false; }
  mbtilesFileSize = 0;
  { File f = SD_MMC.open(sdPath, FILE_READ); if (f) { mbtilesFileSize = f.size(); f.close(); } }
  const String dbPath = String(SD_MOUNT) + sdPath;
  logMbtilesDiagnostic("OPEN map=" + mapName + " SD=" + sdPath + " SQLite=" + dbPath + " size=" + String((unsigned long long)mbtilesFileSize));
  int rc = sqlite3_open_v2(dbPath.c_str(), &mbtilesDb, SQLITE_OPEN_READONLY, nullptr);
  if (rc != SQLITE_OK || !mbtilesDb) {
    mbtilesLastRc = rc;
    mbtilesLastError = "sqlite3_open_v2(" + dbPath + "): " + String(mbtilesDb ? sqlite3_errmsg(mbtilesDb) : "нет SQLite handle");
    Serial.printf("MBTiles open failed (%d): %s\n", rc, mbtilesLastError.c_str());
    logMbtilesDiagnostic("OPEN FAILED rc=" + String(rc) + " (" + String(sqlite3_errstr(rc)) + "): " + mbtilesLastError);
    if (mbtilesDb) sqlite3_close(mbtilesDb);
    mbtilesDb = nullptr; return false;
  }
  sqlite3_exec(mbtilesDb, "PRAGMA query_only=ON;", nullptr, nullptr, nullptr);
  sqlite3_exec(mbtilesDb, "PRAGMA mmap_size=0;", nullptr, nullptr, nullptr);
  sqlite3_exec(mbtilesDb, "PRAGMA cache_size=2;", nullptr, nullptr, nullptr);
  sqlite3_exec(mbtilesDb, "PRAGMA temp_store=FILE;", nullptr, nullptr, nullptr);
  rc = sqlite3_prepare_v2(mbtilesDb, "SELECT rowid FROM tiles WHERE zoom_level=?1 AND tile_column=?2 AND tile_row=?3 LIMIT 1", -1, &mbtilesTileStmt, nullptr);
  if (rc != SQLITE_OK) {
    mbtilesLastRc = rc; mbtilesLastError = "SQL prepare таблицы tiles: " + String(sqlite3_errmsg(mbtilesDb));
    Serial.printf("MBTiles query prepare failed (%d): %s\n", rc, mbtilesLastError.c_str());
    logMbtilesDiagnostic("PREPARE FAILED rc=" + String(rc) + " (" + String(sqlite3_errstr(rc)) + "): " + mbtilesLastError);
    sqlite3_close(mbtilesDb); mbtilesDb = nullptr; mbtilesTileStmt = nullptr; return false;
  }
  mbtilesReadZoomRange();
  mbtilesLastRc = SQLITE_OK; mbtilesLastError = "SQLite открыл базу, таблица tiles доступна";
  mbtilesOpenName = mapName; Serial.printf("MBTiles opened: %s (zoom %d-%d)\n", dbPath.c_str(), mbtilesMinZoom, mbtilesMaxZoom);
  logMbtilesDiagnostic("OPEN OK map=" + mapName + " zoom=" + String(mbtilesMinZoom) + "-" + String(mbtilesMaxZoom));
  return true;
}

const char* tileMimeType(const uint8_t* data, int size) {
  if (!data || size < 4) return "application/octet-stream";
  if (size >= 8 && data[0] == 0x89 && data[1] == 'P' && data[2] == 'N' && data[3] == 'G') return "image/png";
  if (data[0] == 0xFF && data[1] == 0xD8) return "image/jpeg";
  if (size >= 12 && data[0] == 'R' && data[1] == 'I' && data[2] == 'F' && data[3] == 'F' &&
      data[8] == 'W' && data[9] == 'E' && data[10] == 'B' && data[11] == 'P') return "image/webp";
  return "application/octet-stream";
}

// 1x1 transparent PNG returned for tiles that are simply absent from the archive (outside the
// coverage). A 404 made Leaflet report every such tile as a load error.
static const uint8_t BLANK_PNG[] = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x08, 0x06, 0x00, 0x00, 0x00, 0x1F, 0x15, 0xC4, 0x89, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x44, 0x41, 0x54, 0x78, 0xDA, 0x63, 0x64, 0x60, 0xF8, 0x5F, 0x0F, 0x00, 0x02, 0x87, 0x01, 0x80, 0xEB, 0x47, 0xBA, 0x92, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82};
static void sendBlankTile() {
  server.sendHeader("X-Tile-Missing", "1");
  server.sendHeader("Cache-Control", "public, max-age=3600");
  server.send_P(200, "image/png", reinterpret_cast<const char*>(BLANK_PNG), sizeof(BLANK_PNG));
}

void handleMbtilesTile(const String& mapName, int z, int x, int y) {
  if (!openMbtiles(mapName)) {
    JsonDocument errorDoc; errorDoc["error"] = mbtilesLastError; errorDoc["sqlite_rc"] = mbtilesLastRc;
    String errorBody; serializeJson(errorDoc, errorBody);
    server.send(500, "application/json; charset=utf-8", errorBody); return;
  }

  // MBTiles uses TMS rows; Leaflet requests XYZ rows.
  const int tmsY = ((1 << z) - 1) - y;
  sqlite3_reset(mbtilesTileStmt);
  sqlite3_clear_bindings(mbtilesTileStmt);
  sqlite3_bind_int(mbtilesTileStmt, 1, z);
  sqlite3_bind_int(mbtilesTileStmt, 2, x);
  sqlite3_bind_int(mbtilesTileStmt, 3, tmsY);

  const uint32_t heapBefore = ESP.getFreeHeap();
  const uint32_t largestBefore = ESP.getMaxAllocHeap();
  const int rc = sqlite3_step(mbtilesTileStmt);
  if (rc != SQLITE_ROW) {
    logMbtilesDiagnostic("SQLITE STEP memory free=" + String(heapBefore) + " largest=" + String(largestBefore) +
      " min_free=" + String(ESP.getMinFreeHeap()));
    const String detail = rc == SQLITE_DONE
      ? "Тайл отсутствует для этих z/x/y: точка может быть вне покрытия карты"
      : "sqlite3_step вернул код " + String(rc) + ": " + String(sqlite3_errmsg(mbtilesDb));
    if (rc != SQLITE_DONE) {
      mbtilesLastRc = rc; mbtilesLastError = detail;
      logMbtilesDiagnostic("TILE ROWID FAILED map=" + mapName + " z=" + String(z) + " x=" + String(x) +
        " yXYZ=" + String(y) + " yTMS=" + String(tmsY) + " rc=" + String(rc) +
        " (" + String(sqlite3_errstr(rc)) + "): " + String(sqlite3_errmsg(mbtilesDb)));
    }
    sqlite3_reset(mbtilesTileStmt);
    if (rc == SQLITE_DONE) { sendBlankTile(); return; }
    JsonDocument errorDoc; errorDoc["error"] = detail; errorDoc["sqlite_rc"] = rc;
    errorDoc["map"] = mapName; errorDoc["z"] = z; errorDoc["x"] = x; errorDoc["y_xyz"] = y; errorDoc["y_tms"] = tmsY;
    String errorBody; serializeJson(errorDoc, errorBody);
    server.send(500, "application/json; charset=utf-8", errorBody);
    return;
  }

  const sqlite3_int64 rowid = sqlite3_column_int64(mbtilesTileStmt, 0);
  sqlite3_reset(mbtilesTileStmt);

  // Incremental BLOB I/O avoids allocating the complete compressed tile in ESP32 heap.
  sqlite3_blob *tileBlob = nullptr;
  int blobRc = sqlite3_blob_open(mbtilesDb, "main", "tiles", "tile_data", rowid, 0, &tileBlob);
  if (blobRc != SQLITE_OK || !tileBlob) {
    mbtilesLastRc = blobRc;
    mbtilesLastError = "sqlite3_blob_open: " + String(sqlite3_errmsg(mbtilesDb));
    logMbtilesDiagnostic("BLOB OPEN FAILED rowid=" + String((long long)rowid) + " rc=" +
      String(blobRc) + " error=" + String(sqlite3_errmsg(mbtilesDb)));
    JsonDocument errorDoc; errorDoc["error"] = mbtilesLastError; errorDoc["sqlite_rc"] = blobRc;
    String errorBody; serializeJson(errorDoc, errorBody);
    server.send(500, "application/json; charset=utf-8", errorBody);
    return;
  }

  const int blobSize = sqlite3_blob_bytes(tileBlob);
  if (blobSize <= 0) {
    sqlite3_blob_close(tileBlob);
    sendBlankTile();
    return;
  }

  uint8_t chunkBuffer[1024];
  const int firstSize = blobSize < 16 ? blobSize : 16;
  blobRc = sqlite3_blob_read(tileBlob, chunkBuffer, firstSize, 0);
  if (blobRc != SQLITE_OK) {
    mbtilesLastRc = blobRc;
    mbtilesLastError = "sqlite3_blob_read header: " + String(sqlite3_errmsg(mbtilesDb));
    logMbtilesDiagnostic("BLOB HEADER READ FAILED rowid=" + String((long long)rowid) +
      " rc=" + String(blobRc) + " error=" + String(sqlite3_errmsg(mbtilesDb)));
    sqlite3_blob_close(tileBlob);
    JsonDocument errorDoc; errorDoc["error"] = mbtilesLastError; errorDoc["sqlite_rc"] = blobRc;
    String errorBody; serializeJson(errorDoc, errorBody);
    server.send(500, "application/json; charset=utf-8", errorBody);
    return;
  }

  const char* mime = tileMimeType(chunkBuffer, firstSize);
  logMbtilesDiagnostic("TILE BLOB OPEN map=" + mapName + " z=" + String(z) + " x=" + String(x) +
    " y=" + String(y) + " rowid=" + String((long long)rowid) + " bytes=" + String(blobSize) +
    " heap_free=" + String(ESP.getFreeHeap()) + " heap_largest=" + String(ESP.getMaxAllocHeap()));

  server.sendHeader("Cache-Control", "public, max-age=86400");
  server.setContentLength(blobSize);
  server.send(200, mime, "");

  auto& client = server.client();
  int offset = 0;
  int stalls = 0;
  while (offset < blobSize && client.connected()) {
    const int count = (blobSize - offset) > (int)sizeof(chunkBuffer)
      ? (int)sizeof(chunkBuffer) : (blobSize - offset);
    if (offset != 0) {
      blobRc = sqlite3_blob_read(tileBlob, chunkBuffer, count, offset);
      if (blobRc != SQLITE_OK) {
        logMbtilesDiagnostic("BLOB STREAM READ FAILED rowid=" + String((long long)rowid) +
          " offset=" + String(offset) + " rc=" + String(blobRc) +
          " error=" + String(sqlite3_errmsg(mbtilesDb)));
        break;
      }
    } else if (count > firstSize) {
      blobRc = sqlite3_blob_read(tileBlob, chunkBuffer, count, 0);
      if (blobRc != SQLITE_OK) break;
    }

    int sent = 0;
    while (sent < count && client.connected()) {
      const size_t written = client.write(chunkBuffer + sent, count - sent);
      if (written == 0) {
        if (++stalls > 200) break;
        delay(5);
        continue;
      }
      stalls = 0;
      sent += (int)written;
    }
    if (sent < count) break;
    offset += count;
  }
  sqlite3_blob_close(tileBlob);
}

void handleMbtilesLogs() {
  JsonDocument doc;
  doc["sd_ready"] = sdReady;
  doc["map_open"] = mbtilesOpenName;
  doc["file_size_bytes"] = mbtilesFileSize;
  doc["sqlite_rc"] = mbtilesLastRc;
  doc["sqlite_error"] = mbtilesLastError;
  doc["log"] = mbtilesDiagnosticLog;
  String out; serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", out);
}

void handleMapInfo() {
  if (!server.hasArg("map") || !validMapName(server.arg("map"))) {
    server.send(400, "application/json", R"({"error":"invalid_map"})"); return;
  }
  const String name = server.arg("map"), sdPath = "/maps/" + name + ".mbtiles";
  JsonDocument doc; doc["name"] = name; doc["sd_ready"] = sdReady;
  const bool exists = sdReady && SD_MMC.exists(sdPath); doc["file_exists"] = exists;
  uint64_t fileSize = 0;
  if (exists) { File f = SD_MMC.open(sdPath, FILE_READ); if (f) { fileSize = f.size(); f.close(); } }
  doc["file_size_bytes"] = fileSize;
  doc["large_file_support"] = true;
  doc["note"] = "Для MBTiles включено 64-битное файловое позиционирование SQLite.";
  const bool dbOk = openMbtiles(name); doc["db_open"] = dbOk;
  doc["sqlite_rc"] = mbtilesLastRc; doc["sqlite_error"] = mbtilesLastError;
  const int minZoom = dbOk ? mbtilesMinZoom : -1;
  const int maxZoom = dbOk ? mbtilesMaxZoom : 16;
  doc["min_zoom"] = minZoom; doc["max_zoom"] = maxZoom; doc["maxZoom"] = maxZoom;
  String out; serializeJson(doc, out);
  server.send(dbOk ? 200 : 500, "application/json; charset=utf-8", out);
}

// Leaflet is served from microSD (/lib/...) because the ESP32 Wi-Fi AP has no internet, so
// https://unpkg.com cannot be reached from the phone and the page used to hang waiting for it.
static const char* const LEAFLET_FILES[] = {
  "/lib/leaflet.js", "/lib/leaflet.css",
  "/lib/images/layers.png", "/lib/images/layers-2x.png",
  "/lib/images/marker-icon.png", "/lib/images/marker-icon-2x.png", "/lib/images/marker-shadow.png"
};

void handleLib() {
  const String uri = server.uri();
  bool allowed = false;
  for (const char* item : LEAFLET_FILES) if (uri == item) { allowed = true; break; }
  if (!allowed) { server.send(404, "text/plain", "Not found"); return; }
  if (!sdReady || !SD_MMC.exists(uri)) { server.send(404, "text/plain", "Leaflet file not found on microSD: " + uri); return; }
  File file = SD_MMC.open(uri, FILE_READ);
  if (!file) { server.send(500, "text/plain", "Cannot open " + uri); return; }
  const char* type = uri.endsWith(".js") ? "application/javascript" : uri.endsWith(".css") ? "text/css" : "image/png";
  server.sendHeader("Cache-Control", "public, max-age=604800");
  server.streamFile(file, type);
  file.close();
}

static bool isDigitsOnly(const String& value) {
  if (value.length() == 0 || value.length() > 8) return false;
  for (unsigned int i = 0; i < value.length(); i++) if (!isdigit((unsigned char)value[i])) return false;
  return true;
}

void handleTile() {
  if (!server.hasArg("map") || !server.hasArg("z") || !server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  const String mapName = server.arg("map");
  if (!validMapName(mapName)) {
    server.send(400, "text/plain", "Invalid map");
    return;
  }

  // z/x/y end up in a file path for legacy PNG folders, so accept plain integers only
  // (previously "../" sequences were passed straight into the path).
  const String zs = server.arg("z"), xs = server.arg("x"), ys = server.arg("y");
  if (!isDigitsOnly(zs) || !isDigitsOnly(xs) || !isDigitsOnly(ys)) {
    server.send(400, "text/plain", "Invalid tile coordinates");
    return;
  }
  const long zi = zs.toInt(), xi = xs.toInt(), yi = ys.toInt();
  if (zi > 22 || xi >= (1L << zi) || yi >= (1L << zi)) {
    server.send(400, "text/plain", "Invalid tile coordinates");
    return;
  }
  if (!sdReady) {
    server.send(503, "text/plain", "microSD not mounted");
    return;
  }

  // Prefer a single-file MBTiles archive; keep legacy XYZ PNG directories compatible.
  // When the archive is already open skip the FAT lookup: it costs a card access per tile.
  const bool alreadyOpen = mbtilesDb && mbtilesOpenName == mapName;
  if (alreadyOpen || SD_MMC.exists("/maps/" + mapName + ".mbtiles")) {
    handleMbtilesTile(mapName, (int)zi, (int)xi, (int)yi);
    return;
  }

  const String path = "/maps/" + mapName + "/" + String(zi) + "/" + String(xi) + "/" + String(yi) + ".png";

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

static String firmwareResetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "Power-on";
    case ESP_RST_EXT: return "External reset";
    case ESP_RST_SW: return "Software reset";
    case ESP_RST_PANIC: return "Panic / exception";
    case ESP_RST_INT_WDT: return "Interrupt watchdog";
    case ESP_RST_TASK_WDT: return "Task watchdog";
    case ESP_RST_WDT: return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep sleep";
    case ESP_RST_BROWNOUT: return "Brownout";
    case ESP_RST_SDIO: return "SDIO reset";
    default: return "Unknown (" + String((int)esp_reset_reason()) + ")";
  }
}

void handleSatellites() {
  JsonDocument doc;
  JsonArray items = doc["satellites"].to<JsonArray>();
  uint32_t now = millis();
  for (uint8_t i = 0; i < satelliteInfoCount; ++i) {
    SatelliteInfo& sat = satelliteInfo[i];
    if (now - sat.lastSeen > 12000) continue;
    JsonObject item = items.add<JsonObject>();
    item["system"] = sat.system; item["prn"] = sat.prn;
    item["elevation"] = sat.elevation; item["azimuth"] = sat.azimuth;
    if (sat.snrValid) item["snr"] = sat.snr; else item["snr"] = nullptr;
    item["used"] = sat.used; item["age_ms"] = now - sat.lastSeen;
  }
  doc["visible"] = items.size(); doc["used_gga"] = satellites;
  doc["receiver"] = "u-blox NEO-6M / GY-NEO-6M-V2";
  doc["spectrum_available"] = false;
  String out; serializeJson(doc, out); server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", out);
}

void handleWifiScanStart() {
  if (WiFi.getMode() != WIFI_AP_STA && WiFi.getMode() != WIFI_STA) WiFi.mode(WIFI_AP_STA);
  wifiScanStatus = WiFi.scanNetworks(true, true);
  JsonDocument doc; doc["started"] = wifiScanStatus == WIFI_SCAN_RUNNING; doc["status"] = wifiScanStatus;
  if (wifiScanStatus == WIFI_SCAN_FAILED) doc["error"] = "Не удалось запустить сканирование Wi-Fi";
  String out; serializeJson(doc, out); server.send(200, "application/json", out);
}

void handleWifiScanStatus() {
  const int16_t status = WiFi.scanComplete();
  JsonDocument doc;
  doc["done"] = status >= 0; doc["scanning"] = status == WIFI_SCAN_RUNNING; doc["failed"] = status == WIFI_SCAN_FAILED;
  if (status >= 0) {
    JsonArray networks = doc["networks"].to<JsonArray>();
    for (int i = 0; i < status; ++i) {
      JsonObject item = networks.add<JsonObject>();
      item["ssid"] = WiFi.SSID(i); item["rssi"] = WiFi.RSSI(i);
      item["channel"] = WiFi.channel(i); item["secure"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    }
    doc["count"] = status;
  }
  String out; serializeJson(doc, out); server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void applyTimezoneOffset(int offsetMinutes) {
  timezoneOffsetMinutes = constrain(offsetMinutes, -720, 840);
  const int absoluteMinutes = abs(timezoneOffsetMinutes);
  char tz[24];
  snprintf(tz, sizeof(tz), "UTC%c%d:%02d", timezoneOffsetMinutes >= 0 ? '-' : '+', absoluteMinutes / 60, absoluteMinutes % 60);
  setenv("TZ", tz, 1); tzset();
}

void handleNetworkTimezone() {
  if (!server.hasArg("offset")) { server.send(400, "application/json", R"({"ok":false,"error":"offset required"})"); return; }
  const int offset = server.arg("offset").toInt();
  if (offset < -720 || offset > 840) { server.send(400, "application/json", R"({"ok":false,"error":"timezone offset out of range"})"); return; }
  applyTimezoneOffset(offset); preferences.putInt("tz_min", timezoneOffsetMinutes);
  server.send(200, "application/json", R"({"ok":true})");
}

void handleNetworkConfig() {
  String ssid = server.arg("ssid"), password = server.arg("password"); ssid.trim();
  if (!ssid.length() || ssid.length() > 32 || password.length() > 63) {
    server.send(400, "application/json", R"({"ok":false,"error":"Укажите SSID и пароль до 63 символов"})"); return;
  }
  preferences.putString("wifi_ssid", ssid); preferences.putString("wifi_pass", password);
  WiFi.mode(WIFI_AP_STA); WiFi.begin(ssid.c_str(), password.c_str());
  ntpConfigured = false;
  server.send(200, "application/json", R"({"ok":true,"message":"Настройки сохранены; подключение выполняется"})");
}
void handleNetworkStatus() {
  JsonDocument doc;
  doc["sta_connected"] = WiFi.status() == WL_CONNECTED;
  doc["ssid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : preferences.getString("wifi_ssid", "");
  doc["ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  time_t epoch = time(nullptr); bool valid = epoch > 1700000000;
  doc["time_synced"] = valid;
  if (valid) {
    struct tm utc, local; char utcStamp[24], localStamp[32];
    gmtime_r(&epoch, &utc); localtime_r(&epoch, &local);
    strftime(utcStamp, sizeof(utcStamp), "%Y-%m-%dT%H:%M:%SZ", &utc);
    strftime(localStamp, sizeof(localStamp), "%Y-%m-%d %H:%M:%S", &local);
    doc["utc"] = utcStamp; doc["local"] = localStamp;
  }
  doc["timezone_offset_minutes"] = timezoneOffsetMinutes;
  String out; serializeJson(doc, out); server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleFirmwareInfo() {
  JsonDocument doc;
  const esp_app_desc_t* app = esp_app_get_description();
  const esp_partition_t* part = esp_ota_get_running_partition();
  esp_image_metadata_t meta = {};
  uint32_t imageSize = ESP.getSketchSize();
  if (part) {
    esp_partition_pos_t pos = {part->address, part->size};
    if (esp_image_get_metadata(&pos, &meta) == ESP_OK) imageSize = meta.image_len;
  }
  char hash[65] = {0};
  if (app) {
    for (int i=0;i<32;i++) snprintf(hash+i*2,3,"%02x",app->app_elf_sha256[i]);
    doc["app_version"]=app->version; doc["build_date"]=FIRMWARE_BUILD_DATE; doc["build_time"]=FIRMWARE_BUILD_TIME;
    doc["build_timestamp_utc"]=FIRMWARE_BUILD_TIMESTAMP;
    doc["project"]=app->project_name; doc["idf_version"]=app->idf_ver;
  }
  doc["firmware_version"]=FIRMWARE_VERSION; doc["git_commit"]=FIRMWARE_GIT_SHA; doc["elf_sha256"]=hash;
  doc["chip_model"]=ESP.getChipModel(); doc["chip_revision"]=ESP.getChipRevision();
  doc["cores"]=ESP.getChipCores(); doc["cpu_mhz"]=ESP.getCpuFreqMHz();
  doc["flash_size_bytes"]=ESP.getFlashChipSize(); doc["flash_speed_hz"]=ESP.getFlashChipSpeed();
  doc["flash_mode"]=(int)ESP.getFlashChipMode(); doc["psram_size_bytes"]=ESP.getPsramSize();
  doc["image_size_bytes"]=imageSize; doc["partition_label"]=part?part->label:"unknown";
  doc["partition_address"]=part?part->address:0; doc["partition_size_bytes"]=part?part->size:0;
  doc["free_heap_bytes"]=ESP.getFreeHeap(); doc["min_free_heap_bytes"]=ESP.getMinFreeHeap();
  doc["largest_heap_block_bytes"]=ESP.getMaxAllocHeap(); doc["free_sketch_bytes"]=ESP.getFreeSketchSpace();
  doc["sketch_md5"]=ESP.getSketchMD5(); doc["mac"]=WiFi.macAddress(); doc["sdk_version"]=ESP.getSdkVersion();
  doc["reset_reason"]=firmwareResetReason(); doc["uptime_ms"]=millis();
  String json; serializeJson(doc,json); server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json; charset=utf-8",json);
}

void handleFirmwareDownload() {
  const esp_partition_t* part=esp_ota_get_running_partition();
  if(!part){server.send(500,"text/plain","Running application partition not found");return;}
  esp_partition_pos_t pos={part->address,part->size};esp_image_metadata_t meta={};
  if(esp_image_get_metadata(&pos,&meta)!=ESP_OK||meta.image_len==0||meta.image_len>part->size||meta.image_len>0xFFFFFFFFULL){server.send(500,"text/plain","Cannot determine ZIP32 firmware image size");return;}
  char entryName[64];snprintf(entryName,sizeof(entryName),"ESP32-GNSS-v%s.bin",FIRMWARE_VERSION);const uint16_t nameLen=(uint16_t)strlen(entryName);uint8_t buffer[1024];uint32_t crc=0xFFFFFFFFUL;size_t offset=0;
  while(offset<meta.image_len){size_t count=min(sizeof(buffer),(size_t)(meta.image_len-offset));if(esp_partition_read(part,offset,buffer,count)!=ESP_OK){server.send(500,"text/plain","Flash read failed while calculating CRC32");return;}for(size_t i=0;i<count;i++){crc^=buffer[i];for(uint8_t bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1)?0xEDB88320UL:0);}offset+=count;}crc^=0xFFFFFFFFUL;
  const uint32_t imageSize=(uint32_t)meta.image_len;const uint32_t zipSize=30U+nameLen+imageSize+46U+nameLen+22U;char filename[96];snprintf(filename,sizeof(filename),"ESP32-GNSS-v%s.zip",FIRMWARE_VERSION);
  server.sendHeader("Content-Disposition",String("attachment; filename=\"")+filename+"\"");server.sendHeader("Cache-Control","no-store");server.setContentLength(zipSize);server.send(200,"application/zip","");
  WiFiClient client=server.client();auto put16=[&](uint16_t v){uint8_t q[2]={(uint8_t)v,(uint8_t)(v>>8)};client.write(q,2);};auto put32=[&](uint32_t v){uint8_t q[4]={(uint8_t)v,(uint8_t)(v>>8),(uint8_t)(v>>16),(uint8_t)(v>>24)};client.write(q,4);};
  put32(0x04034b50);put16(20);put16(0);put16(0);put16(0);put16(0);put32(crc);put32(imageSize);put32(imageSize);put16(nameLen);put16(0);client.write((const uint8_t*)entryName,nameLen);
  offset=0;while(offset<meta.image_len&&client.connected()){size_t count=min(sizeof(buffer),(size_t)(meta.image_len-offset));if(esp_partition_read(part,offset,buffer,count)!=ESP_OK)return;size_t sent=0;while(sent<count&&client.connected()){size_t n=client.write(buffer+sent,count-sent);if(!n){delay(2);continue;}sent+=n;}if(sent!=count)return;offset+=count;}
  put32(0x02014b50);put16(20);put16(20);put16(0);put16(0);put16(0);put16(0);put32(crc);put32(imageSize);put32(imageSize);put16(nameLen);put16(0);put16(0);put16(0);put16(0);put32(0);put32(0);client.write((const uint8_t*)entryName,nameLen);
  put32(0x06054b50);put16(0);put16(0);put16(1);put16(1);put32(46U+nameLen);put32(30U+nameLen+imageSize);put16(0);
}
void handleFirmwareUploadResult() {
  if(!firmwareUploadOk){
    JsonDocument doc;doc["ok"]=false;doc["error"]=firmwareUploadError.length()?firmwareUploadError:"Firmware upload failed";
    String body;serializeJson(doc,body);server.send(500,"application/json; charset=utf-8",body);return;
  }
  JsonDocument doc;doc["ok"]=true;doc["message"]="Firmware written and verified; rebooting";doc["bytes"]=firmwareUploadBytes;
  String body;serializeJson(doc,body);server.send(200,"application/json; charset=utf-8",body);
  delay(600);ESP.restart();
}

static uint16_t zipRead16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t zipRead32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint32_t zipCrc32Update(uint32_t crc, const uint8_t* data, size_t length) {
  for (size_t i=0;i<length;i++) { crc^=data[i]; for(uint8_t b=0;b<8;b++) crc=(crc>>1)^((crc&1)?0xEDB88320UL:0); } return crc;
}
static void firmwareUploadFail(const String& message) {
  if(firmwareUploadStarted) Update.abort();
  firmwareUploadStarted=false; firmwareUploadOk=false; firmwareUploadError=message;
}
static bool firmwareZipHeaderReady() {
  const uint8_t* h=firmwareZipHeader;
  if(zipRead32(h)!=0x04034b50UL){firmwareUploadFail("ZIP не распознан: отсутствует заголовок архива");return false;}
  const uint16_t flags=zipRead16(h+6),method=zipRead16(h+8),nameLength=zipRead16(h+26),extraLength=zipRead16(h+28);
  if(flags&0x0008){firmwareUploadFail("ZIP с data descriptor не поддерживается. Скачайте ZIP из интерфейса устройства");return false;}
  if(method!=0){firmwareUploadFail("ZIP использует сжатие. Поддерживается ZIP без сжатия из интерфейса устройства или обычный BIN");return false;}
  if(firmwareZipHeaderExpected!=30U+nameLength+extraLength||!nameLength||firmwareZipHeaderBytes<firmwareZipHeaderExpected){firmwareUploadFail("Некорректный заголовок ZIP");return false;}
  String entry;for(uint16_t i=0;i<nameLength;i++)entry+=(char)h[30+i];entry.toLowerCase();
  if(!entry.endsWith(".bin")){firmwareUploadFail("В ZIP не найден BIN-файл прошивки");return false;}
  const uint32_t compressed=zipRead32(h+18),uncompressed=zipRead32(h+22);
  if(!compressed||compressed!=uncompressed){firmwareUploadFail("Некорректный размер BIN внутри ZIP");return false;}
  firmwareZipRemaining=compressed;firmwareZipExpectedCrc=zipRead32(h+14);firmwareZipCrc=0xFFFFFFFFUL;
  if(!Update.begin(uncompressed,U_FLASH)){firmwareUploadFail("Не удалось начать OTA, код "+String(Update.getError()));Update.printError(Serial);return false;}
  firmwareUploadStarted=true;return true;
}
static void firmwareUploadData(const uint8_t* data,size_t length) {
  if(firmwareUploadError.length()||!length)return;
  if(!firmwareUploadIsZip){
    if(!firmwareUploadStarted){if(!Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH)){firmwareUploadFail("Update.begin failed, code "+String(Update.getError()));return;}firmwareUploadStarted=true;}
    size_t written=Update.write(const_cast<uint8_t*>(data),length);if(written!=length){firmwareUploadFail("Ошибка записи BIN, код "+String(Update.getError()));return;}firmwareUploadBytes+=written;return;
  }
  size_t offset=0;
  while(offset<length&&!firmwareUploadError.length()){
    if(!firmwareUploadStarted){
      if(firmwareZipHeaderBytes>=sizeof(firmwareZipHeader)){firmwareUploadFail("Заголовок ZIP слишком большой");break;}
      firmwareZipHeader[firmwareZipHeaderBytes++]=data[offset++];
      if(firmwareZipHeaderBytes==30&&firmwareZipHeaderExpected==30){
        firmwareZipHeaderExpected=30U+zipRead16(firmwareZipHeader+26)+zipRead16(firmwareZipHeader+28);
        if(firmwareZipHeaderExpected>sizeof(firmwareZipHeader)){firmwareUploadFail("Заголовок ZIP слишком большой");break;}
      }
      if(firmwareZipHeaderBytes==firmwareZipHeaderExpected)firmwareZipHeaderReady();
      continue;
    }
    size_t count=min(length-offset,(size_t)firmwareZipRemaining);if(!count)break;
    size_t written=Update.write(const_cast<uint8_t*>(data+offset),count);if(written!=count){firmwareUploadFail("Ошибка записи BIN из ZIP, код "+String(Update.getError()));break;}
    firmwareZipCrc=zipCrc32Update(firmwareZipCrc,data+offset,count);firmwareUploadBytes+=written;firmwareZipRemaining-=(uint32_t)count;offset+=count;
  }
}
void handleFirmwareUploadStream() {
  HTTPUpload& upload=server.upload();
  if(upload.status==UPLOAD_FILE_START){
    firmwareUploadOk=false;firmwareUploadError="";firmwareUploadBytes=0;
    firmwareUploadIsZip=upload.filename.endsWith(".zip")||upload.filename.endsWith(".ZIP");
    firmwareUploadStarted=false;firmwareZipHeaderBytes=0;firmwareZipHeaderExpected=30;firmwareZipRemaining=0;firmwareZipExpectedCrc=0;firmwareZipCrc=0xFFFFFFFFUL;
    Serial.printf("OTA upload start: %s\n",upload.filename.c_str());
    if(!firmwareUploadIsZip&&!(upload.filename.endsWith(".bin")||upload.filename.endsWith(".BIN")))firmwareUploadFail("Выберите файл BIN или ZIP");
  }else if(upload.status==UPLOAD_FILE_WRITE)firmwareUploadData(upload.buf,upload.currentSize);
  else if(upload.status==UPLOAD_FILE_END){
    if(!firmwareUploadError.length()&&firmwareUploadIsZip&&(!firmwareUploadStarted||firmwareZipRemaining))firmwareUploadFail("Не удалось полностью извлечь BIN из ZIP");
    if(!firmwareUploadError.length()&&firmwareUploadIsZip&&((firmwareZipCrc^0xFFFFFFFFUL)!=firmwareZipExpectedCrc))firmwareUploadFail("Контрольная сумма BIN внутри ZIP не совпала");
    if(!firmwareUploadError.length()&&firmwareUploadStarted&&Update.end(true)){firmwareUploadOk=true;firmwareUploadStarted=false;Serial.printf("OTA upload complete: %u bytes\n",(unsigned)firmwareUploadBytes);}
    else {if(!firmwareUploadError.length())firmwareUploadFail("Проверка образа не пройдена, код "+String(Update.getError()));else if(firmwareUploadStarted){Update.abort();firmwareUploadStarted=false;}Update.printError(Serial);}
  }else if(upload.status==UPLOAD_FILE_ABORTED){if(firmwareUploadStarted)Update.abort();firmwareUploadStarted=false;firmwareUploadOk=false;firmwareUploadError="Загрузка отменена пользователем";}
}
void handleSystemReboot() {
  server.send(200, "application/json; charset=utf-8", R"({"ok":true,"message":"ESP32 перезагружается"})");
  delay(250); ESP.restart();
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
  aiMonitor.reset();

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
  aiLoggingEnabled=preferences.getBool("ai_log_on",false);
  aiReferenceEnabled=preferences.getBool("ai_ref_on",false);
  aiReferenceLat=preferences.getDouble("ai_ref_lat",0.0);
  aiReferenceLon=preferences.getDouble("ai_ref_lon",0.0);
  aiReferenceRadiusM=preferences.getFloat("ai_ref_rad",50.0f);
  if (!preferences.isKey("points")) preferences.putString("points", "[]");
  reportIntervalSeconds = preferences.getUShort("report_sec", 300);
  if (reportIntervalSeconds < 10 || reportIntervalSeconds > 1800) reportIntervalSeconds = 300;
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
    sdReady = true;
    loadAiProfileFromSd();
    Serial.println("SD card mounted");
  } else {
    Serial.println("SD card not mounted");
  }

  WiFi.mode(WIFI_AP_STA);
  timezoneOffsetMinutes = preferences.getInt("tz_min", 0);
  applyTimezoneOffset(timezoneOffsetMinutes);
  WiFi.softAP(AP_SSID, AP_PASS);
  String savedSsid = preferences.getString("wifi_ssid", "");
  String savedPass = preferences.getString("wifi_pass", "");
  if (savedSsid.length()) WiFi.begin(savedSsid.c_str(), savedPass.c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/satellites", HTTP_GET, handleSatellites);
  server.on("/network/config", HTTP_POST, handleNetworkConfig);
  server.on("/network/status", HTTP_GET, handleNetworkStatus);
  server.on("/network/timezone", HTTP_POST, handleNetworkTimezone);
  server.on("/wifi/scan", HTTP_POST, handleWifiScanStart);
  server.on("/wifi/scan/status", HTTP_GET, handleWifiScanStatus);
  server.on("/coords", HTTP_GET, handleCoords);
  server.on("/system/reboot", HTTP_POST, handleSystemReboot);
  server.on("/firmware/info", HTTP_GET, handleFirmwareInfo);
  server.on("/firmware/download", HTTP_GET, handleFirmwareDownload);
  server.on("/firmware/update", HTTP_POST, handleFirmwareUploadResult, handleFirmwareUploadStream);
  server.on("/points", HTTP_GET, handleSavedPoints);
  server.on("/points/save", HTTP_POST, handleSavePoint);
  server.on("/points/update", HTTP_POST, handleUpdatePoint);
  server.on("/points/delete", HTTP_POST, handleDeletePoint);
  server.on("/uart/baud", HTTP_POST, handleSetBaud);
  server.on("/ubx/send", HTTP_POST, handleUbxSend);
  server.on("/ubx/status", HTTP_GET, handleUbxStatus);
  server.on("/imu", HTTP_GET, handleImu);
  server.on("/ai/status", HTTP_GET, handleAiStatus);
  server.on("/ai/profile", HTTP_GET, handleAiProfileDownload);
  server.on("/ai/profile", HTTP_POST, handleAiProfileUpload);
  server.on("/ai/reset", HTTP_POST, handleAiReset);
  server.on("/ai/logging", HTTP_POST, handleAiLogging);
  server.on("/ai/reference", HTTP_POST, handleAiReference);
  server.on("/ai/reference/clear", HTTP_POST, handleAiReferenceClear);
  server.on("/ai/situation-logging", HTTP_POST, handleSituationLogging);
  server.on("/ai/report-interval", HTTP_POST, handleReportInterval);
  server.on("/ai/situation-log", HTTP_GET, handleSituationLogDownload);
  server.on("/ai/dataset", HTTP_GET, handleAiDataset);
  server.on("/uart", HTTP_GET, handleUart);
  server.on("/maps", HTTP_GET, handleMaps);
  server.on("/maps/info", HTTP_GET, handleMapInfo);
  server.on("/maps/logs", HTTP_GET, handleMbtilesLogs);
  server.on("/settings/fix-timeout", HTTP_POST, handleSetFixTimeout);
  server.on("/tiles", HTTP_GET, handleTile);
  for (const char* leafletFile : LEAFLET_FILES) server.on(leafletFile, HTTP_GET, handleLib);
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
  const double radiusLat = 0.00018; // about 20 m north/south
  const double radiusLng = 0.000314; // about 20 m east/west at Sarov latitude
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
  double east = 0.000314 * cos(angle) * cos(simulatorLat * pi / 180.0);
  double north = -0.00018 * sin(angle);
  double course = atan2(east, north) * 180.0 / pi;
  if (course < 0) course += 360.0;
  double circumference = 2.0 * pi * 20.0;
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

  if (WiFi.status() == WL_CONNECTED && !ntpConfigured) {
    configTime(0, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");
    ntpConfigured = true;
    Serial.println("NTP synchronization configured (UTC)");
  }
  if (WiFi.status() != WL_CONNECTED && ntpConfigured) ntpConfigured = false;

  if (haveFix && millis() - lastFixMs > (unsigned long)fixTimeoutSeconds * 1000UL) {
    haveFix = false; satellites = 0; hdop = 0.0;
  }
  if (lastGgaMs && millis() - lastGgaMs > 3000 && !nmeaSimulatorEnabled) { satellites = 0; haveFix = false; }
  if (lastGsvMs && millis() - lastGsvMs > 5000) { satellitesVisible = 0; gsvSnrCount = 0; avgGsvSnr = 0.0f; }

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
      if (gps.location.isValid() && gps.location.isUpdated() &&
          satellites > 0 && lastGgaMs && millis() - lastGgaMs <= 3000) {
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

  static unsigned long lastAi = 0;
  if (millis() - lastAi >= 1000) {
    lastAi = millis();
    const float gpsSpeedMps = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
    const float accelMagnitude = imuReady ? sqrtf(ax*ax + ay*ay + az*az) : 0.0f;
    const float gyroMagnitude = imuReady ? sqrtf(gx*gx + gy*gy + gz*gz) : 0.0f;
    const bool positionUpdated = coordinatesAvailable && (millis() - lastCoordinateMs <= 1100);
    const bool trustedForLearning = !aiReferenceEnabled || (haveFix && coordinatesAvailable && aiDistanceMeters(aiReferenceLat,aiReferenceLon,lat,lng) <= aiReferenceRadiusM);
    aiMonitor.setLearningAllowed(trustedForLearning);
    aiMonitor.update(haveFix, (float)hdop, satellites, satellitesVisible,
      gsvSnrCount ? avgGsvSnr : NAN, gpsSpeedMps,
      accelMagnitude, gyroMagnitude, imuReady, lat, lng,
      positionUpdated, millis());
    logAiDatasetRow();
    writeSituationLog();
  }

  static unsigned long lastBle = 0;
  if (bleConnected && millis() - lastBleMessageMs >= (unsigned long)reportIntervalSeconds * 1000UL) {
    lastBleMessageMs = millis();

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
