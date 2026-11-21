#pragma once

#include <Arduino.h>

#define MAP_LOG_CAPACITY 200

struct MapLogEntry {
  unsigned long ms;
  char level[8];
  char event[32];
  char details[160];
};

static MapLogEntry mapLogBuffer[MAP_LOG_CAPACITY];
static size_t mapLogHead = 0;
static size_t mapLogCount = 0;

inline void mapLogClear() {
  mapLogHead = 0;
  mapLogCount = 0;
}

inline void mapLog(const char* level, const char* event, const String& details) {
  MapLogEntry& entry = mapLogBuffer[mapLogHead];
  entry.ms = millis();
  strlcpy(entry.level, level, sizeof(entry.level));
  strlcpy(entry.event, event, sizeof(entry.event));
  strlcpy(entry.details, details.c_str(), sizeof(entry.details));
  mapLogHead = (mapLogHead + 1) % MAP_LOG_CAPACITY;
  if (mapLogCount < MAP_LOG_CAPACITY) mapLogCount++;

  Serial.printf("[MAP][%s] %s: %s\n", level, event, details.c_str());
}

inline void mapLogf(const char* level, const char* event, const char* format, ...) {
  char buffer[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  mapLog(level, event, String(buffer));
}

inline String mapLogsText() {
  String out;
  out += "ESP32 GNSS offline map log\n";
  out += "Entries: " + String(mapLogCount) + "\n\n";

  size_t start = (mapLogCount == MAP_LOG_CAPACITY) ? mapLogHead : 0;
  for (size_t i = 0; i < mapLogCount; i++) {
    const MapLogEntry& entry = mapLogBuffer[(start + i) % MAP_LOG_CAPACITY];
    out += "[" + String(entry.ms) + " ms][" + entry.level + "] " + entry.event;
    if (entry.details[0]) out += ": " + String(entry.details);
    out += "\n";
  }
  return out;
}

inline String mapLogsJson() {
  String out = "{";
  out += "\"count\":" + String(mapLogCount) + ",";
  out += "\"lines\":\"";

  size_t start = (mapLogCount == MAP_LOG_CAPACITY) ? mapLogHead : 0;
  for (size_t i = 0; i < mapLogCount; i++) {
    const MapLogEntry& entry = mapLogBuffer[(start + i) % MAP_LOG_CAPACITY];
    if (i) out += "\\n";
    out += "[" + String(entry.ms) + " ms][" + entry.level + "] " + entry.event;
    if (entry.details[0]) out += ": " + String(entry.details);
  }

  out += "\"}";
  return out;
}
