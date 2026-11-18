#pragma once

#include <Arduino.h>
#include <math.h>

// Lightweight on-device online anomaly model for ESP32-S3.
// This is a statistical ML baseline, not a trained neural network.
class GnssAiMonitor {
public:
  void reset() {
    samples_ = 0;
    for (uint8_t i = 0; i < 5; ++i) {
      featureSamples_[i] = 0;
      mean_[i] = 0.0f;
      m2_[i] = 0.0f;
      last_[i] = NAN;
    }
    qualityScore_ = -1;
    maxZ_ = 0.0f;
    anomaly_ = false;
    anomalyCode_ = "warming_up";
    risk_ = "unknown";
    motion_ = "unknown";
    mismatch_ = false;
    mismatchSpeed_ = 0.0f;
    lastPositionValid_ = false;
    lastPositionMs_ = 0;
    lastLat_ = lastLon_ = 0.0;
    lastUpdateMs_ = 0;
    hdopTrend_ = 0.0f;
    cnoTrend_ = 0.0f;
  }

  void update(bool fix, float hdop, int used, float avgCno,
              float gpsSpeedMps, float accelMagnitude, float gyroMagnitude,
              bool imuAvailable, double lat, double lon, bool positionUpdated, uint32_t nowMs) {
    if (nowMs - lastUpdateMs_ < 900) return;
    lastUpdateMs_ = nowMs;

    // Motion class is deliberately conservative: a 6-axis IMU cannot determine
    // absolute heading or distinguish all forms of constant-velocity motion.
    const float dynamicAccel = fabsf(accelMagnitude - 9.80665f);
    if (!imuAvailable) motion_ = "imu_unavailable";
    else if (fix && gpsSpeedMps > 1.0f) motion_ = "moving";
    else if (dynamicAccel > 2.2f) motion_ = "vibration";
    else if (gyroMagnitude > 12.0f) motion_ = "rotation";
    else if (accelMagnitude > 7.5f && accelMagnitude < 12.2f && gyroMagnitude < 4.0f) motion_ = "stationary_or_low_dynamics";
    else motion_ = "uncertain";

    // Detect physically implausible jumps by comparing GNSS displacement speed
    // against the receiver's reported speed. This is a warning, not proof of spoofing.
    if (fix && positionUpdated && isfinite(lat) && isfinite(lon)) {
      if (lastPositionValid_) {
        const float dt = (nowMs - lastPositionMs_) / 1000.0f;
        if (dt >= 0.5f && dt <= 10.0f) {
          const float distance = haversineMeters(lastLat_, lastLon_, lat, lon);
          const float impliedSpeed = distance / dt;
          const float allowedDifference = fmaxf(15.0f, gpsSpeedMps * 1.5f + 8.0f);
          mismatchSpeed_ = fabsf(impliedSpeed - gpsSpeedMps);
          mismatch_ = distance > 30.0f && mismatchSpeed_ > allowedDifference;
        }
      }
      lastLat_ = lat;
      lastLon_ = lon;
      lastPositionMs_ = nowMs;
      lastPositionValid_ = true;
    } else if (!fix) {
      lastPositionValid_ = false;
      mismatch_ = false;
    }

    if (!fix || !isfinite(hdop) || hdop <= 0.0f || used <= 0) {
      risk_ = fix ? "limited_data" : "no_fix";
      qualityScore_ = -1;
      anomaly_ = false;
      anomalyCode_ = "no_valid_fix";
      return;
    }

    // Feature vector: HDOP, satellites used, average GSV SNR/CN0 proxy.
    // NMEA GSV SNR is not identical to calibrated receiver C/N0 telemetry.
    const bool cnoAvailable = isfinite(avgCno) && avgCno > 0.0f;
    float features[5] = { hdop, (float)used, avgCno, dynamicAccel, gyroMagnitude };

    maxZ_ = 0.0f;
    anomalyCode_ = "normal";
    for (uint8_t i = 0; i < 5; ++i) {
      if ((i == 2 && !cnoAvailable) || (i >= 3 && !imuAvailable)) continue;
      if (featureSamples_[i] >= 12) {
        const float variance = (featureSamples_[i] > 1) ? m2_[i] / (float)(featureSamples_[i] - 1) : 0.0f;
        const float sigma = sqrtf(fmaxf(variance, 0.04f));
        const float z = fabsf(features[i] - mean_[i]) / sigma;
        if (z > maxZ_) {
          maxZ_ = z;
          anomalyCode_ = (i == 0) ? "hdop_deviation" :
            (i == 1) ? "satellite_count_deviation" :
            (i == 2) ? "signal_level_deviation" :
            (i == 3) ? "imu_acceleration_deviation" : "imu_gyro_deviation";
        }
      }
    }

    const bool baselineReady = featureSamples_[0] >= 12 && featureSamples_[1] >= 12;
    anomaly_ = baselineReady && maxZ_ >= 3.5f;
    if (!anomaly_) {
      ++samples_;
      for (uint8_t i = 0; i < 5; ++i) {
        if ((i == 2 && !cnoAvailable) || (i >= 3 && !imuAvailable)) continue;
        ++featureSamples_[i];
        const float delta = features[i] - mean_[i];
        mean_[i] += delta / (float)featureSamples_[i];
        const float delta2 = features[i] - mean_[i];
        m2_[i] += delta * delta2;
      }
    }

    if (!baselineReady) {
      qualityScore_ = -1;
      risk_ = "learning";
    } else {
      const float penalty = fminf(100.0f, maxZ_ * 18.0f);
      qualityScore_ = (int)lroundf(fmaxf(0.0f, 100.0f - penalty));
      risk_ = anomaly_ ? "anomaly" : "normal";
    }

    if (isfinite(last_[0])) hdopTrend_ = hdop - last_[0];
    if (cnoAvailable && isfinite(last_[2])) cnoTrend_ = avgCno - last_[2];
    last_[0] = hdop;
    last_[1] = (float)used;
    if (cnoAvailable) last_[2] = avgCno;

    if (hdopTrend_ > 0.35f && (!cnoAvailable || cnoTrend_ < -2.5f)) {
      risk_ = "degrading";
    }
    if (mismatch_) risk_ = "integrity_warning";
  }

  uint32_t samples() const { return samples_; }
  int qualityScore() const { return qualityScore_; }
  float maxZ() const { return maxZ_; }
  bool anomaly() const { return anomaly_; }
  bool mismatch() const { return mismatch_; }
  float mismatchSpeed() const { return mismatchSpeed_; }
  const char* anomalyCode() const { return anomalyCode_; }
  const char* risk() const { return risk_; }
  const char* motion() const { return motion_; }
  float hdopTrend() const { return hdopTrend_; }
  float cnoTrend() const { return cnoTrend_; }
  bool trained() const { return featureSamples_[0] >= 12 && featureSamples_[1] >= 12; }

private:
  uint32_t samples_ = 0;
  uint32_t featureSamples_[5] = {0, 0, 0, 0, 0};
  float mean_[5] = {0, 0, 0, 0, 0};
  float m2_[5] = {0, 0, 0, 0, 0};
  float last_[3] = {NAN, NAN, NAN};
  int qualityScore_ = -1;
  float maxZ_ = 0.0f;
  bool anomaly_ = false;
  bool mismatch_ = false;
  float mismatchSpeed_ = 0.0f;
  bool lastPositionValid_ = false;
  uint32_t lastPositionMs_ = 0;
  uint32_t lastUpdateMs_ = 0;
  double lastLat_ = 0.0, lastLon_ = 0.0;
  float hdopTrend_ = 0.0f, cnoTrend_ = 0.0f;
  const char* anomalyCode_ = "warming_up";
  const char* risk_ = "unknown";
  const char* motion_ = "unknown";

  static float haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    const double rad = 0.017453292519943295;
    const double dLat = (lat2 - lat1) * rad;
    const double dLon = (lon2 - lon1) * rad;
    const double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
      cos(lat1 * rad) * cos(lat2 * rad) * sin(dLon / 2.0) * sin(dLon / 2.0);
    return (float)(6371000.0 * 2.0 * atan2(sqrt(a), sqrt(fmax(0.0, 1.0 - a))));
  }
};
