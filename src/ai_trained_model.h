#pragma once
#include <stdint.h>

// Generated from ai_dataset (2).csv: stable FIX + non-anomalous rows only.
// GNSS-only baseline: IMU features excluded because the recording spans
// separate boot sessions with different IMU magnitude baselines.
namespace GnssTrainedModel {
static constexpr bool enabled = true;
static constexpr uint8_t featureMask = 39;
static constexpr uint16_t centerQ[6] = {77, 1200, 3917, 1, 1, 2200};
static constexpr uint16_t scaleQ[6] = {25, 100, 49, 100, 100, 100};
static constexpr uint16_t thresholdQ = 400;
static constexpr uint32_t trainingSamples = 335;
}
