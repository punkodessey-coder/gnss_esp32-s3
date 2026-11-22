#pragma once
#include <stdint.h>

// Historical signature from a recording made during GNSS spoofing.
// NOT a normal-navigation baseline; it describes the observed spoofing episode.
namespace GnssTrainedModel {
static constexpr const char* role = "spoofing_signature";
static constexpr bool enabled = true;
static constexpr uint8_t featureMask = 39;
static constexpr uint16_t centerQ[6] = {77, 1200, 3917, 1, 1, 2200};
static constexpr uint16_t scaleQ[6] = {25, 100, 49, 100, 100, 100};
static constexpr uint16_t thresholdQ = 400;
static constexpr uint32_t trainingSamples = 335;
}
