#pragma once

// Logging shim for the protocol drivers.
//
// The drivers (bdap_driver / vevor_driver) are deliberately pure C++ so the
// host test harness (tests/, plain g++) can build them and feed recorded
// frames. On the ESPHome build the real ESP_LOG* macros are used; on the host
// they compile to no-ops - the tests assert on parsed values, not on logs.

#if __has_include("esphome/core/log.h")
#include "esphome/core/log.h"
#define HEATER_LOGD(...) ESP_LOGD(__VA_ARGS__)
#define HEATER_LOGW(...) ESP_LOGW(__VA_ARGS__)
#define HEATER_LOGI(...) ESP_LOGI(__VA_ARGS__)
#define HEATER_LOGE(...) ESP_LOGE(__VA_ARGS__)
#else
#define HEATER_LOGD(...) ((void) 0)
#define HEATER_LOGW(...) ((void) 0)
#define HEATER_LOGI(...) ((void) 0)
#define HEATER_LOGE(...) ((void) 0)
#endif
