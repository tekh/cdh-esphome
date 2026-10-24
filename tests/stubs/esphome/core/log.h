#pragma once
// Host syntax-check stub for esphome/core/log.h
//
// IMPORTANT: unlike the real ESP_LOG*, these MUST NOT compile to no-ops.
// A `((void) 0)` macro never type-checks its arguments, so a typo inside a
// log call (e.g. a trailing-underscore member that only exists on the class
// not on Params) compiles clean on the host and explodes on the device.
// This sink is not called at runtime (unless HEATER_LOG_STUB is set) but the
// CALL is fully evaluated and type-checked, including printf-format args via
// -Wformat on the fprintf.
#include <cstdio>
#include <cstdlib>

namespace esphome_stub_log {
template <typename... Args>
inline void sink(const char *tag, const char *fmt, Args... args) {
  (void) tag;  // sink is silent by default; args/fmt still type-checked
  if (std::getenv("HEATER_LOG_STUB") == nullptr) return;
  std::fprintf(stderr, fmt, args...);
}
}  // namespace esphome_stub_log

#define ESP_LOGI(tag, ...) ::esphome_stub_log::sink(tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) ::esphome_stub_log::sink(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) ::esphome_stub_log::sink(tag, __VA_ARGS__)
#define ESP_LOGE(tag, ...) ::esphome_stub_log::sink(tag, __VA_ARGS__)
#define ESP_LOGCONFIG(tag, ...) ::esphome_stub_log::sink(tag, __VA_ARGS__)
