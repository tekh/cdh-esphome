#include "heater_policies.h"

#include <algorithm>
#include <cmath>

#include "heater_log.h"
#include "heater_output.h"

namespace esphome {
namespace heater_uart {

[[maybe_unused]] static const char *TAG = "heater_uart";

// ==================== Thermostat ====================

Thermostat::Result Thermostat::enter_mode(HeaterMode mode, float room_temp, float target_temp,
                                          bool running_intent, const Params &params,
                                          uint32_t now) {
    Result out;
    const HeaterProfile &profile = *profile_;

    if (mode == HeaterMode::OFF) {
        // Force off - stop heater if running
        if (running_intent) {
            HEATER_LOGI(TAG, "Mode Off: stopping heater");
            out.request_stop = true;
            out.enter_cooldown = true;
        }
        in_standby_ = false;
        in_auto_shutdown_ = false;

    } else if (mode == HeaterMode::AUTO) {
        const float threshold = standby_threshold(target_temp, params);

        if (params.external_sensor_present && params.sensor_stale) {
            // No blind starts: the room temperature is stale (sensor dead or
            // never read). Park in standby; the thermostat's standby check
            // starts the heater once fresh readings arrive. If the heater is
            // running, stop it (burning on blind data).
            HEATER_LOGW(TAG, "Mode Auto: room sensor stale - no blind start, parking in standby");
            in_standby_ = true;
            if (running_intent) {
                out.request_stop = true;
                out.enter_cooldown = true;
            }
        } else if (room_temp < threshold) {
            // Cold enough - start heater
            HEATER_LOGI(TAG, "Mode Auto: room %.1f°C < %.1f°C threshold, starting heater",
                        room_temp, threshold);
            out.request_start = true;
            pump_freq_setting_ = profile.pump_freq_initial;
            last_pump_adjust_time_ = now;
            in_standby_ = false;
        } else {
            // Too warm - enter standby, wait for temp drop
            HEATER_LOGI(TAG, "Mode Auto: room %.1f°C >= %.1f°C threshold, entering standby",
                        room_temp, threshold);
            in_standby_ = true;
            // If heater is running, stop it
            if (running_intent) {
                out.request_stop = true;
                out.enter_cooldown = true;
            }
        }
        in_auto_shutdown_ = false;

    } else if (mode == HeaterMode::ON) {
        // Force on - start heater regardless of temperature
        HEATER_LOGI(TAG, "Mode On: forcing heater start");
        out.request_start = true;
        // Start level on a cold start. Manual-HEAT profiles start at FULL POWER and
        // ramp down as the room approaches the requested temperature
        // (control()); everything else keeps the ignition-rate default.
        pump_freq_setting_ = params.heat_mode_manual ? profile.pump_freq_max
                                                     : profile.pump_freq_initial;
        HEATER_LOGI(TAG, "Mode On: start level %.1f Hz (%.1f = full power)", pump_freq_setting_,
                    profile.pump_freq_max);
        last_pump_adjust_time_ = now;
        in_standby_ = false;
        in_auto_shutdown_ = false;
    }

    out.pump_request_hz = pump_freq_setting_;
    return out;
}

Thermostat::Result Thermostat::control(const Telemetry &tlm, float room_temp, float target_temp,
                                       HeaterMode mode, const Params &params, uint32_t now) {
    Result out;
    const HeaterProfile &profile = *profile_;

    // === ROOM-SENSOR HEALTH GATE ===
    // While the external room sensor keeps producing ACCEPTED readings, the
    // staleness timer is refreshed by the guard (loop path, 1 Hz). NaN and
    // physically implausible samples do NOT refresh it, so a DHT22 that is
    // dead or stuck emitting garbage is blind after 10 min. Then the last
    // known value could hide an overheat (the ambient limit reads the same
    // stale value) and a stale cold value could trigger a blind start. Stop
    // a running heater and block all auto-start/stop logic until fresh data
    // arrives. NOTE: this gate is NOT gated by oversight_enabled - burning on
    // blind data is a host-level fault the ECM cannot see (user requirement,
    // 2026-10-20: DHT22 in warning state > 10 min -> heater off; 2026-10-21:
    // also don't act on glitch samples.)
    if (params.external_sensor_present && params.sensor_stale) {
        if (tlm.on) {
            HEATER_LOGW(TAG, "Room sensor stale > %.0f min (last valid %.1fC): stopping heater",
                        10.0f, room_temp);
            out.request_stop = true;
            out.enter_cooldown = true;
            in_auto_shutdown_ = false;
            if (mode == HeaterMode::AUTO) {
                in_standby_ = true;  // re-arm auto-restart for when the sensor recovers
            }
        }
        out.pump_request_hz = pump_freq_setting_;
        return out;  // block standby/auto-restart/auto-shutdown until fresh readings
    }

    // === STANDBY CHECK (AUTO mode) ===
    // If in standby, check if temperature has dropped enough to start
    if (in_standby_ && mode == HeaterMode::AUTO) {
        float threshold = standby_threshold(target_temp, params);
        if (room_temp < threshold) {
            HEATER_LOGI(TAG, "Standby->Start: room %.1f°C < %.1f°C threshold", room_temp, threshold);
            out.request_start = true;
            pump_freq_setting_ = profile.pump_freq_initial;
            last_pump_adjust_time_ = now;
            in_standby_ = false;
        }
    }

    // Thermostat and safety logic
    // During early ignition (states 1-3) and shutdown (states 6-8), the heater's internal
    // controller manages pump/fan - we should NOT interfere with those sequences.
    // State 4 (Ignited): Ramp pump to max to reach operating temperature quickly.
    // State 5 (Running): Normal thermostat control.
    // Note: In ON mode, we skip thermostat control but still respect safety limits.
    if (tlm.on) {
        if (tlm.run_state == RUN_STATE_IGNITED) {
            // Heater has ignited - ramp pump to max to reach operating temp quickly
            // But respect the adjustment interval - thermal response takes time
            if (pump_freq_setting_ < profile.pump_freq_max) {
                if (now - last_pump_adjust_time_ >= params.pump_adjust_interval_ms) {
                    // Scale step based on how cold the room is
                    float temp_gap = target_temp - room_temp;
                    float multiplier = (temp_gap >= params.gap_threshold_far)   ? params.step_mult_far
                                       : (temp_gap >= params.gap_threshold_mid) ? params.step_mult_mid
                                                                          : 1.0f;
                    float step = profile.pump_freq_step * multiplier;

                    float new_freq = std::min(pump_freq_setting_ + step, profile.pump_freq_max);
                    HEATER_LOGI(TAG, "Ignited (state 4): gap %.1f°C, ramping pump %.1f -> %.1f Hz (x%.0f)",
                                temp_gap, pump_freq_setting_, new_freq, multiplier);
                    pump_freq_setting_ = new_freq;
                    last_pump_adjust_time_ = now;
                }
            }
            out.pump_request_hz = pump_freq_setting_;
            return out;
        }

        if (tlm.run_state != RUN_STATE_RUNNING) {
            // Heater is in early startup (1-3) or shutdown (6-8) - let it handle pump/fan
            HEATER_LOGD(TAG, "Ignition/shutdown in progress (state %d) - thermostat paused", tlm.run_state);
            out.pump_request_hz = pump_freq_setting_;
            return out;
        }

        // === SAFETY: Emergency shutdown if heat exchanger is critically hot ===
        if (params.oversight_enabled && tlm.hx_temp > profile.hx_temp_critical) {
            HEATER_LOGE(TAG, "CRITICAL: Heat exchanger %.0f°C > %.0f°C limit! Emergency shutdown!",
                        tlm.hx_temp, profile.hx_temp_critical);
            out.request_stop = true;
            out.enter_cooldown = true;
            in_auto_shutdown_ = false;
            out.pump_request_hz = pump_freq_setting_;
            return out;
        }

        // === ROOM TEMPERATURE THERMOSTAT (AUTO mode only) ===
        // In ON mode, skip thermostat control - user wants heat regardless of temperature
        // Only adjust if enough time has passed - thermal mass means changes take time
        if (now - last_pump_adjust_time_ < params.pump_adjust_interval_ms) {
            out.pump_request_hz = pump_freq_setting_;
            return out;  // Wait for thermal response before next adjustment
        }

        float new_freq = pump_freq_setting_;

        // Room temperature control - different behavior for AUTO vs HEAT mode
        if (mode == HeaterMode::AUTO) {
            // AUTO mode: Three-phase thermostat control with auto-shutdown
            // 1. Heating phase: room < (target - approach_threshold) → increase pump
            // 2. Approach phase: within approach_threshold of target → decrease pump slowly
            // 3. At/above target: decrease pump rapidly to prevent overshoot
            // 4. Auto-shutdown when at minimum pump and temp exceeded

            float approach_temp = target_temp - params.approach_threshold;
            float temp_gap = std::abs(room_temp - target_temp);

            if (room_temp < approach_temp) {
                // HEATING PHASE: Room is cold, increase pump
                // Larger gaps = more aggressive increase
                float multiplier = (temp_gap >= params.gap_threshold_far)   ? params.step_mult_far
                                   : (temp_gap >= params.gap_threshold_mid) ? params.step_mult_mid
                                                                      : 1.0f;
                float step = profile.pump_freq_step * multiplier;
                new_freq = pump_freq_setting_ + step;
                if (new_freq <= profile.pump_freq_max && new_freq != pump_freq_setting_) {
                    HEATER_LOGI(TAG, "Heating: room %.1f°C < %.1f°C, pump %.1f -> %.1f Hz (x%.0f)",
                                room_temp, approach_temp, pump_freq_setting_, new_freq, multiplier);
                }
            } else if (room_temp < target_temp) {
                // APPROACH PHASE: Within approach threshold, start reducing pump slowly
                // This prevents overshoot by slowing down before reaching target
                float step = profile.pump_freq_step;  // Always 1x step for gentle approach
                new_freq = pump_freq_setting_ - step;
                if (new_freq >= profile.pump_freq_min && new_freq != pump_freq_setting_) {
                    HEATER_LOGI(TAG, "Approach: room %.1f°C approaching %.1f°C, slowing pump %.1f -> %.1f Hz",
                                room_temp, target_temp, pump_freq_setting_, new_freq);
                }
            } else {
                // AT/ABOVE TARGET: Reduce pump rapidly to prevent overshoot
                // Larger overshoot = more aggressive reduction
                float overshoot = room_temp - target_temp;
                float multiplier = (overshoot >= params.os_threshold_big)   ? params.os_mult_big
                                   : (overshoot >= params.os_threshold_mid) ? params.os_mult_mid
                                                                      : 1.0f;
                float step = profile.pump_freq_step * multiplier;
                new_freq = pump_freq_setting_ - step;
                if (new_freq >= profile.pump_freq_min && new_freq != pump_freq_setting_) {
                    HEATER_LOGI(TAG, "At target: room %.1f°C >= %.1f°C, reducing pump %.1f -> %.1f Hz (x%.0f)",
                                room_temp, target_temp, pump_freq_setting_, new_freq, multiplier);
                }
            }
        } else if (mode == HeaterMode::ON) {
            if (params.heat_mode_manual) {
                // HEAT (v2): cold start at full power, then ramp DOWN as the room
                // approaches the requested temperature. Above the request the ramp
                // keeps descending to the profile floor (anti-overshoot); in the
                // approach band it descends toward the user's dial value, which is
                // the level the burn settles at. The HX-soot increase stays disabled
                // here (the user owns the fine tuning); the HX-too-hot decrease and
                // the HX-critical stop still apply below.
                float approach_temp = target_temp - params.approach_threshold;
                if (room_temp < approach_temp) {
                    // still well below the request: hold full power
                    new_freq = profile.pump_freq_max;
                } else if (room_temp < target_temp) {
                    // approaching the request: step down toward the user's level
                    float step = profile.pump_freq_step;
                    new_freq = std::max(pump_freq_user_request_, pump_freq_setting_ - step);
                } else {
                    // at/above the request: keep stepping down, all the way to the floor
                    float overshoot = room_temp - target_temp;
                    float multiplier = (overshoot >= params.os_threshold_big)   ? params.os_mult_big
                                       : (overshoot >= params.os_threshold_mid) ? params.os_mult_mid
                                                                          : 1.0f;
                    float step = profile.pump_freq_step * multiplier;
                    new_freq = std::max(profile.pump_freq_min, pump_freq_setting_ - step);
                }
            } else {
            // HEAT mode: Continuous heat with approach control, no auto-shutdown
            // Ramp pump to minimum when approaching target, but keep heater running

            float approach_temp = target_temp - params.approach_threshold;

            if (room_temp < approach_temp) {
                // HEATING PHASE: Room is cold, increase pump as needed
                float temp_gap = target_temp - room_temp;
                float multiplier = (temp_gap >= params.gap_threshold_far)   ? params.step_mult_far
                                   : (temp_gap >= params.gap_threshold_mid) ? params.step_mult_mid
                                                                      : 1.0f;
                float step = profile.pump_freq_step * multiplier;
                new_freq = pump_freq_setting_ + step;
                if (new_freq <= profile.pump_freq_max && new_freq != pump_freq_setting_) {
                    HEATER_LOGI(TAG, "HEAT mode: room %.1f°C < %.1f°C, pump %.1f -> %.1f Hz (x%.0f)",
                                room_temp, approach_temp, pump_freq_setting_, new_freq, multiplier);
                }
            } else {
                // APPROACH/AT TARGET: Ramp down to minimum pump, keep running
                // This maintains heat without overshooting too much
                if (pump_freq_setting_ > profile.pump_freq_min) {
                    float overshoot = room_temp - target_temp;
                    float multiplier = (overshoot >= params.os_threshold_big)   ? params.os_mult_big
                                       : (overshoot >= params.os_threshold_mid) ? params.os_mult_mid
                                                                          : 1.0f;
                    float step = profile.pump_freq_step * multiplier;
                    new_freq = pump_freq_setting_ - step;
                    if (new_freq < profile.pump_freq_min) {
                        new_freq = profile.pump_freq_min;
                    }
                    if (new_freq != pump_freq_setting_) {
                        HEATER_LOGI(TAG, "HEAT mode approach: room %.1f°C >= %.1f°C, pump %.1f -> %.1f Hz (min %.1f)",
                                    room_temp, approach_temp, pump_freq_setting_, new_freq,
                                    profile.pump_freq_min);
                    }
                }
            }
            }
        }

        // === SAFETY: Heat exchanger limits override thermostat ===
        // Combustion safety takes priority over comfort
        // In manual-HEAT the soot-increase is skipped: the user owns the level,
        // and this override is exactly what used to fight a low-power setpoint.
        // The HX-too-hot decrease below stays active - that is protection.
        bool manual_heat = params.heat_mode_manual && mode == HeaterMode::ON;
        if (params.oversight_enabled && !manual_heat && tlm.hx_temp < profile.hx_temp_low &&
            new_freq < pump_freq_setting_) {
            // HX too cold but thermostat wants to reduce - override, force increase
            new_freq = pump_freq_setting_ + profile.pump_freq_step;
            HEATER_LOGW(TAG, "HX soot safety override: HX %.0f°C < %.0f°C, forcing pump increase to %.1f Hz",
                        tlm.hx_temp, profile.hx_temp_low, new_freq);
        } else if (params.oversight_enabled && tlm.hx_temp >= profile.hx_temp_high &&
                   new_freq > pump_freq_setting_) {
            // HX too hot but thermostat wants to increase - override, force decrease
            new_freq = pump_freq_setting_ - profile.pump_freq_step;
            HEATER_LOGW(TAG, "HX safety override: HX %.0f°C >= %.0f°C, forcing pump decrease to %.1f Hz",
                        tlm.hx_temp, profile.hx_temp_high, new_freq);
        }

        // Apply new pump frequency within limits
        new_freq = std::max(profile.pump_freq_min, std::min(profile.pump_freq_max, new_freq));
        if (new_freq != pump_freq_setting_) {
            pump_freq_setting_ = new_freq;
            last_pump_adjust_time_ = now;
        }

        // === AUTO-SHUTDOWN LOGIC (AUTO mode only) ===
        // Check if conditions are met for auto-shutdown:
        // - Mode is AUTO (not ON - ON mode bypasses thermostat)
        // - Pump at minimum (can't reduce further)
        // - Room temperature exceeds target + overshoot threshold
        if (mode == HeaterMode::AUTO) {
            bool pump_at_min = (pump_freq_setting_ <= profile.pump_freq_min + params.pump_epsilon);
            bool temp_overshoot = (room_temp > target_temp + params.auto_shutdown_overshoot);

            if (pump_at_min && temp_overshoot && !in_auto_shutdown_) {
                HEATER_LOGI(TAG, "Auto-shutdown: pump at min (%.1f Hz), room %.1f°C > target %.1f°C + %.1f°C overshoot",
                            pump_freq_setting_, room_temp, target_temp, params.auto_shutdown_overshoot);
                out.request_stop = true;
                out.enter_cooldown = true;
                in_auto_shutdown_ = true;
            }
        }
    }

    // === AUTO-RESTART LOGIC (AUTO mode only) ===
    // Check if heater should auto-restart from auto-shutdown state
    // This runs when heater is off and was auto-shutdown (not manual off)
    if (!tlm.on && in_auto_shutdown_ && mode == HeaterMode::AUTO) {
        float restart_threshold = standby_threshold(target_temp, params);

        if (room_temp < restart_threshold) {
            HEATER_LOGI(TAG, "Auto-restart: room %.1f°C < target %.1f°C - %.1f°C hysteresis",
                        room_temp, target_temp, params.auto_shutdown_hysteresis);
            out.request_start = true;
            pump_freq_setting_ = profile.pump_freq_initial;
            last_pump_adjust_time_ = now;
            in_auto_shutdown_ = false;
        }
    }

    out.pump_request_hz = pump_freq_setting_;
    return out;
}

// ==================== CombustionSafety ====================

uint16_t CombustionSafety::fan_rpm(float output_fraction, const Telemetry &tlm) const {
    const HeaterProfile &profile = *profile_;

    if (in_cooldown_ || tlm.run_state == RUN_STATE_COOLDOWN) {
        // Cooldown: run fan at high speed to cool the heat exchanger
        return profile.cooldown_fan_rpm;
    }
    if (tlm.run_state == RUN_STATE_IGNITED && tlm.hx_temp < profile.ignition_hx_threshold) {
        // State 4 (Ignited) with HX below threshold: hold moderate fan speed
        // for stable combustion while the pump ramps
        return profile.ignition_fan_rpm;
    }
    // Normal operation: scale fan proportionally with fuel rate to keep
    // combustion air matched to fuel. (The fuel fraction IS the old
    // pump_ratio: fraction = (hz - min) / span.)
    const float pump_ratio = output_domain::clamp01(output_fraction);
    return static_cast<uint16_t>(profile.fan_rpm_min +
                                 pump_ratio * (profile.fan_rpm_max - profile.fan_rpm_min));
}

// ==================== RoomSensorGuard ====================

// A DHT22 right after a failed read can emit a sample that is valid but
// garbage (2026-10-21: 18.4C against a 19.2C trend, which tripped a false
// standby->start in AUTO mode). Such samples must not be acted upon:
//   - NaN / warning readings are rejected outright.
//   - A non-NaN reading more than MAX_STEP away from the last accepted value
//     is a glitch sample: reject it, hold the last trusted value, and do NOT
//     refresh the staleness timer.
//   - At boot, no single sample may become the baseline: the first
//     BOOT_SAMPLES non-NaN samples, at least BOOT_MIN_SPACING_MS apart so
//     each is a distinct DHT22 conversion (~30 s total), must agree within
//     BOOT_MAX_SPREAD; the baseline is their median. A failed window is
//     discarded and retried with the next three samples. Until the baseline
//     is established stale() is true, so AUTO keeps the heater off - the
//     safe state right after a reboot.
bool RoomSensorGuard::refresh(float sample, uint32_t now) {
    if (std::isnan(sample)) return false;

    if (!accepted_ever_) {
        // Boot consensus window: require 3 distinct readings to agree before
        // establishing the baseline.
        if (boot_sample_count_ > 0 && now - boot_last_sample_time_ < BOOT_MIN_SPACING_MS) {
            return false;  // same DHT22 reading as the one already counted; wait for the next conversion
        }
        boot_sample_[boot_sample_count_++] = sample;
        boot_last_sample_time_ = now;
        if (boot_sample_count_ < BOOT_SAMPLES) return false;
        boot_sample_count_ = 0;

        const float a = boot_sample_[0];
        const float b = boot_sample_[1];
        const float c = boot_sample_[2];
        const float mn = a < b ? (a < c ? a : c) : (b < c ? b : c);
        const float mx = a > b ? (a > c ? a : c) : (b > c ? b : c);
        const float spread = mx - mn;
        if (spread > BOOT_MAX_SPREAD) {
            HEATER_LOGW(TAG, "Boot room-temp samples disagree (spread %.1fC > %.1fC): retrying 3-sample consensus",
                        spread, BOOT_MAX_SPREAD);
            return false;  // discard window; next three samples start over
        }
        const float median = a + b + c - mn - mx;  // median of three
        accepted_ever_ = true;
        value_ = median;
        last_accept_time_ = now;
        HEATER_LOGI(TAG, "Room temp baseline %.1fC established (3-sample consensus, spread %.1fC)",
                    median, spread);
        return true;
    }

    if (std::abs(sample - value_) > MAX_STEP) {
        // Implausible jump: DHT22 glitch sample. Hold the last trusted value
        // and deliberately do NOT refresh last_accept_time_.
        if (now - last_reject_log_ >= 60000) {
            last_reject_log_ = now;
            HEATER_LOGW(TAG, "Room sensor glitch sample %.1fC rejected (last accepted %.1fC, max step %.1fC/interval): holding last trusted value",
                        sample, value_, MAX_STEP);
        }
        return false;
    }

    value_ = sample;
    last_accept_time_ = now;
    return true;
}

bool RoomSensorGuard::stale(uint32_t now) const {
    // Staleness is defined on ACCEPTED readings (refresh): NaN and implausible
    // samples do not refresh the timer, so a sensor that is dead or stuck
    // emitting garbage is blind after the timeout.
    if (!accepted_ever_) return true;  // no accepted reading since boot
    return (now - last_accept_time_) > TIMEOUT_MS;
}

}  // namespace heater_uart
}  // namespace esphome
