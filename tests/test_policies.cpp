#include "tiny_test.h"

// Policy host tests (ROADMAP Phase 3): the thermostat state machine, the
// combustion-protection overrides, and the room-sensor physics guard - the
// logic that previously only ever met a bench flash.

#include "heater_policies.h"

using esphome::heater_uart::CombustionSafety;
using esphome::heater_uart::HeaterMode;
using esphome::heater_uart::HeaterProfile;
using esphome::heater_uart::RoomSensorGuard;
using esphome::heater_uart::RUN_STATE_IGNITED;
using esphome::heater_uart::RUN_STATE_RUNNING;
using esphome::heater_uart::RUN_STATE_START_ACK;
using esphome::heater_uart::Telemetry;
using esphome::heater_uart::Thermostat;

static HeaterProfile jeabong_profile() {
  HeaterProfile p{};
  p.name = "test-8kw";
  p.heat_mode_manual = false;
  p.ecm_owned_safety = false;
  p.pump_freq_min = 1.3f;
  p.pump_freq_max = 5.5f;
  p.pump_freq_step = 0.1f;
  p.pump_freq_initial = 1.8f;
  p.pump_prime_freq = 5.0f;
  p.fan_rpm_min = 1450;
  p.fan_rpm_max = 4500;
  p.ignition_fan_rpm = 2000;
  p.ignition_hx_threshold = 100.0f;
  p.cooldown_fan_rpm = 4000;
  p.hx_temp_low = 190.0f;
  p.hx_temp_target = 250.0f;
  p.hx_temp_high = 255.0f;
  p.hx_temp_critical = 265.0f;
  p.cooldown_target_temp = 60.0f;
  return p;
}

static HeaterProfile vevor_v2_profile() {
  HeaterProfile p = jeabong_profile();
  p.name = "test-vevor-v2";
  p.heat_mode_manual = true;
  p.ecm_owned_safety = true;
  p.pump_freq_min = 0.5f;
  p.pump_freq_max = 2.5f;
  p.pump_freq_initial = 1.0f;
  p.pump_prime_freq = 2.5f;
  p.fan_rpm_min = 900;
  p.fan_rpm_max = 3000;
  p.ignition_fan_rpm = 1400;
  p.ignition_hx_threshold = 85.0f;
  p.cooldown_fan_rpm = 2500;
  p.hx_temp_low = 145.0f;
  p.hx_temp_target = 180.0f;
  p.hx_temp_high = 190.0f;
  p.hx_temp_critical = 205.0f;
  p.cooldown_target_temp = 55.0f;
  return p;
}

static Telemetry running_tlm(float hx) {
  Telemetry t;
  t.run_state = RUN_STATE_RUNNING;
  t.on = true;
  t.hx_temp = hx;
  return t;
}

static Thermostat::Params params_for(const HeaterProfile &profile) {
  Thermostat::Params params;
  params.approach_threshold = 1.0f;
  params.auto_shutdown_overshoot = 0.0f;
  params.auto_shutdown_hysteresis = 1.0f;
  params.heat_mode_manual = profile.heat_mode_manual;
  return params;
}

// --- Thermostat: AUTO three-phase ---

TEST(auto_heating_phase_ramps_up) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  // room 15C vs target 19C: gap 4C -> mid multiplier (2x), approach far
  const auto r = t.control(running_tlm(200), 15.0f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 1.8f, 1e-4);  // 1.6 + (gap 4 -> mid 2x) 0.2 = 1.8
}

TEST(auto_approach_phase_slows_down) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.5f);
  // room 18.5C (within 1C approach band below target 19C): slow down 1x
  const auto r = t.control(running_tlm(200), 18.5f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.4f, 1e-4);
}

TEST(auto_at_target_reduces_fast) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.5f);
  // room 19.7C: overshoot 0.7 >= 0.5 -> 2x reduction
  const auto r = t.control(running_tlm(200), 19.7f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.3f, 1e-4);
}

TEST(auto_shutdown_needs_pump_min_and_overshoot) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(1.3f);  // at minimum
  // room 19.4 > target 19 + overshoot 0.0, pump at min -> shutdown
  const auto r = t.control(running_tlm(200), 19.4f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK(r.request_stop == true);
  CHECK(r.enter_cooldown == true);
  CHECK(t.in_auto_shutdown() == true);
}

TEST(auto_no_shutdown_when_pump_not_at_min) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.0f);  // not at minimum
  const auto r = t.control(running_tlm(200), 19.4f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK(r.request_stop == false);
  CHECK(t.in_auto_shutdown() == false);
}

TEST(auto_restart_on_hysteresis) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.set_auto_shutdown(true);  // previously auto-shutdown
  // heater off now; room dropped to 17.5 < 19 - 1.0 -> restart (strictly less)
  Telemetry off;
  off.run_state = 0;
  off.on = false;
  off.hx_temp = 80.0f;
  const auto r = t.control(off, 17.5f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK(r.request_start == true);
  CHECK_NEAR(r.pump_request_hz, 1.8f, 1e-4);  // ignition rate
  CHECK(t.in_auto_shutdown() == false);
}

TEST(auto_no_restart_manual_off) {
  // manual off clears the auto-shutdown flag -> no restart on temp drop
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.set_auto_shutdown(false);  // e.g. turn_off() cleared it
  Telemetry off;
  off.run_state = 0;
  off.on = false;
  off.hx_temp = 80.0f;
  const auto r = t.control(off, 18.0f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK(r.request_start == false);
}

// --- Thermostat: room-sensor health gate ---

TEST(stale_sensor_stops_running_heater_and_blocks_starts) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(3.0f);
  auto params = params_for(p);
  params.external_sensor_present = true;
  params.sensor_stale = true;
  const auto r = t.control(running_tlm(200), 20.0f, 19.0f, HeaterMode::AUTO, params, 6000);
  CHECK(r.request_stop == true);
  CHECK(r.enter_cooldown == true);
  CHECK(t.in_auto_shutdown() == false);
  CHECK(t.in_standby() == true);  // AUTO re-arm for sensor recovery

  // once stopped, still stale: no blind start, no standby->start
  Telemetry off;
  off.run_state = 0;
  off.on = false;
  off.hx_temp = 60.0f;
  const auto r2 = t.control(off, 20.0f, 19.0f, HeaterMode::AUTO, params, 6100);
  CHECK(r2.request_start == false);
  CHECK_NEAR(r2.pump_request_hz, 3.0f, 1e-4);  // request unchanged while blocked
}

TEST(stale_gate_independent_of_oversight_optout) {
  // burning on blind data is a host-level fault - honoured even when the
  // combustion oversight is opted out (owner requirement, 2026-10-20)
  const HeaterProfile p = vevor_v2_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.0f);
  auto params = params_for(p);
  params.external_sensor_present = true;
  params.sensor_stale = true;
  params.oversight_enabled = false;
  const auto r = t.control(running_tlm(150), 21.0f, 19.0f, HeaterMode::AUTO, params, 6000);
  CHECK(r.request_stop == true);
}

// --- Thermostat: ignition / early-state behaviour ---

TEST(ignited_state_ramps_toward_max) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(1.8f);
  t.reset_adjust_timer(0);
  Telemetry tlm;
  tlm.run_state = RUN_STATE_IGNITED;
  tlm.on = true;
  tlm.hx_temp = 150.0f;
  const auto r = t.control(tlm, 15.0f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.0f, 1e-4);  // one tick (gap 4 -> mid 2x): 1.8 + 0.2
}

TEST(early_startup_pause_leaves_thermostat) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.2f);
  Telemetry tlm;
  tlm.run_state = RUN_STATE_START_ACK;  // state 1: heater-internal sequence
  tlm.on = true;
  tlm.hx_temp = 25.0f;
  const auto r = t.control(tlm, 15.0f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.2f, 1e-4);  // untouched
}

// --- Thermostat: HEAT mode (modulating v1 profile) ---

TEST(heat_modulating_cold_room_ramps_up) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.0f);
  // room 15C, target 19C, gap 4C -> 2x increase
  const auto r = t.control(running_tlm(200), 15.0f, 19.0f, HeaterMode::ON, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.2f, 1e-4);
}

TEST(heat_modulating_at_target_ramps_to_min_not_stop) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.5f);
  // room 19.5 > target: reduce (2x overshoot 0.5) but NEVER stop in HEAT
  const auto r = t.control(running_tlm(200), 19.5f, 19.0f, HeaterMode::ON, params_for(p), 6000);
  CHECK(r.request_stop == false);
  CHECK_NEAR(r.pump_request_hz, 2.3f, 1e-4);
}

// --- Thermostat: HEAT mode (user-owned v2 profile) ---

TEST(heat_manual_holds_full_power_below_approach) {
  const HeaterProfile p = vevor_v2_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(1.5f);  // user setpoint 1.5 Hz... but v2 starts at FULL power
  // room 14C vs target 19C: below approach band -> hold max
  const auto r = t.control(running_tlm(150), 14.0f, 19.0f, HeaterMode::ON, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.5f, 1e-4);  // pump_freq_max
}

TEST(heat_manual_descends_to_user_level_in_band) {
  const HeaterProfile p = vevor_v2_profile();
  Thermostat t;
  t.begin(p);
  t.set_request(1.5f);   // user's dial value: the settle level
  t.seat_request(2.5f);  // ignition started at full power
  // room 18.5C (approach band below target 19C): descend toward 1.5
  const auto r = t.control(running_tlm(150), 18.5f, 19.0f, HeaterMode::ON, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.4f, 1e-4);  // one step down, not below user level
}

TEST(heat_manual_keeps_descending_above_request) {
  const HeaterProfile p = vevor_v2_profile();
  Thermostat t;
  t.begin(p);
  t.set_request(1.5f);
  t.seat_request(2.0f);
  // room 20.3C: overshoot 1.3 >= 1.0 -> 3x step down (2.0 - 0.3 = 1.7)
  const auto r = t.control(running_tlm(150), 20.3f, 19.0f, HeaterMode::ON, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 1.7f, 1e-4);
  CHECK(r.request_stop == false);  // HEAT never stops on overshoot
}

// --- Thermostat: mode entry (Phase 3b - shares threshold with control) ---

TEST(enter_auto_cold_starts_at_ignition_rate) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  const auto r = t.enter_mode(HeaterMode::AUTO, 15.0f, 19.0f, false,
                              params_for(p), 1000);
  CHECK(r.request_start == true);
  CHECK_NEAR(r.pump_request_hz, 1.8f, 1e-4);  // seated at pump_freq_initial
  CHECK(t.in_standby() == false);
}

TEST(enter_auto_warm_parks_in_standby_and_stops_if_running) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  // room 20 >= threshold 18 -> standby; heater running -> stop
  const auto r = t.enter_mode(HeaterMode::AUTO, 20.0f, 19.0f, true,
                              params_for(p), 1000);
  CHECK(r.request_stop == true);
  CHECK(r.enter_cooldown == true);
  CHECK(t.in_standby() == true);
}

TEST(enter_auto_stale_no_blind_start) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  auto params = params_for(p);
  params.external_sensor_present = true;
  params.sensor_stale = true;
  const auto r = t.enter_mode(HeaterMode::AUTO, 15.0f, 19.0f, true, params, 1000);
  // stale: park in standby (even though cold enough), stop the running heater
  CHECK(r.request_start == false);
  CHECK(r.request_stop == true);
  CHECK(t.in_standby() == true);
}

TEST(enter_on_seats_full_power_only_for_manual_profiles) {
  // manual profile (v2): ON seats FULL power
  const HeaterProfile p2 = vevor_v2_profile();
  Thermostat t2;
  t2.begin(p2);
  const auto r2 = t2.enter_mode(HeaterMode::ON, 20.0f, 19.0f, false,
                                params_for(p2), 1000);
  CHECK(r2.request_start == true);
  CHECK_NEAR(r2.pump_request_hz, 2.5f, 1e-4);  // v2 max

  // modulating profile (8kW): ON seats the ignition rate
  const HeaterProfile p1 = jeabong_profile();
  Thermostat t1;
  t1.begin(p1);
  const auto r1 = t1.enter_mode(HeaterMode::ON, 20.0f, 19.0f, false,
                                params_for(p1), 1000);
  CHECK(r1.request_start == true);
  CHECK_NEAR(r1.pump_request_hz, 1.8f, 1e-4);  // 8kW initial
}

TEST(enter_off_stops_only_when_running) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  const auto r = t.enter_mode(HeaterMode::OFF, 18.0f, 19.0f, true,
                              params_for(p), 1000);
  CHECK(r.request_stop == true);
  CHECK(r.enter_cooldown == true);
  CHECK(t.in_standby() == false);
  CHECK(t.in_auto_shutdown() == false);

  // not running: no commands at all
  Thermostat t2;
  t2.begin(p);
  const auto r2 = t2.enter_mode(HeaterMode::OFF, 18.0f, 19.0f, false,
                                params_for(p), 1000);
  CHECK(r2.request_stop == false);
  CHECK(r2.enter_cooldown == false);
}

// --- CombustionSafety: HX overrides (via Thermostat) ---

TEST(hx_soot_floor_forces_increase_on_reduce) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.5f);
  // HX 185 < hx_temp_low 190 and thermostat wants to reduce -> force increase
  const auto r = t.control(running_tlm(185), 18.6f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.6f, 1e-4);  // pump_freq_setting_ + step
}

TEST(hx_too_hot_forces_decrease_on_increase) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.5f);
  // HX 256 >= 255 and thermostat wants to increase (cold room) -> force decrease
  const auto r = t.control(running_tlm(256), 15.0f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r.pump_request_hz, 2.4f, 1e-4);
}

TEST(hx_critical_triggers_emergency_stop) {
  const HeaterProfile p = jeabong_profile();
  Thermostat t;
  t.begin(p);
  t.seat_request(2.5f);
  const auto r = t.control(running_tlm(266), 15.0f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK(r.request_stop == true);
  CHECK(r.enter_cooldown == true);
  CHECK(t.in_auto_shutdown() == false);
}

TEST(manual_heat_skips_hx_soot_increase) {
  const HeaterProfile p = vevor_v2_profile();
  Thermostat t;
  t.begin(p);
  t.set_request(1.0f);
  t.seat_request(2.0f);
  // HX 140 < 145 while the user's burn is at 2.0 and the room is warm enough
  // that HEAT-manual wants to reduce: no forced soot-increase in manual-HEAT
  const auto r = t.control(running_tlm(140), 18.6f, 19.0f, HeaterMode::ON, params_for(p), 6000);
  CHECK(r.request_stop == false);
  // approach band: descend one step toward 1.0
  CHECK_NEAR(r.pump_request_hz, 1.9f, 1e-4);
}

TEST(oversight_optout_disables_hx_soot_override) {
  const HeaterProfile p = vevor_v2_profile();
  // oversight OFF: the thermostat descends normally through the approach band
  Thermostat t;
  t.begin(p);
  t.seat_request(2.0f);
  auto params = params_for(p);
  params.oversight_enabled = false;
  const auto r = t.control(running_tlm(140), 18.6f, 19.0f, HeaterMode::AUTO, params, 6000);
  CHECK_NEAR(r.pump_request_hz, 1.9f, 1e-4);
  // oversight ON (default): the soot floor overrides the same reduction
  Thermostat t2;
  t2.begin(p);
  t2.seat_request(2.0f);
  const auto r2 = t2.control(running_tlm(140), 18.6f, 19.0f, HeaterMode::AUTO, params_for(p), 6000);
  CHECK_NEAR(r2.pump_request_hz, 2.1f, 1e-4);
}

// --- CombustionSafety: fan computation ---

TEST(fan_scales_with_fuel_rate) {
  const HeaterProfile p = jeabong_profile();
  CombustionSafety s;
  s.begin(p, true);
  CHECK(s.fan_rpm(0.0f, running_tlm(200)) == 1450);
  CHECK(s.fan_rpm(0.5f, running_tlm(200)) == 2975);
  CHECK(s.fan_rpm(1.0f, running_tlm(200)) == 4500);
}

TEST(fan_ignition_hold_below_hx_threshold) {
  const HeaterProfile p = jeabong_profile();
  CombustionSafety s;
  s.begin(p, true);
  Telemetry tlm;
  tlm.run_state = RUN_STATE_IGNITED;
  tlm.on = true;
  tlm.hx_temp = 80.0f;  // < 100 threshold
  CHECK(s.fan_rpm(1.0f, tlm) == 2000);
  // above the threshold the fan scales again
  tlm.hx_temp = 150.0f;
  CHECK(s.fan_rpm(1.0f, tlm) == 4500);
}

TEST(fan_cooldown_purge_rpm) {
  const HeaterProfile p = jeabong_profile();
  CombustionSafety s;
  s.begin(p, true);
  s.begin_cooldown();
  CHECK(s.fan_rpm(0.5f, running_tlm(200)) == 4000);
  s.end_cooldown();
  // heater's own cooldown state also purges
  Telemetry tlm = running_tlm(200);
  tlm.run_state = 8;  // RUN_STATE_COOLDOWN
  CHECK(s.fan_rpm(0.5f, tlm) == 4000);
}

TEST(cooldown_completes_at_target_temp) {
  const HeaterProfile p = jeabong_profile();
  CombustionSafety s;
  s.begin(p, true);
  s.begin_cooldown();
  CHECK(s.cooldown_complete(61.0f) == false);  // target 60
  CHECK(s.cooldown_complete(60.0f) == true);
  CHECK(s.cooldown_complete(45.0f) == true);
}

TEST(ambient_limit_only_in_heat_mode) {
  CombustionSafety s;
  s.begin(jeabong_profile(), true);
  CHECK(s.ambient_limit_exceeded(HeaterMode::ON, 27.0f, 26.0f) == true);
  CHECK(s.ambient_limit_exceeded(HeaterMode::ON, 26.0f, 26.0f) == false);  // strictly above
  CHECK(s.ambient_limit_exceeded(HeaterMode::AUTO, 27.0f, 26.0f) == false);  // AUTO handles its own
  CHECK(s.ambient_limit_exceeded(HeaterMode::OFF, 27.0f, 26.0f) == false);
}

// --- RoomSensorGuard ---

TEST(room_sensor_rejects_nan) {
  RoomSensorGuard g;
  // establish the baseline first (3 samples, >=5 s apart)
  CHECK(g.refresh(19.0f, 0) == false);
  CHECK(g.refresh(19.0f, 6000) == false);
  CHECK(g.refresh(19.1f, 12000) == true);
  // NaN sample rejected outright; trusted value held
  CHECK(g.refresh(std::nanf(""), 22000) == false);
  CHECK_NEAR(g.value(), 19.0f, 1e-4);  // median of {19.0, 19.0, 19.1}
}

TEST(room_sensor_boot_requires_consensus) {
  RoomSensorGuard g;
  // first counted sample
  CHECK(g.refresh(19.0f, 0) == false);
  CHECK(g.stale(1000) == true);            // no baseline yet
  // second call <5 s later: same conversion, NOT counted
  CHECK(g.refresh(19.1f, 2000) == false);
  // second counted sample (>=5 s after the first)
  CHECK(g.refresh(19.1f, 6000) == false);
  // third counted sample -> consensus -> baseline = median
  CHECK(g.refresh(19.2f, 12000) == true);
  CHECK_NEAR(g.value(), 19.1f, 1e-4);      // median of {19.0, 19.1, 19.2}
  CHECK(g.stale(13000) == false);
}

TEST(room_sensor_boot_disagreement_retries) {
  RoomSensorGuard g;
  CHECK(g.refresh(19.0f, 0) == false);
  CHECK(g.refresh(19.1f, 6000) == false);
  // third sample wildly different -> spread 5.5 > 2.0 -> window discarded
  CHECK(g.refresh(24.5f, 12000) == false);
  CHECK(g.stale(13000) == true);
  // next three samples start over
  CHECK(g.refresh(19.2f, 20000) == false);
  CHECK(g.refresh(19.2f, 26000) == false);
  CHECK(g.refresh(19.3f, 32000) == true);
  CHECK_NEAR(g.value(), 19.2f, 1e-4);
}

TEST(room_sensor_glitch_step_rejected) {
  RoomSensorGuard g;
  // establish baseline
  CHECK(g.refresh(19.0f, 0) == false);
  CHECK(g.refresh(19.0f, 6000) == false);
  CHECK(g.refresh(19.1f, 12000) == true);
  // plausible drift accepted
  CHECK(g.refresh(19.4f, 22000) == true);
  CHECK_NEAR(g.value(), 19.4f, 1e-4);
  // glitch jump (2.1C step) rejected, last trusted value held
  CHECK(g.refresh(21.5f, 32000) == false);
  CHECK_NEAR(g.value(), 19.4f, 1e-4);
}

TEST(room_sensor_persistent_badness_declares_blind) {
  RoomSensorGuard g;
  // establish baseline at t=0
  CHECK(g.refresh(19.0f, 0) == false);
  CHECK(g.refresh(19.0f, 6000) == false);
  CHECK(g.refresh(19.1f, 12000) == true);
  // from here on every sample glitches -> timer never refreshed
  for (uint32_t t = 22000; t < 640000; t += 10000) {
    CHECK(g.refresh(23.0f, t) == false);  // always rejected
  }
  // after the last accepted reading (12000) + 10 min -> blind
  CHECK(g.stale(640000) == true);
  CHECK(g.stale(600000) == false);  // exactly 10 min boundary not yet passed
}
