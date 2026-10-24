#include "tiny_test.h"

// Output-domain mapping tests — the single shared conversion between the
// controller's fuel domains (normalized 0..1, pump Hz, power level 1..10).
// This mapping used to exist as four hand-copied variants; these tests pin
// the one true behaviour.

#include "heater_output.h"

using esphome::heater_uart::HeaterModel;
using esphome::heater_uart::HeaterProfile;
using esphome::heater_uart::profile_for;
namespace od = esphome::heater_uart::output_domain;

static HeaterProfile make_profile(float hz_min, float hz_max) {
  HeaterProfile p{};
  p.pump_freq_min = hz_min;
  p.pump_freq_max = hz_max;
  return p;
}

TEST(clamp01_bounds) {
  CHECK_NEAR(od::clamp01(-0.5f), 0.0f, 1e-6);
  CHECK_NEAR(od::clamp01(0.0f), 0.0f, 1e-6);
  CHECK_NEAR(od::clamp01(0.42f), 0.42f, 1e-6);
  CHECK_NEAR(od::clamp01(1.0f), 1.0f, 1e-6);
  CHECK_NEAR(od::clamp01(1.5f), 1.0f, 1e-6);
}

TEST(level_to_normalized_extremes) {
  CHECK_NEAR(od::normalized_from_level(1), 0.0f, 1e-6);
  CHECK_NEAR(od::normalized_from_level(10), 1.0f, 1e-6);
}

TEST(level_clamping) {
  // out-of-dial levels clamp to the dial's ends, never wrap
  CHECK_NEAR(od::normalized_from_level(0), 0.0f, 1e-6);
  CHECK_NEAR(od::normalized_from_level(255), 1.0f, 1e-6);
}

TEST(normalized_to_level_extremes) {
  CHECK(od::level_from_normalized(0.0f) == 1);
  CHECK(od::level_from_normalized(1.0f) == 10);
  // out-of-range fractions clamp to the dial's ends
  CHECK(od::level_from_normalized(-1.0f) == 1);
  CHECK(od::level_from_normalized(2.0f) == 10);
}

TEST(level_round_trip) {
  // every level maps to a fraction that maps back to the same level
  for (int level = 1; level <= 10; level++) {
    const float n = od::normalized_from_level(static_cast<uint8_t>(level));
    CHECK(od::level_from_normalized(n) == level);
  }
}

TEST(hz_mapping_midpoints) {
  const HeaterProfile p = make_profile(0.5f, 2.5f);
  CHECK_NEAR(od::normalized_from_hz(0.5f, p), 0.0f, 1e-6);
  CHECK_NEAR(od::normalized_from_hz(1.5f, p), 0.5f, 1e-6);
  CHECK_NEAR(od::normalized_from_hz(2.5f, p), 1.0f, 1e-6);
  CHECK_NEAR(od::hz_from_normalized(0.5f, p), 1.5f, 1e-6);
}

TEST(hz_clamping) {
  const HeaterProfile p = make_profile(1.3f, 5.5f);
  CHECK_NEAR(od::normalized_from_hz(-10.0f, p), 0.0f, 1e-6);
  CHECK_NEAR(od::normalized_from_hz(99.0f, p), 1.0f, 1e-6);
  CHECK_NEAR(od::hz_from_normalized(-0.5f, p), 1.3f, 1e-6);
  CHECK_NEAR(od::hz_from_normalized(1.5f, p), 5.5f, 1e-6);
}

TEST(hz_degenerate_profile_is_deterministic) {
  // min >= max must not divide by zero (or NaN onto the bus); it maps to 0.0
  const HeaterProfile flat = make_profile(2.0f, 2.0f);
  CHECK_NEAR(od::normalized_from_hz(2.0f, flat), 0.0f, 1e-6);
  CHECK_NEAR(od::normalized_from_hz(99.0f, flat), 0.0f, 1e-6);
  const HeaterProfile inverted = make_profile(3.0f, 1.0f);
  CHECK_NEAR(od::normalized_from_hz(2.0f, inverted), 0.0f, 1e-6);
}

TEST(level_hz_round_trip_on_real_profiles) {
  // the exact conversion the Vevor TX boundary + power dial use, on the
  // actual shipped profiles: level -> fraction -> Hz -> fraction -> level
  for (const HeaterModel m :
       {HeaterModel::VEVOR_2KW, HeaterModel::VEVOR_2KW_V2, HeaterModel::JEABONG_8KW}) {
    const HeaterProfile &p = profile_for(m);
    for (int level = 1; level <= 10; level++) {
      const float hz = od::hz_from_normalized(od::normalized_from_level(static_cast<uint8_t>(level)), p);
      const uint8_t back = od::level_from_normalized(od::normalized_from_hz(hz, p));
      CHECK(back == level);
    }
  }
}

TEST(level_encoding_matches_v1_formula) {
  // v1 shipped `level = 1 + (int)(ratio * 9 + 0.5)` at the TX boundary and the
  // dial. The shared mapping must reproduce that encoding exactly (including
  // half-step rounding), so a reflash is not a behaviour change on the bus.
  const HeaterProfile p = make_profile(0.0f, 1.0f);  // fraction == Hz, clean sweep
  for (int i = 0; i <= 100; i++) {
    const float n = i / 100.0f;
    const uint8_t v1 = static_cast<uint8_t>(1 + static_cast<int>(n * 9.0f + 0.5f));
    CHECK(od::level_from_normalized(od::normalized_from_hz(n, p)) == v1);
  }
}
