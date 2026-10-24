#pragma once

// Header-only micro test framework for the host-side harness (tests/).
// Zero dependencies (C++17 standard library only) so the harness builds with
// plain `g++` anywhere — no doctest/catch download needed.
//
// Usage:
//   TEST(my_feature) { CHECK(...); CHECK_NEAR(a, b, 1e-4); }
//   // in exactly one TU: #include "tiny_test.h" + TINYTEST_MAIN()

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace tinytest {

struct TestCase {
  std::string name;
  std::function<void()> fn;
};

inline std::vector<TestCase> &registry() {
  static std::vector<TestCase> r;
  return r;
}

inline int &failures() {
  static int f = 0;
  return f;
}

inline int &checks() {
  static int c = 0;
  return c;
}

struct Registrar {
  Registrar(const char *name, std::function<void()> fn) {
    registry().push_back({name, std::move(fn)});
  }
};

inline int run_all() {
  int ran = 0;
  for (const auto &tc : registry()) {
    std::printf("[ RUN ] %s\n", tc.name.c_str());
    const int before = failures();
    tc.fn();
    ++ran;
    if (failures() > before) {
      std::printf("[FAIL] %s\n\n", tc.name.c_str());
    } else {
      std::printf("[ ok ] %s\n\n", tc.name.c_str());
    }
  }
  std::printf("=== %d test(s), %d check(s), %d failure(s) ===\n", ran, checks(), failures());
  return failures() == 0 ? 0 : 1;
}

}  // namespace tinytest

#define TINYTEST_MAIN()                                    \
  int main() { return tinytest::run_all(); }

#define TEST(name)                                                       \
  static void tinytest_##name();                                          \
  static tinytest::Registrar tinytest_reg_##name(#name, tinytest_##name); \
  static void tinytest_##name()

#define CHECK(cond)                                                     \
  do {                                                                  \
    tinytest::checks()++;                                               \
    if (!(cond)) {                                                       \
      tinytest::failures()++;                                           \
      std::printf("  CHECK failed: %s  (%s:%d)\n", #cond, __FILE__, __LINE__); \
    }                                                                   \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
  do {                                                                     \
    tinytest::checks()++;                                                  \
    const double _ta = static_cast<double>(a);                              \
    const double _tb = static_cast<double>(b);                              \
    const double _tt = static_cast<double>(tol);                            \
    if (std::fabs(_ta - _tb) > _tt) {                                       \
      tinytest::failures()++;                                               \
      std::printf("  CHECK_NEAR failed: %s=%g vs %s=%g (tol %g)  (%s:%d)\n", \
                  #a, _ta, #b, _tb, _tt, __FILE__, __LINE__);                \
    }                                                                      \
  } while (0)
