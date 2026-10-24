#!/usr/bin/env bash
# Syntax-check every ESPHome-facing translation unit against the minimal
# stub headers (tests/stubs/) so typos in log calls, API mismatches and
# signature errors surface WITHOUT a full device compile.
#
# Run from the repo root:  tests/stub_check.sh
# Exit 0 = all TUs clean.
set -uo pipefail
cd "$(dirname "$0")/.."

fail=0
for f in components/heater_uart/*.cpp; do
  out=$(g++ -std=c++17 -fsyntax-only -Wall -Wextra -Wformat \
        -Itests/stubs -Icomponents/heater_uart "$f" 2>&1)
  if [ -n "$out" ]; then
    echo "STUB-CHECK FAIL: $f"
    echo "$out" | head -15
    fail=1
  fi
done
# header-only entity
out=$(g++ -std=c++17 -fsyntax-only -Wall -Wextra -Wformat \
      -Itests/stubs -Icomponents/heater_uart -x c++ \
      components/heater_uart/fuel_prime_button.h 2>&1)
if [ -n "$out" ] && ! echo "$out" | grep -q "pragma once"; then
  echo "STUB-CHECK FAIL: fuel_prime_button.h"
  echo "$out" | head -15
  fail=1
fi
[ $fail -eq 0 ] && echo "stub-check: all component TUs clean"
exit $fail
