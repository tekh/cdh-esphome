# Host-side test harness for heater_uart logic.
# Pure C++ (no ESPHome dependency) — the extracted logic must stay includable
# from plain g++. Run: make -C tests
#
# As ROADMAP Phase 2/3 extract more logic (ProtocolDriver encode/decode,
# SafetyPolicy, ControlPolicy, room-sensor guard), their test files join here.

CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -g
INCLUDES := -I../components/heater_uart

TEST_SRCS := $(wildcard test_*.cpp)
TEST_OBJS := $(TEST_SRCS:.cpp=.o)

.PHONY: all test clean

all: test

test: $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) -o test_run $^
	./test_run

%.o: %.cpp tiny_test.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f *.o test_run
