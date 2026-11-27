# Host-side Autumn tests. Not part of the PROS `make` or the template zip.
# Prefer gnu++26 (matches common.mk) but fall back to gnu++20 so the suite
# builds on distro compilers older than GCC 14. Override with CXX_STANDARD=.
CXX ?= g++
CXX_STANDARD ?= $(shell echo 'int main(){}' | $(CXX) -std=gnu++26 -x c++ -fsyntax-only - >/dev/null 2>&1 \
	&& echo gnu++26 || echo gnu++20)
ROOT := ..
BUILD := build

AUTUMN_SRC := $(wildcard $(ROOT)/src/autumn/*.cpp) \
	$(wildcard $(ROOT)/src/autumn/*/*.cpp) \
	$(wildcard $(ROOT)/src/autumn/*/*/*.cpp)
AUTUMN_SRC := $(filter-out $(ROOT)/src/autumn/ui/auton_selector.cpp,$(AUTUMN_SRC))

SRCS := $(AUTUMN_SRC) $(wildcard *.cpp) stubs/pros/detail/scheduler.cpp
CXXFLAGS := -std=$(CXX_STANDARD) -O1 -g -pthread -D_USE_MATH_DEFINES \
	-Istubs -I$(ROOT)/include -I. \
	-Wall -Wextra -Wno-unused-parameter -Wno-deprecated-literal-operator
TARGET := $(BUILD)/autumn_tests

.PHONY: all clean
all: $(TARGET)
	$(TARGET)

$(BUILD):
	mkdir -p $(BUILD)

$(TARGET): $(SRCS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@

clean:
	$(RM) -r $(BUILD)
