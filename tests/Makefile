# Host-side Autumn tests. Not part of the PROS `make` or the template zip.
# CXX_STANDARD matches common.mk. GCC 14+ accepts gnu++26.
CXX ?= g++
CXX_STANDARD ?= gnu++26
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
