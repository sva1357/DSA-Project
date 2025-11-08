# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra

# Executable names
TARGET_PHASE1 = phase1
TARGET_PHASE2 = phase2

# Common source
SRC_COMMON = SampleDriver.cpp

# Phase-specific sources
SRC_PHASE1 = $(SRC_COMMON) phase1/graph.cpp phase1/QueryProcessor.cpp
SRC_PHASE2 = $(SRC_COMMON) phase2/graph.cpp phase2/QueryProcessor.cpp

# Default target
all: $(TARGET_PHASE1) $(TARGET_PHASE2)

# Build Phase 1
$(TARGET_PHASE1): $(SRC_PHASE1)
	$(CXX) $(CXXFLAGS) -Iphase1 -DPHASE1 -o $@ $^

# Build Phase 2
$(TARGET_PHASE2): $(SRC_PHASE2)
	$(CXX) $(CXXFLAGS) -Iphase2 -DPHASE2 -o $@ $^

# Clean
clean:
	rm -f $(TARGET_PHASE1) $(TARGET_PHASE2) *.o

.PHONY: all clean
