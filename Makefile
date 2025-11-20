# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g

# Executable names
TARGET_PHASE1 = phase1
TARGET_PHASE2 = phase2
TARGET_PHASE3 = phase3

# Common source
SRC_COMMON = SampleDriver.cpp

# Phase-specific sources
SRC_PHASE1 = $(SRC_COMMON) Phase1/graph.cpp Phase1/QueryProcessor.cpp
SRC_PHASE2 = $(SRC_COMMON) Phase2/graph.cpp Phase2/QueryProcessor.cpp
SRC_PHASE3 = $(SRC_COMMON) Phase3/graph.cpp Phase3/QueryProcessor.cpp

# Default target
all: $(TARGET_PHASE1) $(TARGET_PHASE2)

# Build Phase 1
$(TARGET_PHASE1): $(SRC_PHASE1)
	$(CXX) $(CXXFLAGS) -IPhase1 -DPHASE1 -o $@ $^

# Build Phase 2
$(TARGET_PHASE2): $(SRC_PHASE2)
	$(CXX) $(CXXFLAGS) -IPhase2 -DPHASE2 -o $@ $^

# Build Phase 3
$(TARGET_PHASE3): $(SRC_PHASE3)
	$(CXX) $(CXXFLAGS) -IPhase3 -DPHASE3 -o $@ $^

# Clean
clean:
	rm -f $(TARGET_PHASE1) $(TARGET_PHASE2) $(TARGET_PHASE3) *.o

.PHONY: all clean
