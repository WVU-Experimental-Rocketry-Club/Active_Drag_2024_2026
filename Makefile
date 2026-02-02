# Top-level Makefile for Active_Drag_2024_2026
# Builds the mothmansRevenge desktop binary (and can be extended for other targets)

SRC_DIR := flightCodeSrc/mothmansRevenge
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(SRCS:.cpp=.o)
TARGET := $(SRC_DIR)/desktop

CXX := g++
CXXFLAGS := -g -O2 -std=c++17 -Wall -I$(SRC_DIR)
LDFLAGS :=

.PHONY: all build clean run

all: build

build: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(SRC_DIR)/*.o $(TARGET)

run: build
	$(TARGET)
