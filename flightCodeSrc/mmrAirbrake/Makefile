# Makefile for C++ Project

# Variables
CXX = g++
CXXFLAGS = -Wall -g
TARGET = desktop
OBJS = desktop.o computations.o

# The default target
all: $(TARGET)

# Rule to link object files into the final executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET)

# Rules to generate object files from source files
desktop.o: desktop.cpp computations.h config_data.h
	$(CXX) $(CXXFLAGS) -c desktop.cpp

computations.o: computations.cpp computations.h
	$(CXX) $(CXXFLAGS) -c computations.cpp
# Clean rule to remove generated files
clean:
	rm -f $(TARGET) $(OBJS)