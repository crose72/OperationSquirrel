# Define the compiler and compiler flags
CXX = g++
CXXFLAGS = -Wall -std=c++11

# Define the target executable
TARGET = debug-terminal

# Define the source files
SRC = main.cpp

# Rule to build the target executable
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

# Rule to clean up the build
clean:
	rm -f $(TARGET)

# Rule to run the program
run: $(TARGET)
	./$(TARGET)

# Add a rule to show the content of the Makefile
show:
	@cat Makefile
