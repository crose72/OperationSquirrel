# Compiler
CXX := g++
# Compiler flags
CXXFLAGS := -std=c++11 -Wall -Wextra
# Linker flags
LDFLAGS := -lrt
# Source files
SRCS := main.cpp interrupt_handler.cpp
# Object files
OBJS := $(SRCS:.cpp=.o)
# Output executable name
EXEC := main

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(EXEC) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(EXEC)

