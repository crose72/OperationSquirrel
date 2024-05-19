CC = g++
CFLAGS = -I. -I./lib/mavlink/v2.0/common -I./apphdr -I./lib -I./tests

# get list of all .cpp files in the appsrc directory
SRCS := $(wildcard appsrc/*.cpp)

# generate list of object files in the obj directory
OBJ := $(patsubst appsrc/%.cpp,obj/%.o,$(SRCS))

# Define the configuration flag
# Use UART functionality on Jetson Nano
# CFLAGS += -DUSE_UART
# Use TCP functionality
CFLAGS += -DUSE_TCP

# default target
all: main

# create the obj directory if it doesn't exist
obj:
	mkdir -p obj

# compile source code into object files
obj/%.o: appsrc/%.cpp | obj
	$(CC) $(CFLAGS) -c $< -o $@

# link object files into executable
main: obj $(OBJ)
	$(CC) $(OBJ) -o $@ -lrt -lpthread

# clean up object and executable files
clean:
	rm -f $(OBJ) main
	rm -rf obj
