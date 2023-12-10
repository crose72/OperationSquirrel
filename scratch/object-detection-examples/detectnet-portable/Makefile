CC = g++
NVCC = nvcc
CFLAGS = -I./inc/mavlink/v2.0/common -I./jetson-utils -I./c -I.

# get list of all .cpp files in the source directory
SRCS := $(wildcard *.cpp)
CUDA_SRCS := $(wildcard *.cu)

# generate list of object files
OBJ := $(SRCS:.cpp=.o)
CUDA_OBJ := $(CUDA_SRCS:.cu=.o)

# Define the configuration flag
# Use UART functionality on Jetson Nano
CFLAGS += -DUSE_UART
# Use TCP functionality
# CFLAGS += -DUSE_TCP

# OpenCV flags
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4)
OPENCV_LIBS = $(shell pkg-config --libs opencv4)

# GStreamer flags
GST_CFLAGS = $(shell pkg-config --cflags gstreamer-1.0 gstreamer-video-1.0)
GST_LIBS = $(shell pkg-config --libs gstreamer-1.0 gstreamer-video-1.0)

# CUDA Toolkit paths (adjust these paths based on your installation)
CUDA_PATH := /usr/local/cuda
CUDA_INCLUDE := $(CUDA_PATH)/include
CUDA_LIB := $(CUDA_PATH)/lib64

# Combine all flags
CFLAGS += -I$(CUDA_INCLUDE) $(OPENCV_CFLAGS) $(GST_CFLAGS)
LIBS = -L$(CUDA_LIB) -lcudart -lrt $(OPENCV_LIBS) $(GST_LIBS)

# Add jetson-inference and jetson-utils libraries
LIBS += -L/usr/local/jetson-inference/lib -ljetson-inference
LIBS += -L/usr/local/jetson-utils/lib -ljetson-utils

# default target
all: main

# compile source code into object files
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# compile CUDA source code into object files
%.o: %.cu
	$(NVCC) $(CFLAGS) -c $< -o $@

# link object files into executable
main: $(OBJ) $(CUDA_OBJ)
	$(CC) $(OBJ) $(CUDA_OBJ) -o $@ $(LIBS)

# clean up object and executable files
clean:
	rm -f $(OBJ) $(CUDA_OBJ) main
