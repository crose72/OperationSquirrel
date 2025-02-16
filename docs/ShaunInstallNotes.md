790  cd crose72
  791  git clone --recursive --depth=1 https://github.com/crose72/jetson-inference.git
  792  cd jetson-inference/
  793  mkdir build
  794  cd build/
  795  cmake ../
  796  history

~/git/crose72/OperationSquirrel/scripts

cat Jetson-Orin-Install.sh

cmake ../
Command 'cmake' not found, but can be installed with:
sudo snap install cmake  # version 3.31.4, or
sudo apt  install cmake  # version 3.22.1-1ubuntu1.22.04.2
See 'snap info cmake' for additional versions.


sudo apt install cmake




 sudo apt install plocate locate NvInfer


 mkdir build
cd build
cmake ../
make -j$(nproc) <---- fatal error: NvInfer.h: No such file or directory
 sudo apt install libnvinfer10 libnvinfer-dev  <>


Next problem:
[ 67%] Building CXX object CMakeFiles/jetson-inference.dir/c/tensorNet.cpp.o
/home/shaun/git/crose72/jetson-inference/c/tensorNet.cpp:35:10: fatal error: NvOnnxParser.h: No such file or directory
   35 | #include "NvOnnxParser.h"
      |          ^~~~~~~~~~~~~~~~

found it on system:
shaun@jetson-orin:~/git/crose72/jetson-inference/build$ sudo find / -name NvOnnxParser.h
/var/lib/docker/overlay2/p14b9svn6mfskuktx9gfwzkkl/diff/usr/include/aarch64-linux-gnu/NvOnnxParser.h
/var/lib/docker/overlay2/03a9f965a2fe5c8d972875aac831172fed24169903610e7ce0f9f9e35b4cadcc/diff/usr/include/aarch64-linux-gnu/NvOnnxParser.h
/home/shaun/.local/lib/python3.10/site-packages/tensorflow/include/third_party/gpus/cuda/include/NvOnnxParser.h

copyied it to jetson-inference:
shaun@jetson-orin:~/git/crose72/jetson-inference/c$ cp /home/shaun/.local/lib/python3.10/site-packages/tensorflow/include/third_party/gpus/cuda/include/NvOnnxParser.h ./

...more missing headers, also in my .local path
...adding this path to CMakeLists.txt:
include_directories(/home/shaun/.local/lib/python3.10/site-packages/tensorflow/include/third_party/gpus/cuda/include/)

Trying to install these too:
sudo apt install libnvinfer-plugin10 libnvinfer-plugin-dev libnvonnxparsers10 libnvonnxparsers-dev




---------------
trying to compile OperationSquirrel