# Install log - windows runtime
### Machine:
- Windows 11 Pro
- Laptop, asus ROG Zephyrus M16 GU603ZM
- Graphics, nVidia GEFORCE RTX 3060 Laptop
- Memory, 40 GB
- HDD, 2 physical C & D
### Log:
- refering to https://jinscott.medium.com/build-opencv-on-windows-with-cuda-f880270eadb0
  - note: cuDNN link is to archive, ends in 2023 as of 2024/11/12
  - using https://developer.nvidia.com/cudnn 9.5.1 windows,x86_64,10,exe
- install CUDA framework 12.6, windows,x86_64,11,exe (local)
  - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
- install cuDNN
  - Copy bin,include,lib into ...CUDA/v12.6/bin,include,lib
- set environment variables for CUDA
  - System Properties, Advanced, Environment Variables ...
  - Edit Path, to add:
    - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin
    - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\lib
    - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\include
    - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\lib\x64
    - And for cuDNN:
      - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin\12.6
      - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\lib\12.6\x64
      - C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\include\12.6
- install cmake
  - https://github.com/Kitware/CMake/releases/download/v3.31.0/cmake-3.31.0-windows-x86_64.msi
  - Select D:\Program Files\ for install location
  - Elect to have desktop shortcut
- install Chocolatey (choco) package manager for system utils / general packages
  - open powershel as admin
  - `Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.SecurityProtocolType]::Tls12; Invoke-Expression ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
  - test in powershell install w run `choco` gives v2.3.0
- install `make` using `choco`
  - open powershell as admin
  - install via `choco install make`
  - observe success (make v4.4.1)
- install C++ or C compiler (gcc / MinGW) using `choco`
  - install via `choco install mingw`, Y to prompts
  - observe success (mingw v13.2.0)
   - PATH updated: C:\ProgramData\mingw64\bin
   - install to 'C:\ProgramData\chocolatey\lib\mingw\tools\install'
   - in new console, command `gcc` returns real response
- install Visual Studio Community Edition
  - note: this is reflucking tarded. NVidia CUDA toolkit relies on VS's compiler in windows
  - note: requires 50g.. jfc lol wtf 
  - go to microsoft download installer, install w all options except the .NET stuff
  - alternatively; can try using choco for headless install but be prepared to wait a long time:
    - open powershell as admin
    - install via `choco install visualstudio2022community --package-parameters "--add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended --includeOptional --installPath D:\Applications\VS2022"

choco install visualstudio2022community --package-parameters "--add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended --includeOptional"`
- extract OpenCV & opencv_contrib
  - OpenCV src https://github.com/opencv/opencv/archive/refs/tags/4.10.0.zip
  - extract to D:\src\opencv-4.10.0
  - opencv_contrib https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.zip
  - extract to D:\src\opencv_contrib-4.10.0
- install OpenCV 4.10 from source
  - run cmake gui
    - note: tried & failed to diverge from medium instructions, using vscode not VS 17
    - ...require visual studio to compile binaries w CUDA support (jeeles nvidia...)
    - ... tried and failed to specify the generator for this project = Unix Makefiles
  - if you get a similar error in "Configure" step, you need to clear the cache in CMake w `File -> clear cache`, and check your using 'Unix Makefiles' after `configure`:
    ```
    CMake Error at CMakeLists.txt:127 (enable_language):
    Generator
      Visual Studio 17 2022
    could not find any instance of Visual Studio.
    ```
  - set OPENCV_EXTRA_MODULES_PATH to D:/src/opencv_contrib-4.10.0/opencv_contrib-4.10.0/modules
    - click "Configure" 
    - observe "Configuring done (16.2s)
  - set to "checked" OPENCV_DNN_CUDA, WITH_CUDA
    - click "Configure"
    - observe error.. gdamn it. fix path & install Visual Studio Community edition w choco
      ```
      CMake Error at modules/dnn/CMakeLists.txt:49 (message):
      DNN: CUDA backend requires CUDA Toolkit.  Please resolve dependency or
      disable OPENCV_DNN_CUDA=OFF
      ```
    - a few hours later... (jk)
    - click "Configure"
    - observe works yay!
  - set CUDA_ARCH_BIN to work with selected hw
    - GEFORCE RTX 3000 series (8.6)
    - Jetson Nano (5.3)
    - Jetson Orin Nano (8.7)
    - 5.3;8.6;8.7
    - click "Configure", observe success
  - set BUILD_SHARED_LIBS to FALSE, and BUILD_opencv_world TRUE
    - this will generate a monolithic singular .lib file for easy inclusion in project
    - click "Configure", observe success
  - build the cmake pipeline
    - select "Generate"
    - observe "Generating done (24.1s)
    - open D:/src/opencv to observe build
  - generate the library
    - open vs 2022 community
    - select folder of output of cmake D:\src\opencv
    - when prompted by vscode upon detection of CMakeLists.txt, do not click enable
    - double click "OpenCv.sln" the vs workspace will load w the Cmake configuration correct
    - change "Debug" to "Release"
    - in "Solution Explorer" (file browser), expand CMakeTargets
      - right click "ALL_BUILD"
      - select "Build"
      - observe success, took 1h40m
      - right click "INSTALL"
      - select "Build"
      - observe success, took 25s
      - observe products, D:\src\opencv\install
        - monolithic staticlib: 
        `D:\src\opencv\install\x64\vc17\staticlib\opencv_world4100.lib` 957 MiB
        - headers for C++ / C:
        `D:\src\opencv\install\include\opencv2`
    - add 
  - use CMAKE to setup OpenCV's DNN library to use nvidia gpu
    - info: OpenCV DNN: inference on img / vid, cuDNN: training
    - open CMAKE gui
    - create a new project in VS using Console App C++ template
      - name "OpencvTestCuda"
      - prints "Hello World" by default
      - change code as follows in OpencvTestCuda.cpp :
      ```
      #include <iostream>
      #include <opencv2/opencv.hpp>
      #include <opencv2/cudaimgproc.hpp>


      int main() {
      
       cv::cuda::setDevice(0);
       cv::VideoCapture cap(0);
       cv::Mat frame;
       cv::cuda::GpuMat Gframe;

       if (!cap.isOpened())
       {
        std::cout << "No camera exist\n";
        return -1;
       }

       while (1) {
        cap >> frame;
        Gframe.upload(frame);
        cv::cuda::cvtColor(Gframe, Gframe, cv::COLOR_BGR2GRAY);
        Gframe.download(frame);
        cv::imshow("Gray webcam", frame);
        if (cv::waitKey(1) == 27) { // Press esc
         std::cout << "End camera loop\n";
         return 1;
        }
       }
       return 0;
      }
      ```
      - right click project in Solution Explorer, go to properties
      - add path to OpenCV libraries in Linker, Add. Library Dir's
        - D:\src\opencv\install\x64\vc17\staticlib
      - add path to OpenCV headers in C/C++
        - D:\src\opencv\install\include
      - add path opencv world, note this is a "release" level 0 library (need to repeate for debug..)
      - change C/C++, Code Generation, Runtime Library to Multi-threaded (/MT) from /MD
      


