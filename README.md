# Overview
### This repository contains the source code necessary for running the camera application.


# Building

## Windows
### Prerequisites
- Visual Studio
	- CMake
	- C++ Desktop Development workload
- Install vcpkg and install the following dependencies:
	- realsense2
	- SDL2

#### Building the Project
```powershell
git clone <repository_url>
```
Open the project as a local folder through Visual Studio.
Select camera.exe as the startup item on the top middle dropdown next to the green play button.

## Linux
### Prerequisites
- GCC or Clang
- CMake
- Install the following dependencies using your package manager:
	- libsdl2-dev

### Building the Project
```bash
git clone <repository_url>
cd <repository_directory>
mkdir build
cd build
cmake ..
cmake --build . --target camera -j8
```
the executable will be located in the `build` directory as `camera`.