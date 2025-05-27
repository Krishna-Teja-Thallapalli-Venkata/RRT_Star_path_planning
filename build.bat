@echo off
REM build.bat - Windows build script for RRT* Path Planning

echo ===============================================
echo RRT* Path Planning Project Windos Build Script
echo Using MSVC (Visual Studio) Compiler
echo ===============================================

REM Check if cmake is installed
cmake --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: CMake is not installed or not in PATH
    echo Please install CMake from https://cmake.org/download/
    pause
    exit /b 1
)

REM Set Visual Studio 2022 Generator 
set VS_GENERATOR=Visual Studio 17 2022
set VS_ARCH=x64

REM 
if not exist "build" mkdir build
cd build

echo.
echo Configuring project with CMake...
REM 
REM 
cmake .. -G "%VS_GENERATOR%" -A %VS_ARCH% -DCMAKE_PREFIX_PATH="C:/opencv/build/x64/vc16/lib"

if errorlevel 1 (
    echo ERROR: CMake configuration failed
    pause
    exit /b 1
)

echo.
echo Building project...
REM 
REM 
REM 
REM 

cmake --build . --config Release

if errorlevel 1 (
    echo ERROR: Build failed
    pause
    exit /b 1
)

echo.
echo Build completed successfully!
echo Executable location: build/bin/Release/rrt_star_planner.exe
echo.


echo Build script completed.
pause