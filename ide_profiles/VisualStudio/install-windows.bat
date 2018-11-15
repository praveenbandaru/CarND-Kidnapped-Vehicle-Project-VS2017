@echo off

setlocal

set "CarNDKVProjectPlatform=x64"
set "CarNDKVProjectToolset=v141"
set "CarNDKVProjectBuildType=Debug"

if NOT "%~1"=="" set "CarNDKVProjectPlatform=%~1"
if NOT "%~2"=="" set "CarNDKVProjectToolset=%~2"
if NOT "%~3"=="" set "CarNDKVProjectBuildType=%~3" 

set "VcPkgDir=c:\vcpkg"
set "VcPkgTriplet=%CarNDKVProjectPlatform%-windows"
rem set "VcPkgTriplet=%CarNDKVProjectPlatform%-windows-%CarNDKVProjectToolset%"

if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" (
    set "VcPkgDir=%VCPKG_ROOT_DIR%"
)
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" (
    set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
)
set "VcPkgPath=%VcPkgDir%\vcpkg.exe"

echo. & echo Bootstrapping dependencies for triplet: %VcPkgTriplet% & echo.

rem ==============================
rem Update and Install packages
rem ==============================
call "%VcPkgPath%" update

rem Install latest uwebsockets
call "%VcPkgPath%" install uwebsockets --triplet %VcPkgTriplet%
rem Use adapted main.cpp for latest uwebsockets
copy main.cpp ..\..\src

rem ==============================
rem Configure CMake
rem ==============================

set "VcPkgTripletDir=%VcPkgDir%\installed\%VcPkgTriplet%"

set "CMAKE_PREFIX_PATH=%VcPkgTripletDir%;%CMAKE_PREFIX_PATH%"

echo. & echo Bootstrapping successful for triplet: %VcPkgTriplet% & echo CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH% & echo.

set "CarNDKVProjectCMakeGeneratorName=Visual Studio 15 2017"

if "%CarNDKVProjectPlatform%"=="x86" (
    if "%CarNDKVProjectToolset%"=="v140" set "CarNDKVProjectCMakeGeneratorName=Visual Studio 14 2015"
    if "%CarNDKVProjectToolset%"=="v141" set "CarNDKVProjectCMakeGeneratorName=Visual Studio 15 2017"
)

if "%CarNDKVProjectPlatform%"=="x64" (
    if "%CarNDKVProjectToolset%"=="v140" set "CarNDKVProjectCMakeGeneratorName=Visual Studio 14 2015 Win64"
    if "%CarNDKVProjectToolset%"=="v141" set "CarNDKVProjectCMakeGeneratorName=Visual Studio 15 2017 Win64"
)

set "CarNDKVProjectBuildDir=%~dp0\..\..\products\cmake.msbuild.windows.%CarNDKVProjectPlatform%.%CarNDKVProjectToolset%"
if not exist "%CarNDKVProjectBuildDir%" mkdir "%CarNDKVProjectBuildDir%"
cd "%CarNDKVProjectBuildDir%"

echo: & echo CarNDKVProjectBuildDir=%CD% & echo cmake.exe -G "%CarNDKVProjectCMakeGeneratorName%" "%~dp0\..\.." & echo:

call cmake.exe -G "%CarNDKVProjectCMakeGeneratorName%" "%~dp0\..\.."

call "%VcPkgPath%" integrate install

endlocal