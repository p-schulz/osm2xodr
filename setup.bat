@echo off
setlocal enabledelayedexpansion
REM Fetches vendored third-party sources into external/, bootstraps vcpkg, and configures
REM the CMake preset matching this platform (Windows x64/ARM64). See setup.sh for macOS/Linux.

set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%" || exit /b 1

set "WITH_GUI=0"
set "WITH_VALIDATION=0"
set "DO_CONFIGURE=1"
set "FORCE=0"

:parse_args
if "%~1"=="" goto args_done
if /i "%~1"=="--gui" set "WITH_GUI=1" & shift & goto parse_args
if /i "%~1"=="--validate" set "WITH_VALIDATION=1" & shift & goto parse_args
if /i "%~1"=="--force" set "FORCE=1" & shift & goto parse_args
if /i "%~1"=="--no-configure" set "DO_CONFIGURE=0" & shift & goto parse_args
if /i "%~1"=="--debug" shift & goto parse_args
if /i "%~1"=="-h" set "USAGE_EXIT=0" & goto usage
if /i "%~1"=="--help" set "USAGE_EXIT=0" & goto usage
echo Unknown option: %~1 1>&2
set "USAGE_EXIT=1"
goto usage

:usage
echo Usage: setup.bat [options]
echo.
echo Fetches the vendored dependencies under external\ (imgui, libosmium, protozero,
echo stb, tinyfiledialogs, libOpenDRIVE), bootstraps vcpkg if needed, and runs
echo `cmake --preset` for this architecture.
echo.
echo Options:
echo   --gui             Also enable the GUI (osm2xodr-gui) build
echo   --validate        Enable libOpenDRIVE read-back validation (--validate CLI flag)
echo   --force           Re-fetch dependencies even if external\^<name^> already exists
echo   --no-configure    Only fetch dependencies and bootstrap vcpkg, skip cmake --preset
echo   --debug           Accepted for symmetry with setup.sh; Visual Studio is a
echo                     multi-config generator, select Debug/Release at build time instead:
echo                     cmake --build build\^<preset^> --config Debug
echo   -h, --help        Show this help
popd
exit /b %USAGE_EXIT%

:args_done

where git >nul 2>nul
if errorlevel 1 (
    echo error: required tool 'git' not found in PATH 1>&2
    popd
    exit /b 1
)
where cmake >nul 2>nul
if errorlevel 1 (
    echo error: required tool 'cmake' not found in PATH 1>&2
    popd
    exit /b 1
)

echo ==^> Fetching vendored dependencies into external\

call :fetch_dep imgui https://github.com/ocornut/imgui.git v1.92.8 external\imgui
call :fetch_dep libosmium https://github.com/osmcode/libosmium.git v2.23.1 external\libosmium
call :fetch_dep protozero https://github.com/mapbox/protozero.git v1.8.2 external\protozero
call :fetch_dep stb https://github.com/nothings/stb.git master external\stb
call :fetch_dep tinyfiledialogs https://git.code.sf.net/p/tinyfiledialogs/code master external\tinyfiledialogs
call :fetch_dep libOpenDRIVE https://github.com/pageldev/libOpenDRIVE.git main external\libOpenDRIVE

REM --- vcpkg bootstrap ---------------------------------------------------------------
if "%VCPKG_ROOT%"=="" (
    set "VCPKG_ROOT=%SCRIPT_DIR%vcpkg"
    echo ==^> VCPKG_ROOT not set, defaulting to !VCPKG_ROOT!
)

if not exist "%VCPKG_ROOT%\.git" (
    echo ==^> Cloning vcpkg into %VCPKG_ROOT%
    git clone --quiet https://github.com/microsoft/vcpkg.git "%VCPKG_ROOT%"
    if errorlevel 1 (
        echo error: failed to clone vcpkg 1>&2
        popd
        exit /b 1
    )
) else (
    echo ==^> vcpkg already present at %VCPKG_ROOT%, skipping clone
)

if not exist "%VCPKG_ROOT%\vcpkg.exe" (
    echo ==^> Bootstrapping vcpkg
    call "%VCPKG_ROOT%\bootstrap-vcpkg.bat" -disableMetrics
    if errorlevel 1 (
        echo error: vcpkg bootstrap failed 1>&2
        popd
        exit /b 1
    )
)

setx VCPKG_ROOT "%VCPKG_ROOT%" >nul
echo ==^> VCPKG_ROOT set to %VCPKG_ROOT% ^(persisted for new sessions via setx^)

REM --- pick a preset for this architecture -------------------------------------------
set "PRESET=windows-x64"
if /i "%PROCESSOR_ARCHITECTURE%"=="ARM64" set "PRESET=windows-arm64"
if /i "%PROCESSOR_ARCHITEW6432%"=="ARM64" set "PRESET=windows-arm64"

if "%DO_CONFIGURE%"=="0" (
    echo ==^> Dependencies fetched and vcpkg bootstrapped. Skipping configure ^(--no-configure given^).
    echo ==^> Configure later with: cmake --preset %PRESET%
    popd
    exit /b 0
)

set "CMAKE_ARGS="
if "%WITH_GUI%"=="1" set "CMAKE_ARGS=%CMAKE_ARGS% -DOSM2XODR_BUILD_GUI=ON"
if "%WITH_VALIDATION%"=="1" set "CMAKE_ARGS=%CMAKE_ARGS% -DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON -DOSM2XODR_LIBOPENDRIVE_SOURCE_DIR=%SCRIPT_DIR%external\libOpenDRIVE"

echo ==^> Configuring preset '%PRESET%'
cmake --preset %PRESET% %CMAKE_ARGS%
if errorlevel 1 (
    popd
    exit /b 1
)

echo ==^> Done. Build with: cmake --build build\%PRESET% --config Release
popd
exit /b 0

REM --- subroutine: fetch_dep <name> <url> <ref> <dest> -------------------------------
:fetch_dep
set "DEP_NAME=%~1"
set "DEP_URL=%~2"
set "DEP_REF=%~3"
set "DEP_DEST=%~4"

if exist "%DEP_DEST%\*" (
    if "%FORCE%"=="1" (
        echo ==^> %DEP_NAME%: --force given, removing existing %DEP_DEST%
        rmdir /s /q "%DEP_DEST%"
    ) else (
        echo ==^> %DEP_NAME%: %DEP_DEST% already present, skipping ^(use --force to re-fetch^)
        exit /b 0
    )
)

echo ==^> %DEP_NAME%: cloning %DEP_REF% from %DEP_URL%
git clone --quiet --branch %DEP_REF% --depth 1 %DEP_URL% %DEP_DEST%
exit /b 0
