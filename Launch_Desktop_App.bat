@echo off
title BlueROV2 AUV Digital Twin - Desktop Application
cd /d "%~dp0"

echo ======================================================================
echo    Starting BlueROV2 AUV Digital Twin (Native Desktop Software)
echo ======================================================================
echo.

:: 1. Start ROS 2 Desktop Simulator in Background (if Docker is available)
echo [1/3] Checking ROS 2 Underwater Simulator...
docker info >nul 2>&1
if %errorlevel% equ 0 (
    echo [OK] Docker active. Starting ROS2 Gazebo backend...
    cd simulator
    docker compose up -d >nul 2>&1
    cd ..
) else (
    echo [INFO] Docker not detected or offline. Running in Standalone Physics Mode.
)

:: 2. Ensure frontend bundle is built and ready
echo [2/3] Preparing Digital Twin Core Engine...
if not exist "dist\index.html" (
    echo [INFO] Building production bundle for first run...
    call npm run build
)

:: 3. Launch Native Electron Desktop Application Window
echo [3/3] Launching Native Desktop Window...
echo.
call npm run desktop

echo.
echo ======================================================================
echo Digital Twin Desktop session ended.
echo ======================================================================
pause
