@echo off
title AUV Digital Twin - Desktop Simulator Launcher (ROS2 + Gazebo)
echo ======================================================================
echo    Starting AUV Desktop Underwater Simulator (ROS 2 Humble + Gazebo)
echo ======================================================================
echo.
echo Checking Docker status...
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

echo Starting ROS2 + Gazebo + Rosbridge container with GPU acceleration...
cd /d "%~dp0"
docker compose up -d

echo.
echo ======================================================================
echo [SUCCESS] Simulator is running!
echo.
echo   - ROS2 Bridge WebSocket: ws://localhost:9090
echo   - Digital Twin UI:       Native Desktop Application Window
echo.
echo Launching Native Desktop Application Window...
cd ..
npx electron .
echo ======================================================================
echo Stopping simulator backend...
cd simulator
docker compose down
