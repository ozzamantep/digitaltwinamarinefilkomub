@echo off
title Launch Gazebo 3D GUI in WSL2 - AUV Digital Twin
cd /d "%~dp0"

echo ======================================================================
echo    Launching Gazebo 3D GUI on Windows via WSL2 (WSLg)
echo ======================================================================
echo.
echo [1/2] Opening WSL2 Ubuntu and running Gazebo 3D Simulation...
echo       (Jendela Gazebo 3D native akan muncul di Windows!)
echo.

wsl -d Ubuntu-22.04 -e bash -c "cd /mnt/c/Users/ACER/Downloads/digitaltwin/simulator && chmod +x setup_wsl_gazebo.sh && ./setup_wsl_gazebo.sh"

echo.
echo ======================================================================
echo Gazebo session finished.
pause
