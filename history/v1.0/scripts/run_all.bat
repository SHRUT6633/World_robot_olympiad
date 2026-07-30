@echo off
title WRO 4WS - Full System
cd /d "%~dp0.."
echo ============================================
echo  WRO 4WS - FULL SYSTEM LAUNCH
echo  Raspberry Pi 4B + ESP32-S3
echo ============================================
echo.
echo [1/2] Starting Pi controller...
start "WRO Pi Controller" cmd /c python run_all.py --mode pi --log-level INFO
echo [2/2] Done. Pi process launched.
echo.
echo Press any key to stop all processes...
pause >nul
echo [*] Stopping...
taskkill /f /im python.exe 2>nul
echo [OK] Stopped.
