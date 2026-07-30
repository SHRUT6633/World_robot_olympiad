@echo off
title WRO 4WS - ESP32 Flash
cd /d "%~dp0.."
echo ============================================
echo  WRO 4WS - ESP32-S3 Flasher
echo ============================================
echo.
if not exist "esp\build" (
    echo [*] Building ESP32 firmware...
    cd esp
    idf.py build
    if %errorlevel% neq 0 (
        echo [ERROR] Build failed
        pause
        exit /b 1
    )
    cd ..
)
echo [*] Flashing ESP32-S3...
idf.py -C esp flash
if %errorlevel% equ 0 (
    echo [OK] Flash complete
) else (
    echo [ERROR] Flash failed
)
pause
