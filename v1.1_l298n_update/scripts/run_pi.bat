@echo off
title WRO 4WS - Raspberry Pi Controller
cd /d "%~dp0.."
echo ============================================
echo  WRO 4WS - Pi Controller Launcher
echo ============================================
echo.
python run_all.py --mode pi --config config\pi_config.yaml --log-level INFO
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Process exited with code %errorlevel%
    pause
)
