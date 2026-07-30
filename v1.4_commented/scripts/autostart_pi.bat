@echo off
title WRO 4WS - AUTOSTART
:: ===================================================
:: AUTOSTART - Place in Windows Startup folder or
:: Raspberry Pi: put in /etc/rc.local or .bashrc
:: ===================================================
cd /d "%~dp0.."

echo [WRO_BOOT] Starting WRO 4WS System...
echo [WRO_BOOT] Timestamp: %DATE% %TIME%

:: Check Python available
where python >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python not found!
    pause
    exit /b 1
)

:: Install/update deps silently
python -m pip install -r requirements.txt -q >nul 2>&1

:: Run the boot sequence (self-test + LED + switch)
echo [WRO_BOOT] Running boot sequence...
python run_all.py --mode boot

:: If boot passed, race mode is launched by boot.py internally
echo [WRO_BOOT] System shutdown at %DATE% %TIME%
pause
