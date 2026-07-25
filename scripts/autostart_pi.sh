#!/bin/bash
# ===================================================
# AUTOSTART for Raspberry Pi
# Place in: /etc/rc.local or ~/.bashrc
# ===================================================
echo "[WRO_BOOT] Starting WRO 4WS System..."
cd "$(dirname "$0")/.."

# Wait for network/interfaces
sleep 2

# Install/update deps
pip3 install -r requirements.txt -q 2>/dev/null

# Run boot sequence
echo "[WRO_BOOT] Running boot sequence..."
python3 run_all.py --mode boot

echo "[WRO_BOOT] System shutdown."
