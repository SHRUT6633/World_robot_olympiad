#!/bin/bash
echo "============================================"
echo " WRO 4WS - ESP32-S3 Flasher"
echo "============================================"
cd "$(dirname "$0")/.."
if [ ! -d "esp/build" ]; then
    echo "[*] Building ESP32 firmware..."
    cd esp
    idf.py build || exit 1
    cd ..
fi
echo "[*] Flashing ESP32-S3..."
idf.py -C esp flash && echo "[OK] Flash complete"
