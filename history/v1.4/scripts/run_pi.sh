#!/bin/bash
echo "============================================"
echo " WRO 4WS - Pi Controller Launcher"
echo "============================================"
cd "$(dirname "$0")/.."
python3 run_all.py --mode pi --config config/pi_config.yaml --log-level INFO
