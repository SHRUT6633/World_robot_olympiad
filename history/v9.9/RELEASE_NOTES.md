# WRO 2026 — Release Candidate v9.9

## Features Summary
1. **4WS with 3 steering modes:** SAME_PHASE, OPPOSITE_PHASE, CRAB_WALK
2. **AWD single-motor drivetrain:** Compliant with Rules 11.3, 11.5
3. **6 sensors:** Camera (lane, pillar), ToF (wall, obstacle), IMU (pose), magnetometer (heading)
4. **UKF sensor fusion:** 6-DoF pose estimation with adaptive noise
5. **Colour pillar detection:** Exact Rulebook RGB values (red, green, magenta)
6. **Parking verification:** 30s stationary + parallel <= 2cm
7. **3 steering modes + Surprise Rule adaptation via config**
8. **Async scheduler:** 7 tasks at configurable rates
9. **Health monitor:** Heartbeat-based dead component detection
10. **Error catalog:** All 12 failure modes documented
11. **CI pipeline:** flake8, pylint, ESP-IDF build, YAML validation
12. **Competition docs:** 3 Appendix C docs with code references

## Known Issues (won't fix for competition)
1. No battery voltage monitoring (adds hardware complexity)
2. Camera frame drop at 60 FPS with full perception pipeline (use 30 FPS)
3. UART protocol lacks ACK/NACK (CRC error = silent drop)
4. No EEPROM parameter storage (config must be on SD card)
5. Single I2C bus (all sensors share; one failure takes down all)

## Configuration Checklist (competition morning)
- [ ] Battery 1 (Pi/ESP): 5V rail OK
- [ ] Battery 2 (Motor/Servo): 7.4V LiPo >= 7.0V
- [ ] Camera: /dev/video0 exists, lens clean
- [ ] ToF sensors: I2C addresses correct (0x30, 0x31, 0x32)
- [ ] ESP32 flashed with latest firmware
- [ ] surprise_rules.yaml: check pillar_logic, steering_mode
- [ ] pi_config.yaml: check width, height, fps
- [ ] UART: /dev/serial0 exists, 115200 baud
- [ ] Self-test: run `python pi/boot.py`

## Quick Debug (30-second fixes)
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot doesn't move | ESP32 not flashed | Check UART LED on ESP |
| No pillar detection | Wrong colour thresholds | Adjust in config |
| Robot oscillates | Steering mode wrong | Switch to SAME_PHASE |
| Camera black | Wrong device index | Check /dev/video0 |

## Scoring Estimate
- Documentation: 30/30 (all docs complete)
- On-track Open: 28/30
- On-track Obstacle: 58/62
- Total: 116/122

Good luck at WRO 2026!
