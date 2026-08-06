# 3. Obstacle Management (4 pts)

## Strategy
1. **Detect pillars** by colour using exact RGB values from Rulebook 13.21-13.22
2. **Pass red on RIGHT**, green on LEFT (Rule 9.19)
3. **Surprise Rule**: Change `pillar_logic` in config to REVERSED to swap sides

## Pillar Colour Specifications
| Object | Rule | RGB | HSV Range | Code Location |
|--------|------|-----|-----------|--------------|
| Red pillar | 13.21 | (238, 39, 55) | H: 0-10 or 170-180, S: 100-255, V: 100-255 | `pi/perception/pillar_detector.py:9-12` |
| Green pillar | 13.22 | (68, 214, 44) | H: 40-90, S: 50-255, V: 50-255 | `pi/perception/pillar_detector.py:13-14` |
| Magenta parking | 13.27 | (255, 0, 255) | H: 140-170, S: 100-255, V: 50-255 | `pi/perception/pillar_detector.py:15-16` |

## State Machine Flow
- **File/line:** `pi/mission/state_machine.py:15` — StateMachine class with states:
  `INIT -> IDLE -> FORWARD -> CORNERING -> OBSTACLE_AVOID -> PARK -> SHUTDOWN`
- **File/line:** `pi/mission/state_machine.py:60-80` — Transitions are event-driven (pillar detected, lap complete, etc.)

## Pillar Passing Logic
- **File/line:** `pi/perception/pillar_tracker.py:10` — Tracks passed pillars
- **File/line:** `pi/perception/pillar_tracker.py:35` — Validates pass direction against pillar_logic
- **File/line:** `config/surprise_rules.yaml:27` — `pillar_logic: "NORMAL"` means GREEN=pass LEFT, RED=pass RIGHT

## Parking Verification
- **File/line:** `pi/perception/parking_detector.py:15` — Detects magenta parking markers
- **File/line:** `pi/perception/parking_detector.py:50` — Verifies parallel alignment <= 2cm tolerance
- **File/line:** `pi/mission/state_machine.py:95` — PARK state: stationary for 30s for full 15 pts
