# 3. Obstacle Management (4 pts)

## Strategy
1. **Detect pillars** by colour (RGB values from Rulebook 13.21-13.22)
2. **Pass red on RIGHT**, **green on LEFT** (Rule 9.19)
3. **Surprise Rule**: Change `pillar_logic` in config to REVERSED to swap sides

## Pillar Colour Specifications (from Rulebook)
| Object | Rule | RGB | HSV (CV2 range) |
|--------|------|-----|-----------------|
| Red pillar | 13.21 | (238, 39, 55) | H: 0-10 or 170-180, S: 100-255, V: 100-255 |
| Green pillar | 13.22 | (68, 214, 44) | H: 40-90, S: 50-255, V: 50-255 |
| Magenta parking | 13.27 | (255, 0, 255) | H: 140-170, S: 100-255, V: 50-255 |

## State Machine Flow
```
INIT → IDLE → START_SEARCH → FORWARD → CORNERING
        ↕                      ↕           ↕
   OBSTACLE_AVOID ←────── wall/pillar detected
        ↕
   REVERSE (if stuck)
        ↕
   LAP_FINISHED → PARK → SHUTDOWN
```

## Files
- `pi/perception/pillar_detector.py` — Colour-based pillar detection
- `pi/perception/lane_detection.py` — Lane boundary detection
- `pi/perception/wall_detection.py` — ToF-based wall detection
- `pi/mission/state_machine.py` — Finite state machine
- `pi/planning/global_planner.py` — Waypoint planning
