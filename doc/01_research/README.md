<!--
=============================================================================
WRO 2026 — 4WS AWD Autonomous Robot
File: docs/01_research/README.md
Rev:  v9.9  |  Status: RELEASED
=============================================================================
-->

# Research & Design Analysis

> **All documents in this folder are theoretical research.** No physical prototypes were built for 2WS, FWD, or RWD configurations. Every alternative was analysed mathematically, simulated in Python, and compared on paper before the final 4WS+AWD architecture was chosen and built directly.

## Documents

| Document | Content |
|----------|---------|
| **`turning_radius_analysis.md`** | 2WS vs 4WS steering geometry — Ackermann equations, push-rod kinematics, servo torque, tire slip, Stanley control. Concludes 2WS geometrically insufficient. |
| **`drivetrain_tradeoffs.md`** | FWD vs RWD vs AWD — torque calcs, gear ratios, binding analysis, power budget. Concludes single-motor AWD optimal. |
| **`design_decision_matrix.md`** | 14 weighted decisions covering MCU, comm, power, frame, sensors, control, estimation. Scores all alternatives. |
| **`engineering_journey.md`** | Full narrative from theoretical research (pre-v1.0) through code evolution (v1.0–v4.9). 40 chapters with terminal errors, equations, and code mappings. |

## How to Read These

1. Start with `engineering_journey.md` — the full story from first concept through v4.9
2. Deep-dive into `turning_radius_analysis.md` for the geometry that forced 4WS
3. Read `drivetrain_tradeoffs.md` for the mechanical reasoning behind AWD
4. Check `design_decision_matrix.md` to see why every choice was made

## Key Conclusion

The research proves that **only 4WS+AWD** can meet all WRO 2026 track requirements (turning radius, obstacle clearance, parking precision) with sufficient safety margin. All other configurations are theoretically inferior. The code in `pi/` and `esp/` implements this single chosen architecture directly — no intermediate prototypes were needed.
