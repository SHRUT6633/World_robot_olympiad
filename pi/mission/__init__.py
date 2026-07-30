# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/mission/__init__.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# PURPOSE: Package initializer
# =============================================================================

# This package contains all high-level mission logic for the WRO 2026
# autonomous driving competition.  Each module owns a distinct responsibility:
#
#   state_machine      — Finite-state machine governing the robot's lifecycle
#                        (INIT → IDLE → FORWARD → ... → PARK_VERIFY → SHUTDOWN).
#   lap_counter        — Tracks how many laps have been completed and when
#                        the robot should transition from "racing" to "parking".
#   start_detection    — Template-matching module that recognises the start
#                        zone to trigger end-of-race behaviour.
#   obstacle_strategy  — Reactive decision-maker that picks left/right/forward
#                        when an obstacle blocks the path.
#   race_strategy      — Lap- and obstacle-aware speed scaler (exploration,
#                        normal, racing, cautious modes).
#   checkpoint          — Ordered waypoint manager that feeds (x, y) targets
#                        to the low-level path tracker.
#   direction           — Heading-rate classifier that detects whether the
#                        robot is moving straight, turning left, or turning right.
#   reverse_logic       — Timed reverse manoeuvre helper for when the robot
#                        is boxed in and cannot steer forward.
#
# Modules that don't exist yet (to be created):
#   parking_strategy    — Sub-strategy for parking bay detection and entry.
#   mission_planner     — Top-level coordinator that owns the state machine,
#                        lap counter, checkpoint manager, and strategy modules,
#                        and calls update() on each every tick.
