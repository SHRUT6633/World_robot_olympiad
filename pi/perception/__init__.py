# =============================================================================
# WRO 2026 — 4WS AWD Autonomous Robot
# File: pi/perception/__init__.py
# Rev:  v9.9  |  Status: RELEASED
# -----------------------------------------------------------------------------
# Perception package initialiser
# =============================================================================

# The perception package provides all visual and rangefinder-based sensing
# modules for the WRO 2026 4WS AWD autonomous robot.
#
# Sub-modules:
#   pillar_detector     — HSV-based coloured pillar detection (red/green/magenta)
#   pillar_tracker      — Pillar pass-side counting and verification logic
#   parking_detector    — 7-state state machine for autonomous parking
#   lane_detection      — Canny + Hough transform for lane-line detection
#   wall_detection      — ToF-based wall proximity detection
#   corner_detection    — Shi-Tomasi corner/feature detection
#   object_detection    — Generic contour-based object detection
#   depth_estimation    — Stereo disparity to metric depth conversion
#   landmark_detection  — ORB template-based landmark recognition
#   free_space          — Sobel-gradient free-space / obstacle-cost map
#   road_edge           — Vertical-ROI road-edge profiling via Canny
#   feature_matching    — ORB extraction + BFMatcher (cross-check)
#   visual_odometry     — Monocular VO via essential matrix decomposition
#   optical_flow        — Lucas-Kanade sparse optical flow
#
# Dependencies: OpenCV, NumPy, system.logger
