# v5.3 — Extended Kalman Filter for 2D Localization

**Theme:** "Time for actual math."

Complementary filters are fine for attitude, but they don't give us position. We need a real state estimator — one that fuses encoder odometry, IMU heading, and any available position measurements into a coherent pose estimate. Enter the Extended Kalman Filter.

The EKF state is 3-dimensional: `[x, y, heading]`. The motion model is differential drive: from encoder ticks, compute forward velocity and yaw rate. The prediction step propagates the state and covariance using a linearized model of the robot kinematics.

The measurement model varies: when a position measurement arrives (from camera, ToF, or landmark detection), the EKF corrects the state estimate using the innovation (measurement residual) weighted by the Kalman gain.

I coded `ekf_localization.py` with careful attention to the Jacobians. The state transition Jacobian `F` is a 3x3 matrix derived from the motion model. The measurement Jacobian `H` maps state space to measurement space. For a direct (x,y) measurement, it's just a selection matrix.

The first test was a gentle S-curve driving pattern. The EKF tracked beautifully — smooth path, clean covariance estimates, error under 3cm throughout.

Then I commanded a sharp 180° turn-in-place.

```
[EKF] Sharp turn detected — yaw rate: 112°/s
[EKF] Innovation: dx=0.023 dy=0.094
[EKF] Post-update position error: 11.3cm ← FAIL
```

The EKF failed during sharp turns because the motion model is nonlinear, and the first-order Taylor approximation (linearization) breaks down when the angular velocity is high. During a 180° turn, the heading changes rapidly, and the linearized state transition matrix `F` doesn't capture the true kinematics accurately.

Mathematically: the true state update is `x' = x + v*cos(theta)*dt`. The EKF linearizes this as `F = d(x')/dx = [[1, 0, -v*sin(theta)*dt], [0, 1, v*cos(theta)*dt], [0, 0, 1]]`. During sharp turns, `v*sin(theta)*dt` and `v*cos(theta)*dt` are large, and the linear approximation error grows proportional to `v * dtheta^2 / 2` — the second-order term we're ignoring.

The 10cm+ error during sharp turns is unacceptable. A WRO robot will perform sharp turns at waypoints and obstacles.

I considered several fixes:
1. **Higher order EKF** — Include second-order terms in the Taylor expansion. Complex and not guaranteed to converge.
2. **Sigma-point methods** — Instead of linearizing, propagate a set of sample points through the true nonlinear function. This is the Unscented Kalman Filter (UKF). It captures the nonlinearity accurately without Jacobians.
3. **Multiple models** — Use an Interacting Multiple Model (IMM) filter with separate models for straight driving and turning. Too complex for the gain.
4. **Reduce turn rate** — Command slower turns. But this hurts competition performance.

Option 2 (UKF) is the right architectural choice. It avoids Jacobian linearization entirely, handles nonlinearities naturally, and has essentially the same computational cost for a 3-state system. The next version (v5.4) will replace the EKF with a UKF.

For now, the EKF works well for gentle driving (<45°/s yaw rate). We'll keep it as a fallback and learn from its failure modes.

Key files:
- `ekf_localization.py` — The EKF implementation
- `ekf_test.py` — Test with sharp turn scenario

Error logs:
```
[EKF PREDICT] dt=0.020s, v=0.32m/s, w=1.96rad/s
[EKF PREDICT] Covariance trace: 0.0004 -> 0.0009
[EKF CORRECT] z=(1.02, 0.15), h=(1.00, 0.05) innov=(0.02, 0.10)
[EKF WARNING] Innovation Mahalanobis distance: 4.82 (threshold: 3.0)
[EKF WARNING] Large innovation — possible linearization error
```
