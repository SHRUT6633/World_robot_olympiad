# v5.4 — Unscented Kalman Filter Implementation

**Theme:** "Sigma points beat Jacobians."

The EKF failed during sharp turns because linearization is only valid near the operating point. When heading changes rapidly, the first-order Taylor approximation misses the curvature of the true dynamics. The UKF fixes this by propagating sigma points through the true nonlinear function — no Jacobians needed.

The UKF uses the unscented transform: generate 2n+1 sigma points around the current state estimate, propagate each through the motion model, then reconstruct the predicted mean and covariance from the propagated points. The result is accurate to the third order for Gaussian inputs, compared to the EKF's first-order accuracy.

I implemented `ukf_localization.py` with a 6-state vector: `[x, y, heading, speed, acceleration, yaw_rate]`. The extra states (speed, acceleration, yaw_rate) let the filter estimate dynamic quantities directly, improving prediction accuracy during acceleration and turning.

I used the `filterpy` library for the UKF core, which provides `UnscentedKalmanFilter` and sigma point generators.

First compile attempt:
```
Traceback (most recent call last):
  File "ukf_localization.py", line 8, in <module>
    from filterpy.kalman import MerkedScaledSigmaPoints
ImportError: cannot import name 'MerkedScaledSigmaPoints' from 'filterpy.kalman'
```

Yes, I typed `MerkedScaledSigmaPoints` instead of `MerweScaledSigmaPoints`. The correct spelling is `Merwe` (named after Rudolph van der Merwe, who developed the scaled unscented transform). My fingers combined "Merkel" and "Merwe" into "Merked". Classic typo that cost 5 minutes of staring at the screen wondering why filterpy suddenly broke.

Fixed:
```python
# Wrong:
from filterpy.kalman import MerkedScaledSigmaPoints
# Correct:
from filterpy.kalman import MerweScaledSigmaPoints
```

The UKF parameters: `alpha=0.1` (spread of sigma points), `beta=2.0` (optimal for Gaussian distributions), `kappa=0` (scaling factor). These are the standard Van der Merwe values.

After fixing the import, the UKF ran beautifully. I compared it side-by-side with the EKF on the sharp turn test:

```
[COMPARISON] Straight segment (0-2s):
  EKF error: 2.1cm  UKF error: 1.8cm  (comparable)
[COMPARISON] Sharp turn (2-3s):
  EKF error: 11.3cm  UKF error: 2.7cm  (4x better!)
[COMPARISON] Post-turn (3-5s):
  EKF error: 4.2cm  UKF error: 2.1cm  (EKF still recovering)
```

The UKF handles the sharp turn nonlinearity naturally. The sigma points spread out, capture the curved trajectory, and re-converge correctly after the turn. The EKF, by contrast, "falls off" the true trajectory during the sharp turn and takes several seconds to re-converge (because its covariance was underestimated by the linearization, making it trust its incorrect state).

There's a cost: the UKF is about 2x slower than the EKF (propagating 13 sigma points vs. a single Jacobian multiply). But for a 6-state system running at 100Hz, that's still well under 1ms of CPU time per update. Worth it.

One thing I noticed: the UKF's performance is sensitive to the choice of sigma point parameters and noise matrices. If alpha is too large (>0.5), the sigma points spread too far and the filter becomes noisy. If too small (<0.01), the sigma points cluster too close to the mean and the UKF behaves like a poor EKF. alpha=0.1 is a good default for wheeled robots.

Next step: tune the noise parameters Q and R for this specific robot platform. v5.5.

Key files:
- `ukf_localization.py` — The UKF implementation
- `ukf_vs_ekf.py` — Comparative test showing the difference
