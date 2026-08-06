# v5.7 — Outlier Rejection

**Theme:** "Don't trust bad data."

Sensors fail. The camera sees a false positive. The ToF sensor reflects off a shiny surface. The magnetometer picks up a motor spike. Without outlier rejection, a single bad measurement can corrupt the filter state for seconds.

I implemented Mahalanobis distance-based outlier rejection. The idea: compute the squared Mahalanobis distance `d² = y.T @ S⁻¹ @ y` where `y` is the innovation and `S` is the innovation covariance. Under the Gaussian assumption, `d²` follows a chi-squared distribution with `dim_z` degrees of freedom. If `d²` exceeds the threshold for a given confidence level, reject the measurement.

For 2-DOF measurements (x, y position), the 95% confidence threshold is chi2(0.95, 2) = 5.991. For 3-DOF (x, y, heading), it's chi2(0.95, 3) = 7.815.

Initial threshold: I used `threshold = 3.0` (roughly 3-sigma for Gaussian). This is equivalent to about 89% confidence for 2-DOF.

Immediately, the filter started rejecting good measurements:

```
[OUTLIER] innov=(0.032, 0.041) d²=3.42 > 3.00 → REJECTED
[OUTLIER] Rejection rate: 30% — TOO HIGH
```

30% rejection rate on a well-tuned filter is catastrophic. The filter was starving for corrections. Examining the rejected innovations, they were all well within normal sensor noise. The Mahalanobis distance was inflated not by large innovations but by small covariance.

The problem: the innovation covariance `S = H @ P @ H.T + R` was underestimated because the filter's state covariance `P` had converged to a very small value (trace ~0.002). When `P` is tiny, `S ≈ R`, and the Mahalanobis distance for a perfectly normal innovation is `y.T @ R⁻¹ @ y`. If `R` is tuned aggressively (small), even normal noise produces large Mahalanobis distances.

I considered several thresholds:
- **3-sigma (3.0)**: Too strict, 30% rejection
- **4-sigma (4.0)**: Still 15% rejection — too many false positives
- **5-sigma (5.0)**: 5% rejection — acceptable for safety, but might miss real outliers
- **chi2 90% (4.605)**: 10% false rejection — marginal
- **chi2 95% (5.991)**: 5% false rejection — acceptable
- **chi2 99% (9.210)**: 1% false rejection — might let outliers through

I settled on chi2 with 95% confidence (5.991 for 2-DOF). This gives a 5% false rejection rate under ideal conditions — acceptable for a competition robot. The filter can tolerate missing 1 in 20 corrections; it has 50Hz of correction opportunities anyway.

I also added a gating mechanism: instead of binary accept/reject, I compute a weight `w = exp(-0.5 * d²)` and use it to scale the measurement noise `R`. Bad measurements get `R` inflated, reducing their influence rather than completely ignoring them. This is softer and prevents the filter from going "blind" if it rejects too many.

```python
if d2 > CHI2_THRESHOLD:
    weight = math.exp(-0.5 * (d2 - CHI2_THRESHOLD))
    ukf.R = R_nominal / weight  # inflate R for bad measurements
else:
    ukf.R = R_nominal
```

This soft gating reduced the effective rejection rate to about 2% (the weight is very small for real outliers) while keeping all measurements contributing to the estimate.

Testing with injected outliers (random 50cm jumps in position measurement):
- Without rejection: filter jumps 30cm, takes 3s to recover
- With binary rejection: filter ignores the spike, no disruption
- With soft gating: filter ignores the spike, no disruption, plus better behavior on borderline cases

Key files:
- `outlier_reject.py` — UKF with Mahalanobis distance outlier rejection
- `outlier_test.py` — Test with injected bad measurements
