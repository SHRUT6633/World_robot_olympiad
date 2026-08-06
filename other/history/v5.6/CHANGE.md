# v5.6 — Adaptive Noise Estimation

**Theme:** "Let the filter tune itself."

Fixed Q and R work well for a specific scenario. But the WRO course has varied conditions: carpet vs. tile, fast vs. slow segments, bright vs. dim lighting for the camera. A fixed Q works fine on carpet but underestimates process noise on tile (where wheels slip more). A fixed R works in good lighting but fails when the camera struggles in shadows.

The solution: adapt Q based on the filter's own innovation residuals. The innovation is `z - h(x)`, the difference between what the sensors measure and what the filter predicts. When the innovation is large, the filter is surprised — something is wrong. Either the motion model is inaccurate (high process noise) or the sensor is noisy (high measurement noise).

I implemented `adaptive_noise.py` where the filter tracks a sliding window of the last 50 innovation residuals. Every update step, it computes the empirical covariance of the residuals and adjusts Q proportionally:

```python
innovation = z - hx
window.append(innovation)
if len(window) >= 50:
    empirical_R = np.cov(np.array(window).T)
    Q_scale = np.trace(empirical_R) / np.trace(nominal_R)
    Q = Q_nominal * max(0.1, min(10.0, Q_scale))
```

On paper, this is elegant. The filter detects when it's being surprised and increases process noise accordingly, allowing faster correction. When things are calm, it reduces process noise and filters more smoothly.

In practice, the noise estimate oscillated wildly.

```
[ADAPTIVE] Q_scale jumped from 0.3 to 8.2 in single step
[ADAPTIVE] Covariance trace: 0.12 → 0.89 → 0.09 → 0.95
[ADAPTIVE] Filter behaving erratically — Q oscillating
```

The problem: raw innovation residuals are noisy. A single large measurement error (e.g., camera glitch) causes Q to spike, which makes the filter over-trust the next measurement, which causes another large correction, which makes Q spike again. Positive feedback loop.

I tried smoothing the innovations with a simple moving average of 10 samples. It helped a bit but the oscillations persisted.

I tried capping the Q adjustment to ±50% per step. The filter became sluggish again.

The fix: exponential moving average (EMA) with alpha=0.1 over the last 50 residuals. The EMA is effectively a low-pass filter on the innovation sequence. With alpha=0.1, each new residual has only a 10% weight in the running estimate. This smooths the Q adaptation to respond to systematic changes (e.g., moving from carpet to tile) while ignoring transient glitches.

```python
ema_innovation = alpha * innovation + (1 - alpha) * ema_innovation
ema_covariance = alpha * np.outer(innovation, innovation) + (1 - alpha) * ema_covariance
```

With EMA, Q changes take about 50 update steps (1 second at 50Hz) to reach 63% of a step change. That's fast enough to adapt to surface changes but slow enough to ignore sensor glitches.

The window size of 50 samples (approximately 1 second at 50Hz correction rate) was chosen empirically. Smaller windows (10-20) made the filter too twitchy. Larger windows (100+) made adaptation too slow.

The adaptive noise filter now performs well across surfaces. On carpet (low slip), Q settles around 0.8e-3. On tile (more slip), Q settles around 2.5e-3. The filter automatically adjusts, no manual tuning needed.

Key files:
- `adaptive_noise.py` — UKF with EMA-based adaptive noise estimation
- `surface_test.py` — Test script running on carpet vs tile
