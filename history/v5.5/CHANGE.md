# v5.5 — UKF Tuning

**Theme:** "Q and R: the knobs of destiny."

The UKF works. But how well depends entirely on the noise covariance matrices Q (process noise) and R (measurement noise). Get them wrong and the filter either ignores its sensors (too much Q) or ignores its motion model (too much R). This version is all about empirically finding the right values.

Q represents how much we trust the motion model. High Q means "the model is unreliable, listen to sensors." Low Q means "the model is accurate, ignore sensor noise." R represents how much we trust each measurement. High R means "this sensor is noisy, trust the model." Low R means "this sensor is precise, correct aggressively."

I started with Q=1e-2 (diagonal) and R=1e-2 (diagonal). The result was a jittery, nervous filter that jumped at every measurement:

```
[UKF] dt=0.02s, Q=1e-2, R=1e-2
[UKF] Position jump: 3.2cm from single measurement correction
[UKF] Covariance trace dropped from 0.15 to 0.02 in one step
```

The filter was over-correcting. Every measurement pulled the state estimate significantly, even when the measurement was noisy. This is because Q was too high — the filter thought the motion model was unreliable, so it trusted each new measurement too much.

I tried Q=1e-6, R=1e-1. The opposite problem: the filter barely moved. It stubbornly held onto the motion model prediction even when measurements consistently disagreed.

```
[UKF] Sensor bias detected: x error = 8cm (consistent for 2 seconds)
[UKF] Filter not correcting — Q too low, R too high
```

The filter was essentially dead reckoning with a tiny nudge from measurements. It would never converge to the true position.

I tried dozens of combinations. Here's a sample of the tuning runs:

| Q_diag | R_diag | Result |
|--------|--------|--------|
| 1e-2   | 1e-2   | Jittery, 4cm RMS error |
| 1e-2   | 1e-1   | Still jittery, 3.5cm RMS |
| 1e-3   | 1e-2   | Good response, 2.1cm RMS |
| 1e-3   | 1e-1   | Smooth, 1.8cm RMS ← best |
| 1e-4   | 1e-1   | Sluggish, 2.8cm RMS |
| 1e-4   | 1e-2   | Moderate, 2.2cm RMS |

The winner: Q=1e-3, R=1e-1. This gives the filter a reasonable trust in the motion model (Q=1e-3 means ~3cm uncertainty per step) while being skeptical of measurements (R=1e-1 means ~30cm measurement uncertainty). The skepticism about measurements is appropriate because our position measurements come from camera detections which have significant latency and quantization error.

I also learned that the ratio Q/R matters more than the absolute values. A Q/R ratio of 0.01 (1e-3 / 1e-1) produces smooth, responsive tracking. Ratios above 0.1 produce jitter. Ratios below 0.001 produce sluggish response.

The absolute values matter for convergence speed. With Q=1e-3, R=1e-1, the filter converges from an initial position error of 50cm to <5cm in about 2 seconds. That's fast enough for our use case.

I saved the tuning parameters to a JSON config file so they can be adjusted without code changes. The tune_ukf.py script is now the reference for anyone adjusting filter behavior.

Key insight: the correct Q and R depend on the actual sensor noise characteristics. I measured the camera position noise empirically: 100 static measurements had a standard deviation of 8.2cm. So R=0.1 (10cm std) is about right. The motion model noise depends on encoder quality and floor surface. Q=1e-3 gives 3cm std per step, which matches our encoder testing from v5.0.

Key files:
- `tune_ukf.py` — Parameter sweep and analysis
- `ukf_params.json` — Saved optimal parameters

Final parameters:
```json
{
  "Q": [0.001, 0.001, 0.001, 0.01, 0.1, 0.01],
  "R": [0.1, 0.1, 0.05],
  "alpha": 0.1,
  "beta": 2.0,
  "kappa": 0
}
```
