# Results

What was measured, how, and what was not measured. The distinction between
**simulated** and **on the physical robot** is kept explicit throughout,
because most of the encouraging numbers are simulated and most of the
difficult ones are physical.

---

## 1. Camera calibration — physical ✅

| | |
|---|---|
| Images | 23 chessboard views |
| Model | equidistant fisheye |
| Reprojection RMS | **0.411 px** |
| Focal length | **973 px** |
| Field of view | **133.5° diagonal / 94.0° horizontal** |
| Distortion | (−0.134, 0.028, −0.045, 0.029) |

Cross-checked against a grid target at known distance.

The bundled lens was labelled "2.8 mm, D148°". Measured diagonal is 133.5° —
a 3.3% disagreement with the label, which is normal. What matters more is
that **the label implied a pinhole-equivalent f_px of 755, while the true
value is 973.** Trusting the label would have introduced a 29% depth bias.

### The model choice is not cosmetic

Running this lens through a pinhole model, with everything else correct:

| Incidence angle | Depth error |
|---|---|
| 44° | 2.1 cm |
| 62° | 10.4 cm |
| 72° | 37.7 cm |
| 80° | **158.7 cm** |

The reprojection residual at that last row is **1.90 px** — comfortably
inside any sane gate. The estimator is confidently, catastrophically wrong
and nothing flags it. Projection models cannot be validated by residual;
they have to be measured.

---

## 2. Trajectory estimation — simulated ✅

150 synthetic throws per row, median values. Detection noise is injected as
Gaussian error on the bounding-box centre.

| Drop height | Noise | Flight | Frames | First prediction | Time left to move | First error | Final error | Success |
|---|---|---|---|---|---|---|---|---|
| 1.5 m | 1.0 px | 0.73 s | 41 | 0.47 s | 0.26 s | 2.2 cm | 0.1 cm | 100% |
| 2.0 m | 1.0 px | 0.81 s | 46 | 0.53 s | 0.28 s | 2.9 cm | 0.2 cm | 100% |
| 2.5 m | 0.5 px | 0.88 s | 50 | 0.48 s | **0.40 s** | 4.3 cm | 0.1 cm | 100% |
| 2.5 m | 1.0 px | 0.88 s | 50 | 0.62 s | 0.27 s | 2.9 cm | 0.4 cm | 100% |
| 2.5 m | 2.0 px | 0.88 s | 50 | 0.78 s | 0.10 s | 2.2 cm | 1.5 cm | **30%** |
| 3.0 m | 1.0 px | 0.95 s | 53 | 0.65 s | 0.30 s | 3.6 cm | 1.0 cm | 99% |

### The conclusion that matters

**Detection noise, not the estimator, decides whether a catch happens.**

Compare the 2.5 m rows. Going from 0.5 px to 2.0 px of detection noise does
not degrade the final prediction much (0.1 cm → 1.5 cm, both well inside the
13 cm bin radius). What collapses is *when* the prediction becomes
trustworthy: 0.48 s → 0.78 s after release. Since flight is only 0.88 s,
the robot's time to move drops from 0.40 s to 0.10 s, and success goes from
100% to 30%.

So effort spent on sharper detections — shorter exposure, more gain, a
global shutter, larger pixels — converts directly into catch rate, while
effort spent on a better estimator does not. This is why the hardware choices
in the [BOM](bom.md) lean the way they do.

### ⚠️ Caveat

These runs used **f_px = 1739** (a 6 mm lens, 45°×35° FOV), which is not the
lens that was eventually fitted. At the real f_px of 973, the same pixel
noise corresponds to roughly **1.8× more angular error**. That is partly
offset — the wider field keeps the object in frame longer, yielding more
observations — so it is a trade rather than a straight loss. **This table
should be re-run at the measured optics.** It has not been.

---

## 3. Detection — physical ✅

| | |
|---|---|
| Model | YOLOv8n, single class `trash`, transfer-learned from COCO |
| Inference | Hailo-8L, 640×640 letterboxed |
| Rate | **60 fps sustained** |
| Confidence on real throws | 0.83–0.90, continuous through flight |

An early version re-created the Hailo network activation and inference
streams on every frame. Hoisting that into initialisation was worth a large
constant factor; before the fix, frame periods could exceed the 150 ms
control watchdog.

Rotation augmentation is deliberately **off** during training. This pipeline
consumes only the bounding-box centre, and rotating an image perturbs box
centres — it damages exactly the signal the estimator depends on. Horizontal
and vertical flips are used instead.

---

## 4. Multi-hypothesis tracking — simulated ✅

YOLO sometimes scores a ceiling fixture higher than the actual object. Taking
the most confident detection per frame means the real object is invisible on
those frames, and the contaminated track's residual diverges.

Instead every detection spawns a hypothesis, and the physics decides. A
stationary false positive cannot be fitted by any parabola, so it is rejected
on residual.

### Cycle timeout, measured

Before the fix, a cycle could only end once a trajectory had been committed —
so while the robot was staring at ceiling lights and committing nothing,
hypotheses accumulated indefinitely. Adding a pre-commitment timeout:

| Robot speed | False-positive frames committed, before | after | change |
|---|---|---|---|
| 0.25 m/s | 10.4 | 1.5 | **−86%** |
| 0.50 m/s | 27.4 | 3.8 | **−86%** |
| 0.80 m/s | 14.1 | 3.5 | **−75%** |

Counter to expectation: the reset re-enables a "first fit is exempt from the
depth-convergence check" allowance, which should *increase* false positives.
It does not, and by a wide margin. While the camera is moving, accumulated
observations of static points supply increasingly convincing parallax — so
cutting that accumulation short matters far more than the exemption costs.

---

## 5. Early start — simulated ✅

Measured from a real 60 fps log: **17 frames (0.267 s)** elapsed between
first detection and first motor command. Objects pass overhead 0.10–0.13 s
after first detection. The robot was departing after the object had already
gone by.

Bearing, however, is available immediately: an upward-facing camera maps
screen position to azimuth independently of range. So the robot can commit to
a direction long before it knows a distance.

| Gating rule | Frames to first command | False positives triggering it |
|---|---|---|
| Cumulative displacement ≥ 40 px | 6 | 0% |
| No gate at all | 3 | **high** |
| **Screen speed ≥ 350 px/s** | **3** | **0%** |

Screen *speed* separates real throws from stationary false positives far
better than accumulated displacement, and it is meaningful from the second
observation rather than the sixth:

| | Screen speed at 3 observations |
|---|---|
| Real throws | 406–773 px/s |
| Static false positive, robot stationary | 56–197 px/s |
| Static false positive, robot at 0.5 m/s | 120–302 px/s |

The 350 px/s threshold sits 2.7× above the noise floor (~127 px/s at 1.5 px
detection noise), so noise alone cannot trigger it.

One subtlety: the **direction of travel** is used, not the current bearing.
Early in flight the object is on the far side of the sky and its current
bearing points roughly opposite to where it will land. A real log shows the
vertical image coordinate sweeping 182 → 951 while the true landing point was
forward.

---

## 6. Control loop — physical ✅

| | |
|---|---|
| Pico loop | 20 ms (50 Hz), fixed period |
| Encoder | PIO quadrature, 1× decoding, 687.5 counts per wheel revolution (measured at the output shaft) |
| Link | USB CDC, both sides drain to the most recent frame |
| Watchdog | 150 ms, motors stop if commands stop arriving |

Four bugs found by inspection and fixed, each structural rather than
incidental:

1. **Odometry under-reporting.** Pose was integrated from Kalman-filtered
   velocity. With the placeholder noise parameters the filter's time constant
   was ≈1.4 s, against a 0.5 s catch manoeuvre — pose reflected **18%** of
   real motion (a 50 cm move reported as 8.9 cm). Since the Pico computes
   speed from `target − pose`, it simply never stopped accelerating. Pose is
   now integrated from raw velocity; the filter is used only for telemetry.

2. **Missing world→body rotation.** The target direction was passed to
   inverse kinematics without the `R(−θ)` rotation, valid only while θ ≈ 0 —
   but θ has no reset and drifts from encoder noise alone.

3. **PID integral not cleared on mode change**, allowing a stale integral to
   command full duty on the first tick after a switch.

4. **Serial backlog.** The receive path returned after one frame while the
   sender ran faster than the 50 Hz loop, accumulating unbounded latency.
   Both sides now drain to the newest complete frame.

None of these produce an error message. All of them produce a robot that
drives past its target.

---

## 7. Real throws — physical ⚠️

Two logged attempts, 2026-09-09:

| | Distance to target | Time remaining | Speed required | Robot capability |
|---|---|---|---|---|
| Throw 1 | 1.57 m | 0.41 s | **3.8 m/s** | ~1.2–1.4 m/s |
| Throw 2 | 0.49 m | 0.20 s | **2.5 m/s** | ~1.2–1.4 m/s |

Detection was clean in both cases — 60 fps continuous, confidence 0.83–0.90.
The predictions were produced on time. **Both failures were physically
impossible catches**, not perception failures.

This is the single most important result in this document, and it is a
negative one: **until throws are constrained to the reachable region, catch
rate measures the thrower, not the robot.** Any study of whether more
training data improves performance has to separate these trials out first, or
the improvement is invisible underneath them.

`run.log` records distance and time remaining per cycle, so
`required speed = distance ÷ time` can be computed automatically for every
trial and used as a filter.

---

## 8. Not measured

Honestly, this is the longer list.

| | Why it matters |
|---|---|
| **Catch success rate** | The headline number. Never measured systematically |
| **Maximum speed** | `WHEEL_MAX_SPEED_MPS` = 1.8 is a raised software clamp, not a measurement. Theory says 1.22 |
| **Acceleration and stopping distance** | The controller is tuned for full-speed approach (`DRIVE_AGGRESSION = 8`). Whether overshoot stays inside the 13 cm bin radius is unknown |
| **Odometry scale and slip** | Never compared against a tape measure. Motors can deliver ~6× the torque friction can absorb, so slip is expected |
| **Residual distribution on real throws** | `MAX_RESIDUAL_PX = 6.0` was chosen from simulation |
| **False positives while driving** | Simulation says a moving camera makes static points look parabolic. Never quantified on hardware |
| **Loop rate under load** | No fps logging exists. Whether the 150 ms watchdog is ever exceeded is unknown |
| **Camera tilt compensation** | Measured at 3.9°, worth ~4 cm of landing error. The estimator assumes a vertical optical axis and cannot correct it |

The tools for most of these exist — `pico_test.py --max` for stopping
distance, `teleop_test.py --speed-test` for top speed, `test_accuracy.py` for
prediction error against measured ground truth. They were written; they were
not run before the project ended.

---

## 9. If you are continuing this work

In priority order:

1. **Measure the wheel speed limit** and set it in all three config files.
   Everything downstream — reachability, the speed envelope, whether lateral
   motion tracks straight — depends on this number being real.
2. **Define a reachable throwing region** from measured acceleration and
   stopping distance, and constrain trials to it. Without this, §7 repeats.
3. **Find the root cause of the direction inversion** instead of
   compensating with `DRIVE_INVERT`.
4. **Add loop-rate logging.** One line every N frames. There is currently no
   way to tell whether the watchdog is being exceeded.
5. **Re-run §2 at the measured optics.** The headline estimator numbers are
   from a lens that was never fitted.

Design rationale for the constants involved is in
[`algorithm.md`](../algorithm.md) and [`physics.md`](physics.md); the full
list of known-unresolved issues is [`open-questions.md`](open-questions.md)
(Korean).
