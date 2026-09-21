# Results

## The short version

**The robot catches about half of what is thrown into the region it can
physically reach, and nothing outside it.**

| | |
|---|---|
| Catch rate, inside the reachable envelope | **~50%** (n = 10) |
| Catch rate, outside it | **0%** |
| Reachable radius in the available time | **~40 cm** |

The bottleneck is the **drive**, not the perception or the estimator. We
tested five candidate causes and eliminated four of them. The conclusion we
reached is that **the motors chosen were unsuitable for this task**, and that
further work on the detector — more training data, a better model — would not
have changed the outcome.

That last point is worth stating plainly, because it was the question the
project set out to answer. **More data would not have helped.** The robot was
not failing to see the object; it was failing to arrive.

---

## How the bottleneck was found

Five candidates, each tested independently:

| # | Candidate | Verdict |
|---|---|---|
| 1 | Trajectory estimation code | ❌ Not the cause |
| 2 | YOLO detection | ❌ Not the cause |
| 3 | Pi ↔ Pico communication | ❌ Not the cause |
| 4 | **Motor performance** | ✅ **This is it** |
| 5 | Direction-dependent failures | ❌ Explained by #4 |

---

## 1. Trajectory estimation — not the cause

Tested against synthetic parabolas with known ground truth, sweeping
detection noise against observation count.

**With zero noise, every configuration from 5 to 20 observations fitted
successfully with 0.0000 cm landing error.** The mathematics and the
implementation are correct.

With noise, the picture changes sharply — and the sensitivity is to
**observation count** more than to noise level:

| Noise | n=10 | n=20 | n=25 | n=30 | n=35 |
|---|---|---|---|---|---|
| 0.5 px | 87.4 cm | 8.3 cm | 3.4 cm | 2.6 cm | **1.4 cm** |
| 1.0 px | 156.2 cm | 33.4 cm | 12.6 cm | 7.7 cm | **3.8 cm** |
| 1.5 px | 176.4 cm | 60.4 cm | 26.2 cm | 17.7 cm | **7.2 cm** |
| 2.0 px | 194.8 cm | 86.3 cm | 43.9 cm | 32.2 cm | **16.4 cm** |

(median landing error; at 60 fps, n=30 is a 0.50 s observation span)

Fit success rate follows the same shape — at 1.0 px noise it climbs from 67%
at n=10 to 99% at n=30.

**The real system observes for 0.4–0.5 s, i.e. n ≈ 25–30.** That lands in the
region where median error is a few centimetres but the 90th percentile is
still tens of centimetres — the estimator is usable there, but it is not
comfortable. Every additional frame of observation is worth a lot.

**Conclusion:** the estimator works. Its accuracy is governed by how many
good observations it gets, which makes detection quality and observation
window the levers — not the solver.

---

## 2. YOLO detection — not the cause

Three recorded throws were checked frame by frame offline. The detector found
the object correctly throughout, and bounding-box centres were produced at
**60 per second**.

**One real limitation:** when the object visually overlaps a ceiling light,
detection fails for those frames. Since this tends to happen early in flight
— the object is high, near the lights — it delays the start of the
observation window, which per §1 is exactly the quantity that matters.

**Conclusion:** detection is not what is losing catches. It is good enough
that improving it further has no effect while the drive remains the limit.

---

## 3. Pi ↔ Pico communication — not the cause

Both directions are non-blocking and stateless: the Pico streams odometry
continuously rather than replying to requests, and the Pi sends a target
whenever a fit succeeds. Each side reads to the newest frame and discards the
rest.

| Direction | Measured | Note |
|---|---|---|
| Odometry, Pico → Pi | **50 Hz** | Equal to the Pico's 50 Hz control loop — i.e. no frames lost |
| Commands, Pi → Pico | **20–44 Hz** | Event-driven, not periodic: only sent when a fit succeeds |

**Conclusion:** the link runs at the design ceiling. Not a bottleneck.

---

## 4. Motor performance — this is the bottleneck

### What was measured

| | |
|---|---|
| Speed over 1 m | **0.8 m/s** |
| Speed over 2 m | **0.9 m/s** |
| Time remaining after detection and prediction | **0.56 s mean** (range 0.5–0.6 s) |
| Distance coverable in that window | **~40 cm** |

That 40 cm is the whole story. An object landing further than ~40 cm from the
robot's starting position cannot be caught, no matter how good the
perception is.

### Why the window is so short

The time available is the flight time minus the perception and prediction
delay:

```
T = flight time − (detection + fitting delay)
  ≈ 0.6 s − 0.15 s ≈ 0.45 s          (for a 2 m throw)
```

And reachable distance grows as T² in this regime, because the robot never
reaches top speed — it is still accelerating when the object lands:

| T | Distance passed (cm) | Distance with a full stop (cm) |
|---|---|---|
| 0.30 s | 8–9 | 5 |
| **0.45 s** | **18–21** | 10–12 |
| **0.60 s** | **32–38** | 18–21 |
| 0.80 s | 55–65 | 33–38 |
| 1.00 s | 80–93 | 51–59 |
| 1.50 s | 141–164 | 110–128 |

*Passed* means the bin was at that point at time T — which is all that is
required, since the object only needs the bin to be there at the moment it
lands. *With a full stop* additionally requires decelerating to rest.

**Going from T = 0.45 s to T = 0.60 s nearly doubles the reachable distance**
(18–21 cm → 32–38 cm). In the T² regime, shaving latency is worth far more
than it intuitively seems.

### Why acceleration, not top speed, is the limit

Per wheel, the motors can deliver about **35 N**. The floor can only
transmit about **4.9 N** (μ ≈ 0.5). Acceleration is therefore limited by
traction by a factor of seven, and a faster or more powerful motor would
change nothing:

| Direction | Max acceleration | Time to reach top speed |
|---|---|---|
| Hexagon edge (worst) | 1.77 m/s² | ~0.7 s |
| Hexagon vertex (best) | 2.09 m/s² | ~0.7 s |

Since the entire catch window is ~0.5 s and top speed needs ~0.7 s, **the
robot never reaches top speed during a catch.** Specifying a faster motor
would not have helped. A lighter robot, better tyres, or lower latency would.

Two mechanical effects also work against us:

- **Wheel slip on launch.** Commanding duty too abruptly exceeds traction,
  the wheels spin, and the encoders over-report distance travelled.
- **Dragged idle wheel.** A wheel commanded to zero duty coasts and is
  dragged by roller friction. It can be held with a short brake or a
  zero-target PID; the current firmware does neither.

---

## 5. Direction-dependent failures — explained by #4

Throws arriving from certain directions failed more often. This turned out
not to be a perception or estimation effect.

A three-wheel omni platform has a **hexagonal** velocity envelope. The
maximum speed in a given direction is set by whichever wheel has to turn
fastest:

| Direction | M1 | M2 | M3 | Max speed |
|---|---|---|---|---|
| 0° (toward M1) — vertex | 0 | −100% | +100% | **1.41 m/s** |
| 15° | −27% | −73% | +100% | 1.26 m/s |
| 30° — edge midpoint | −50% | −50% | +100% | **1.22 m/s** |
| 45° | −73% | −27% | +100% | 1.26 m/s |
| 60° (between M1 and M3) — vertex | −100% | 0 | +100% | **1.41 m/s** |

At a vertex one wheel is idle and the other two share the load equally; at an
edge midpoint one wheel does twice the work of the other two and saturates
first. The spread is about **15%**, and it propagates into reachable
distance.

**Conclusion:** the directional weakness is a property of the drive geometry,
not a bug in detection or fitting. Reach is simply smaller along the hexagon
edges.

> ⚠️ A related and still-unresolved problem: if the configured wheel speed
> limit is set **above** what the motors actually deliver, an edge-direction
> command saturates one wheel while the other two track correctly. The ratio
> between wheels breaks, and the robot curves instead of translating. See
> [Known limitations](../../README.md#known-limitations).

---

## What this means

1. **The motors were the wrong choice for this task.** Not because they are
   slow — because the catch window is shorter than the time needed to reach
   any useful speed, and acceleration is traction-limited anyway.

2. **Improving the detector would not have improved catch rate.** The project
   began with the question of whether more training data would help. The
   answer, for this robot, is no: perception was already delivering correct
   detections at 60 fps, and the failures were downstream of it. Collecting
   more data would have consumed effort for zero measurable gain.

3. **Latency is worth more than speed.** Reachable distance goes as T², so
   0.15 s of perception delay costs roughly half the reachable area. Effort
   spent shortening the pipeline would have paid better than effort spent on
   the motors.

4. **The estimator is sound and is not the constraint.** Zero-noise synthetic
   throws fit exactly. Real-world accuracy is set by the number of clean
   observations available, which is bounded by flight time and by how early
   detection begins.

If this project were continued, the order would be: reduce latency → reduce
robot mass → improve traction → and only then reconsider motors.

---

## Camera calibration — verified

| | |
|---|---|
| Images | 23 chessboard views |
| Model | equidistant fisheye |
| Reprojection RMS | **0.411 px** |
| Focal length | **973 px** |
| Field of view | **133.5° diagonal / 94.0° horizontal** |

The bundled lens was labelled "2.8 mm, D148°". Measured diagonal is 133.5°.
More importantly, the label implied a pinhole-equivalent f_px of 755 while
the true value is 973 — trusting the label would have introduced a 29% depth
bias.

**The projection model cannot be validated by reprojection residual.** With
this lens fed through a pinhole model, an incidence angle of 80° produces a
**158.7 cm** depth error while the residual stays at **1.90 px** — inside any
reasonable gate. The estimator is confidently wrong and nothing flags it. The
model has to be measured, not inferred.

---

## Not measured

| | Why it matters |
|---|---|
| **Wheel speed limit** | `WHEEL_MAX_SPEED_MPS` is a raised software clamp, not a measurement. The reach figures above use a calculated 1.22–1.41 m/s |
| **Friction coefficient, effective mass** | μ = 0.5 and 3.5 kg are assumptions. Every acceleration and reach number inherits them |
| **Stopping distance** | The controller drives at full speed to the target. Whether overshoot stays inside the 13 cm bin radius is unknown |
| **Odometry scale and slip** | Never compared against a tape measure, despite slip being expected |
| **Loop rate under load** | No fps logging exists; whether the 150 ms watchdog is ever exceeded is unknown |
| **Camera tilt** | Measured at 3.9°, worth ~4 cm of landing error. The estimator assumes a vertical optical axis and cannot correct it |

The tools for most of these exist in the repository — `pico_test.py --max`
for stopping distance, `teleop_test.py --speed-test` for top speed,
`test_accuracy.py` for prediction error against measured ground truth. They
were written but not run before the project ended.

---

## Sources

Figures in §1–§5 are from the project's final report (LABA 6th cohort,
Hanyang University, 2026). Design rationale for the constants involved is in
[`algorithm.md`](../design/algorithm.md) and [`physics.md`](../design/physics.md); the full
list of known-unresolved issues is in
[`open-questions.md`](../design/open-questions.md) (Korean).
