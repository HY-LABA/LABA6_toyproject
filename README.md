# Moving Trash Bin

**A three-wheeled omnidirectional robot that catches thrown objects by
predicting where they will land — using a single upward-facing camera.**

한국어 문서는 [`README.ko.md`](README.ko.md)를 보세요.

---

## The idea

One photograph cannot tell you how far away something is. A small object
nearby and a large object far away produce exactly the same pixels. Most
robots solve this with a second camera, a depth sensor, or a known object
size.

This one uses **gravity as a ruler.**

Gravity is 9.8 m/s² regardless of what the object is. So instead of asking
"how far away is it?", the system asks:

> *For this sequence of pixel positions to have been produced by a constant
> 9.8 m/s² acceleration, how far away must the object have been?*

That question has one answer. The scale falls out of the physics, and the
object's size, mass and shape never enter the calculation. A can, a bottle
and a crumpled ball are handled identically, and the object may tumble
freely in flight — only the centre of its bounding box is used.

Concretely it is a **2N×6 linear least-squares solve**: six unknowns
(position and velocity at t₀), two equations per observation. No iteration,
no initial guess, no convergence failure. The trick is writing the pinhole
projection as a cross-product residual instead of a division, which removes
the nonlinearity:

```
u = fx·X/Z + cx          →    a·Z(t) − fx·X(t) = 0        where a = u − cx
```

Gravity enters as the inhomogeneous term — and that is precisely what makes
the solution unique rather than a family of solutions differing by scale.

Full derivation: [`algorithm.md`](algorithm.md) · Physics: [`docs/physics.md`](docs/physics.md)

---

## Status

**Working end to end.** The robot detects a thrown object, predicts its
landing point, and drives there. It has not been through a systematic
catch-rate evaluation, and several things below are known to be imperfect.

| | |
|---|---|
| ✅ Object detection | Custom YOLOv8n on a Hailo-8L, **60 fps sustained** |
| ✅ Trajectory estimation | Validated in simulation; median final error 0.1–0.4 cm |
| ✅ Camera calibration | 23 chessboard images, RMS 0.411 px, fisheye model |
| ✅ Pi ↔ Pico control loop | 50 Hz, closed loop on wheel encoders |
| ✅ Drives to the predicted point | Including an early-start path that commits on 3 frames |
| ⚠️ Catch success rate | **Not systematically measured** |
| ⚠️ Maximum speed | The configured limit is a guess, not a measurement |
| ⚠️ Lateral motion | Curves under saturation — see [Known limitations](#known-limitations) |

This was a student project at [HY-LABA](https://github.com/HY-LABA), Hanyang
University. It is published because the approach — and the reasoning behind
every constant — may be useful to others, not because it is a finished
product.

---

## How it works

```
                  ┌──────────── Raspberry Pi 5 (60 fps) ───────────┐
   [camera] ──────▶  vision      YOLO → bounding-box centre         │
   facing up      │              + fisheye undistortion             │
                  │     ↓                                           │
                  │  tracker     one hypothesis per detection,      │
                  │              physics gates decide which is real │
                  │     ↓                                           │
                  │  trajectory  gravity-constrained least squares  │
                  │              → 3D path → landing point + time   │
                  │     ↓                                           │
                  │  control     landing point → world-frame target │
                  └───────────────────┬────────────────────────────┘
                                      │ USB CDC
             target + time remaining  ▼   ▲  odometry
                  ┌───────────────────┴────────────────────────────┐
                  │        Raspberry Pi Pico (50 Hz)                │
                  │  encoders (PIO) → forward kinematics → pose     │
                  │  remaining distance ÷ remaining time → speed    │
                  │  → inverse kinematics → PID → PWM               │
                  └───────────────────┬────────────────────────────┘
                                      ▼
                            3 × motor / omni wheel
```

**The Pi decides where and by when. The Pico works out how fast and drives
the wheels.** The Pico never learns what a trajectory is — it receives one
coordinate and one duration.

Three design decisions are worth knowing about:

- **Every detection gets its own hypothesis.** YOLO sometimes scores a
  ceiling light higher than the real object. Rather than picking the most
  confident detection, the system tracks all of them and lets the physics
  decide: a stationary false positive cannot produce a parabola, so its
  reprojection residual explodes and it is rejected.

- **The robot starts moving after 3 frames**, before the depth is known.
  Bearing is accurate immediately even when range is not — an upward-facing
  camera maps screen position directly to azimuth. So the robot accelerates
  in the right direction while the estimate converges. Waiting for a full
  fit means the object has already passed overhead.

- **Losing sight of the object does not stop the robot.** A thrown object is
  ballistic; not seeing it does not change where it is going. And it almost
  always leaves the frame just before landing, which is exactly when the
  robot most needs to be moving.

---

## Hardware

Roughly **KRW 930,000 (~USD 700)** for a complete build; about half of that
is the Pi 5 and the AI accelerator.

- **[Bill of materials](docs/bom.md)** — every part, with specs and rationale
- **[Wiring diagram](docs/images/wiring-diagram.png)**
- **[Hardware notes](docs/hardware.md)** — power design, mounting conventions

Three things that will cost you a rebuild if you get them wrong:

1. **The camera must be global shutter.** A rolling shutter exposes rows at
   different instants, which silently violates the "one timestamp per frame"
   assumption the fit is built on.
2. **Encoder VCC must be 3.3 V, not 5 V.** They run happily at 5 V and will
   destroy the Pico's GPIO while appearing to work fine.
3. **All three motors must be identical.** Mismatched motors make the robot
   curve under load, and it is very hard to diagnose after assembly.

---

## Getting started

```bash
git clone https://github.com/HY-LABA/LABA6_toyproject.git
cd LABA6_toyproject
```

See **[docs/getting-started.md](docs/getting-started.md)** for the full path:
dependencies → firmware flash → camera calibration → direction check →
first run.

The order matters. In particular, **verify the drive direction before you
connect the camera** — with the camera in the loop, a wiring sign error and a
camera mounting error look identical, and you can spend a long time chasing
the wrong one. (We did.)

```bash
# Drive only. No camera, no YOLO.
python pi5/pico_test.py --port /dev/ttyACM0 --dir front --dist 0.5 --speed 0.3

# Prediction only. Robot stationary — measures the estimator in isolation.
python pi5/test_accuracy.py

# Everything.
python pi5/main.py
```

---

## Repository layout

```
pi5/                  Raspberry Pi 5 — Python
  main.py             frame loop: capture → detect → track → command
  trajectory.py       the least-squares solver. Pure functions, no state
  tracker.py          all the state: hypotheses, physics gates, commitment
  vision.py           Picamera2 + Hailo YOLO + undistortion
  control.py          landing point → world target; direction-aware speed limits
  communication.py    USB serial framing
  config.py           every tunable constant, each with its justification
  prep/               dataset collection, labelling and training tools

pico/                 Raspberry Pi Pico — C (Pico SDK). Flash the .uf2
pico_micropython/     the same firmware in MicroPython, for tuning without reflashing

docs/                 design rationale — "why this number"
```

---

## Documentation

The reference documentation is **in Korean** — about 4,400 lines of it,
including the reasoning behind essentially every constant in the codebase.
The English documents are listed first.

| Document | Language | Contents |
|---|---|---|
| [docs/bom.md](docs/bom.md) | English | Bill of materials, substitutions |
| [docs/getting-started.md](docs/getting-started.md) | English | Setup, calibration, first run |
| [docs/results.md](docs/results.md) | English | What was measured, and what was not |
| [algorithm.md](algorithm.md) | 한국어 | The estimator, the control loop, parameters |
| [architecture.md](architecture.md) | 한국어 | File-by-file responsibilities, data flow |
| [docs/physics.md](docs/physics.md) | 한국어 | Motion model, friction, catch radius, optics |
| [docs/hardware.md](docs/hardware.md) | 한국어 | Parts, power, mounting conventions |
| [docs/pico-control.md](docs/pico-control.md) | 한국어 | Real-time loop, kinematics, odometry, PID |
| [docs/protocol.md](docs/protocol.md) | 한국어 | The Pi ↔ Pico wire contract |
| [docs/vision-pipeline.md](docs/vision-pipeline.md) | 한국어 | Dataset, labelling, training, Hailo compilation |
| [docs/open-questions.md](docs/open-questions.md) | 한국어 | Unresolved risks, honestly listed |
| [CHANGELOG.md](CHANGELOG.md) | 한국어 | Every design change and why it happened |

If you read one document, read [`algorithm.md`](algorithm.md). If you read
two, add [`docs/open-questions.md`](docs/open-questions.md) — it is the list
of things we know are wrong.

---

## Results

Synthetic throws at the design camera spec, 150 runs per row, median:

| Drop height | Detection noise | First prediction | Time left to move | Final error | Success |
|---|---|---|---|---|---|
| 1.5 m | 1.0 px | 0.47 s | 0.26 s | 0.1 cm | 100% |
| 2.0 m | 1.0 px | 0.53 s | 0.28 s | 0.2 cm | 100% |
| 2.5 m | 0.5 px | 0.48 s | **0.40 s** | 0.1 cm | 100% |
| 2.5 m | 2.0 px | 0.78 s | 0.10 s | 1.5 cm | **30%** |
| 3.0 m | 1.0 px | 0.65 s | 0.30 s | 1.0 cm | 99% |

**Detection noise dominates everything.** The estimator is not the
bottleneck — at 0.5 px the robot gets 0.40 s to move; at 2.0 px it gets
0.10 s and mostly fails. Motion blur suppression (short exposure, high gain)
converts directly into catch performance, which is where a global shutter
with large pixels earns its cost.

> ⚠️ These are **simulations**, run at f_px = 1739. The lens actually fitted
> measures f_px = 973, so the same pixel noise is ~1.8× more angular error —
> partly offset by the wider field of view keeping the object in frame for
> longer. **This table needs re-running against the real optics.**

What was verified on the physical robot, and what was not, is in
[`docs/results.md`](docs/results.md).

---

## Known limitations

Listed because they are more useful to you than a clean README would be.
Full list in [`docs/open-questions.md`](docs/open-questions.md).

- **Wheel speed limit is unmeasured.** `WHEEL_MAX_SPEED_MPS` is set above
  what the motors actually deliver. When the robot moves sideways, one wheel
  must turn twice as fast as the other two, saturates alone, and the robot
  curves instead of translating. Moving forward is unaffected, because there
  the two active wheels saturate together and the ratio is preserved.

- **Drive direction is corrected in software.** The wheels currently run
  inverted relative to the firmware's convention, compensated by
  `DRIVE_INVERT` on the Pi side rather than fixed in firmware. The root cause
  has not been found.

- **Heading drifts.** The Pico integrates θ from encoders with no reference
  and never resets it. Wheel slip or spurious rotation accumulates, and θ is
  used to rotate incoming commands — so the error feeds back into steering.

- **Camera tilt is not compensated.** The estimator assumes the optical axis
  is exactly vertical. Ours is off by ~3.9°, worth roughly 4 cm of landing
  error.

- **Many real throws are physically uncatchable.** Two logged attempts
  required 3.8 and 2.5 m/s; the robot does about 1.2–1.4 m/s. Detection was
  flawless in both. Any evaluation has to separate "missed because
  perception failed" from "missed because no robot could have made it."

---

## License

- **Code** — [MIT](LICENSE)
- **Documentation and diagrams** — [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)

This project involves lithium polymer batteries, 40 A motor drivers and
moving machinery. It is provided with no warranty of any kind. Build and
operate at your own risk.

## Citation

See [`CITATION.cff`](CITATION.cff), or:

> Choi, J., Jang, I., et al. *Moving Trash Bin: a catching robot using
> gravity-constrained monocular trajectory estimation.* HY-LABA, Hanyang
> University, 2026. https://github.com/HY-LABA/LABA6_toyproject

## Acknowledgements

Built by the LABA 6th cohort at Hanyang University, advised by Prof. 나현종.
