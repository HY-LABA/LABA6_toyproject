# Getting started

From a fresh Raspberry Pi to a robot that catches things.

**Follow the order.** Each step is verifiable on its own, and each one
assumes the previous step passed. Skipping ahead is how you end up debugging
three problems at once — which is exactly what happened to us with the
direction check (step 5).

---

## 0. What you need first

- A built robot — see the [bill of materials](bom.md) and the
  [wiring diagram](../images/wiring-diagram.png)
- A Raspberry Pi 5 running Raspberry Pi OS (64-bit, Bookworm or later)
- A Raspberry Pi Pico wired to three BTS7960 drivers and three encoders
- **Space.** The robot moves at over 1 m/s and has no obstacle detection.
  Clear 2–3 m in every direction and keep a hand on Ctrl+C.

---

## 1. Software dependencies

On the Pi:

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-serial

# Hailo runtime — follow Raspberry Pi's official AI HAT+ instructions,
# then confirm:
hailortcli fw-control identify
```

Optional:

```bash
sudo apt install -y python3-pygame     # gamepad teleop only
pip install ultralytics                # only if you retrain the model
```

Check everything at once:

```bash
python src/pi5/prep/check_setup.py
```

This reports Python version, missing packages, camera access and free disk
space. Fix anything it flags before continuing.

> There is deliberately no `requirements.txt`. On Raspberry Pi OS,
> `picamera2` and the Hailo runtime must come from apt — installing them with
> pip produces a broken environment that fails in confusing ways at runtime.

---

## 2. Flash the Pico

Two firmware editions with identical behaviour. Only one can be on the board
at a time.

### C (default)

```bash
cd src/pico
mkdir -p build && cd build
cmake -G Ninja ..     # needs PICO_SDK_PATH set
ninja
```

Hold **BOOTSEL**, plug in the Pico, and copy `build/moving_trash_bin.uf2`
onto the drive that appears.

### MicroPython (for tuning without reflashing)

Copy everything in `src/pico_micropython/` to the board with Thonny. See
[`src/pico_micropython/README.md`](../../src/pico_micropython/README.md).

> ⚠️ Uploading MicroPython **replaces** the C firmware. To go back, reflash
> the `.uf2`.
>
> ⚠️ The MicroPython edition needs `micropython.kbd_intr(-1)`. Without it, a
> `0x03` byte in the binary protocol is interpreted as Ctrl-C and kills the
> firmware mid-run.

Confirm the link:

```bash
ls /dev/ttyACM*          # the Pico should appear
python src/pi5/control.py    # compares src/pi5/config.py against src/pico/config.h
```

The last command reports any mismatch in wheel angles, mounting radius or
speed limits between the two sides. It should report zero mismatches.

---

## 3. Camera calibration

**Do not skip this.** The intrinsics in `config.py` are ours. A 10% error in
focal length is a 10% error in every depth estimate, and it will not look
like a calibration problem — it will look like the estimator being bad.

Print a chessboard, then:

```bash
python src/pi5/prep/calibrate.py capture --camera gs     # ~20 images, varied angles
python src/pi5/prep/calibrate.py solve --model fisheye   # or --model pinhole
```

Choose the model by the measured field of view, which the script prints:

| Measured diagonal FOV | Model |
|---|---|
| Under ~90° | `pinhole` |
| Over ~90° | `fisheye` |

Ours came out at 133.5° diagonal, f_px 973, RMS 0.411 px. Copy the resulting
`CAMERA_FX/FY/CX/CY`, `CAMERA_DISTORTION` and `CAMERA_MODEL` into
[`src/pi5/config.py`](../../src/pi5/config.py).

> Getting the model wrong is the expensive failure. With a fisheye lens fed
> through a pinhole model, a 42° incidence angle produces a **79% depth
> error** — and the reprojection residual stays small, so nothing warns you.
> See [`docs/design/vision-pipeline.md`](../design/vision-pipeline.md).

---

## 4. Measure the robot

Every one of these is a real number on your robot, not a default:

| Constant | What it is | How |
|---|---|---|
| `CAMERA_OFFSET_M` | Camera position relative to the centre of rotation, in body axes | Tape measure |
| `CATCH_HEIGHT_M` | Height of the bin opening relative to the camera. Negative if the camera is higher | Tape measure. **Check the sign** |
| `CATCH_OPENING_DIAMETER_M` | Bin opening diameter | Tape measure |
| `WHEEL_ANGLES_RAD` | Wheel mounting angles | Should be 90°/210°/330°; verify against the build |
| `WHEEL_MOUNT_RADIUS_M` | Centre to wheel distance | Tape measure |

`CAMERA_OFFSET_M` at its default of zero makes the robot stop 16 cm short on
every throw — a systematic error that no amount of averaging removes.

---

## 5. ★ Direction check — before the camera goes anywhere near this

This is the step to take seriously. With the camera in the loop, a motor
wiring inversion and a camera mounting rotation produce **identical**
symptoms, and you cannot tell them apart. Pin down the drive first.

```bash
python src/pi5/pico_test.py --port /dev/ttyACM0 --dir front --dist 0.5 --speed 0.3
```

Watch the robot, not the log:

| What you see | What it means |
|---|---|
| Moves toward the M1 wheel | Correct |
| Moves **exactly opposite** | All three motor signs inverted. Flip `MOTOR_SIGN` in `src/pico/config.h` and reflash, **or** set `DRIVE_INVERT = True` in `src/pi5/config.py` |
| Drifts diagonally, or rotates | One motor is wrong, not all three. Check wiring and `MOTOR_PINS` order |

> **`MOTOR_SIGN` cannot be verified from logs.** It multiplies both the
> encoder reading and the PWM output, so the control loop stays perfectly
> self-consistent and the reported odometry looks correct — while the robot
> physically drives the other way. Only your eyes can judge this.

Then check all eight directions:

```bash
python src/pi5/pico_test.py --port /dev/ttyACM0 --all --dist 0.5 --speed 0.3
```

The accumulated drift it reports at the end is your odometry quality. The
same pose feeds the catching loop, so whatever error you see here is error
you will see there.

---

## 6. Camera orientation

Now that "forward" means something physical, find how the camera is rotated
relative to it.

```bash
python src/pi5/measure_camera_yaw.py live       # needs a display
python src/pi5/measure_camera_yaw.py capture    # headless: saves annotated frames
```

Because the camera looks up, anything above it appears at its true azimuth.
Stand at a known direction from the robot, click yourself in the frame, press
`f`/`r`/`b`/`l` to record which direction that was, then `s` to solve.

Do all four directions. The solver then separates yaw from **tilt** — with
four evenly spaced observations the tilt bias cancels to first order, and a
10° tilt still leaves yaw accurate to 0.05°.

Put the result in `CAMERA_YAW_RAD` — **wrapped in `math.radians()`**. It is a
radian field; writing `150` there means 150 radians, which is about 313°.
(Yes, this cost us an afternoon.)

> ⚠️ This measurement sees only the camera. It cannot see which way the
> motors turn. If the drive is also inverted, the photo measurement and the
> value that actually works will differ by exactly 180° — which is why step 5
> comes first, and why drive inversion belongs in `DRIVE_INVERT` rather than
> being folded into this angle.

---

## 7. The detection model

The trained `.hef` is not in this repository. Either train your own —
[`docs/design/vision-pipeline.md`](../design/vision-pipeline.md) covers dataset collection,
labelling, training and Hailo compilation — or point at any single-class
detector you have:

```bash
TRASH_HEF=/path/to/model.hef python src/pi5/main.py
```

Four collection paths are provided in [`src/pi5/prep/`](../../src/pi5/prep/), differing
in how labels are produced. The important property is that **the labeller's
bias must differ from the model you are training**, or you only teach the
model what it already knows:

| Path | Camera | Labelled by |
|---|---|---|
| `capture_video.py` → `extract_from_video.py` | stationary | motion (MOG2) |
| `capture_roam.py` → `label_throws.py` | driving | a human |
| `collect_throws.py` | either | the current YOLO + physics fit |
| `collect_live.py`, `extract_background.py` | either | empty labels (hard negatives) |

---

## 8. Prediction accuracy, with the robot stationary

```bash
python src/pi5/test_accuracy.py --throws 10
```

Leave the robot switched off and still. This measures the estimator alone —
no motors, no control error. Throw, then enter where the object actually
landed (`forward left`, in cm):

```
> 55 -20      # 55 cm forward, 20 cm to the right
```

Watch four things:

| Column | Meaning |
|---|---|
| Final prediction error | Must be inside the bin radius to ever catch anything |
| Depth | Underestimated early, should climb toward truth as observations accumulate |
| Residual (px) | Above `MAX_RESIDUAL_PX` the fit is rejected |
| Sign check | After 3+ throws it reports whether left/right is inverted |

Every observation is saved, so you can re-run with different parameters
without throwing anything again:

```bash
python src/pi5/test_accuracy.py --replay logs/accuracy/*.json --span 0.25
```

---

## 9. Run it

```bash
python src/pi5/main.py
```

Useful flags:

| Flag | Effect |
|---|---|
| `--once` | Single catch, then stop |
| `--no-gates` | Disable the physics gates. Diagnostic only — false positives pass straight through |
| `--max-residual-px 3.0` | Tighten the residual gate |
| `--hold 2.0` | Keep driving to the last target for N seconds |

`main.py` exits immediately if the Pico is not connected. To watch detection
without the robot, use `test_accuracy.py`.

---

## When it does not work

| Symptom | Likely cause |
|---|---|
| Moves ~10 cm and stops | Depth underestimated early, so the target lands right in front of the robot. `MIN_TARGET_DIST_M` puts a floor under it |
| Goes the wrong way | Step 5. Do not fix this by guessing at `CAMERA_YAW_RAD` |
| Curves when moving sideways | One wheel is saturating alone. `WHEEL_MAX_SPEED_MPS` is above what the motors deliver |
| Never commits to a trajectory | Check `run.log` for which gate rejects it. Usually `MIN_TIME_SPAN_S` or `MAX_RESIDUAL_PX` |
| Commits to ceiling lights | Expected while idle — the physics gates reject them. If it commits while driving, tighten `MAX_RESIDUAL_PX` |
| Stops the moment the object is lost | Should not happen; coast mode handles this. Check `COAST_EXTRA_S` |
| Predictions are wildly wrong but residuals look fine | Wrong camera model (pinhole vs fisheye). Step 3 |

`run.log` records every cycle's decision — observation count, residual,
depth, landing point, time remaining, odometry and the command sent. Most
questions are answered there.

Known unresolved problems are listed in
[`docs/design/open-questions.md`](../design/open-questions.md) (Korean).
