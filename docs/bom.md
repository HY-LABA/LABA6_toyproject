# Bill of Materials

Everything needed to build one robot. Prices are what we actually paid in
July 2026, in Korean won (KRW), from Korean suppliers. Treat them as a
rough guide — roughly **USD 700 / EUR 640** at 2026 rates.

Most parts have direct equivalents anywhere. The ones that matter are called
out under [Substitutions](#substitutions).

> **The wiring diagram is [`images/wiring-diagram.png`](images/wiring-diagram.png).**
> Read it alongside this list — it shows every connection including the
> power distribution, which is the part most likely to hurt you if you get
> it wrong.

---

## 1. Compute and vision

| # | Part | Spec | Qty | Unit (KRW) | Notes |
|---|---|---|---|---|---|
| 1 | Raspberry Pi 5 (8 GB) | 45 g | 1 | 322,300 | Main computer. Vision, trajectory, targeting |
| 2 | Raspberry Pi AI HAT+ 13 TOPS | Hailo-8L, 28 g | 1 | 116,600 | YOLO inference. Connects over PCIe |
| 3 | Raspberry Pi Active Cooler | 5 V DC | 1 | 7,700 | The Pi 5 throttles without it under continuous inference |
| 4 | Raspberry Pi Pico | 26 GPIO, 16 PWM channels, 3 g | 1 | 5,940 | Real-time motor control. See [why a separate MCU](pico-control.md) |
| 5 | **Global shutter camera** | Sony IMX296, 1456×1088 @ 60 fps | 1 | 79,200 | **Global shutter is not optional** — see below |
| 6 | M12 fisheye lens | ~2.8 mm, measured 133.5° diagonal | 1 | bundled | Ships with the InnoMaker module we used |
| 7 | USB-A to Micro-B cable | 0.6 m | 1 | 1,000 | Pi 5 ↔ Pico link (USB CDC, **not** UART) |

### Why global shutter

A rolling shutter camera exposes the image one row at a time. A falling
object crosses the frame in a few tens of milliseconds, so different rows see
it at measurably different instants. The trajectory fit treats every pixel in
a frame as sharing one timestamp, and a rolling shutter silently violates
that. The resulting error looks like a skewed, slightly wrong parabola — it
does not look like a bug.

### Camera sourcing note

Our BOM originally listed the **Raspberry Pi Global Shutter Camera** (also
IMX296, C/CS mount, no lens included). The robot was actually built with an
**InnoMaker CAM-IMX296Color-GS**, which bundles the M12 fisheye we calibrated
against. Either sensor works; what matters is that **you calibrate whatever
lens you end up with** ([`calibrate.py`](../pi5/prep/calibrate.py)). The
intrinsics in [`config.py`](../pi5/config.py) are ours and will not match your
lens.

---

## 2. Drive

| # | Part | Spec | Qty | Unit (KRW) | Notes |
|---|---|---|---|---|---|
| 8 | **DFRobot FIT0186** geared DC motor | 12 V, 251 RPM, 43.8:1, 18 kgf·cm stall, 7 A stall | 3 | ~45,200 | Built-in Hall quadrature encoder |
| 9 | BTS7960 motor driver module | 43 A cont. / 60 A peak, 5.5–27 V in, 3.3–5 V logic, 30 g | 3 | 5,160 | One per motor |
| 10 | NEXUS 14049 omni wheel | 100 mm, double plastic, rubber rollers, 290 g | 3 | 25,200 | 120° triangular layout |
| 11 | NEXUS 18007 aluminium mounting hub | for 6 mm shaft, 56 g | 3 | 14,700 | Wheel ↔ motor shaft |
| 12 | Geared motor bracket | GMB-25M / GM27 / MB-2430 | 3 | 4,070 | Motor to chassis |

> ⚠ **Motor spec changed during the project.** The original BOM listed a
> 350 RPM / 34:1 / 12 kgf·cm motor (DFRobot FIT0493). We switched to the
> **FIT0186**, which trades top speed for torque. It barely mattered:
> acceleration is limited by floor traction, not by the motor, so the
> effective catch radius moved by less than a centimetre. The reasoning is in
> [`physics.md` §3](physics.md). **Use the FIT0186 numbers** — they are what
> the firmware constants assume.

### Why 100 mm wheels and not 127 mm

Larger wheels raise top speed (1.83 → 2.33 m/s on paper) but cost more than
they give:

- Rotational inertia rises, so acceleration drops.
- A taller wheel puts the contact patch further from the drive axis, losing
  torque transfer.
- Above 100 mm, the only omni wheels available to us were aluminium, and
  much heavier.

This robot spends its entire catch window accelerating — it almost never
reaches top speed. **Acceleration wins.** See [`physics.md`](physics.md).

---

## 3. Power

Two isolated supplies that share a common ground. Motors are electrically
noisy and will brown out a Pi if they share a rail.

| # | Part | Spec | Qty | Unit (KRW) | Notes |
|---|---|---|---|---|---|
| 13 | 3S LiPo battery | 11.1 V, 2200 mAh, **55C**, XT60, 14 AWG | 1 | ~27,000 | **Motor rail only** |
| 14 | Matek PDB-XT60 power distribution board | XT60 in, dual BEC 5 V / 12 V, 18 AWG out | 1 | 7,800 | Splits the motor rail three ways |
| 15 | 18650 UPS module (X1200 v1.2) | 5 V 5 A, 85 × 56 mm | 1 | 49,500 | **Compute rail only.** ~6 h runtime |
| 16 | 18650 Li-ion cell | 3.7 V, 2200 mAh | 2 | 2,250 | For the UPS module |
| 17 | LiPo low-voltage alarm (BX100) | buzzer, 11 g | 1 | 8,000 | **Do not skip.** Over-discharging a LiPo ruins it and can start a fire |
| 18 | Automotive blade fuse | 32 V, 30 A | 2 | 100 | Inline on the battery positive lead |
| 19 | Fuse holder (SZH-FU004) | ATO/ATC, 12 AWG | 2 | 2,200 | |

> **On the C rating.** Our purchase list specified a 35C pack; we ended up
> using a 55C pack that was already in the lab. Either is comfortable here —
> three motors at 7 A stall is 21 A worst case, and even 2200 mAh × 35C is
> 77 A. Anything 20C or above has plenty of margin.

---

## 4. Wiring and fasteners

| # | Part | Spec | Qty | Unit (KRW) | Notes |
|---|---|---|---|---|---|
| 20 | Silicone wire kit | 18 AWG, 6 colours, 30 m | 1 | 26,000 | Battery → drivers → motors. 14–18 AWG required |
| 21 | Jumper wires M/M | 20 cm, 40 pcs | 1 | 3,000 | Logic signals |

### ⚠ Encoder VCC must be 3.3 V, not 5 V

The Hall encoders accept 3.3–5 V and their output swings to **whatever VCC
you give them**. At 5 V they work perfectly — and they will damage the Pico,
whose GPIO is not 5 V tolerant. Take encoder power from the Pico's 3V3 rail.

This is the single easiest way to destroy the board, and it fails silently:
the encoder is fine, the Pico dies. Details in [`hardware.md`](hardware.md).

---

## 5. Enclosure (the bin)

Not in the purchase BOM — we built this ourselves, and **it is a performance
part, not a cosmetic one.** The catch radius is the distance the robot can
travel plus the radius of the opening, so every centimetre of opening is a
centimetre of catch radius, essentially for free.

| Property | Design | As built |
|---|---|---|
| Outer diameter | 300 mm | 300 mm |
| **Clear opening** (inside the rim) | — | **260 mm** |
| Overall height | — | 490 mm |
| Depth | 200–250 mm | |
| Shape | circular | The robot never rotates, so orientation does not matter |
| Lining | foam or cloth | Stops objects bouncing back out |
| Camera position | at the rim, facing up | 160 mm forward of the centre of rotation, 30 mm above the rim plane |

> **Measure the clear opening, not the outer diameter.** Ours is 300 mm
> across the outside and **260 mm** inside the rim. The object has to fall
> through the opening, so 260 mm is the number that goes into
> `CATCH_OPENING_DIAMETER_M` and into every catch-radius calculation — a
> 20 mm rim quietly costs you 20 mm of catch radius.

The camera numbers are not decoration either — they are `CAMERA_OFFSET_M` and
`CATCH_HEIGHT_M` in [`config.py`](../pi5/config.py). **Measure yours and put
them in.** With the offset left at zero the robot parks 16 cm off target on
every single throw.

Sizing rationale is in [`physics.md` §6](physics.md).

---

## 6. Cost summary

| | KRW |
|---|---|
| Full build from nothing | **932,630** |
| What we actually spent (we already owned the Pi 5, AI HAT, cooler, cables) | **456,030** |

The Pi 5 and the AI HAT are over half the total. If you already have a Pi 5
and any NPU, or are willing to run a smaller model on the CPU at lower frame
rate, this gets a lot cheaper.

---

## Substitutions

| Part | Free to swap? | Why |
|---|---|---|
| Raspberry Pi 5 | Yes, with care | Any Linux SBC that can run Picamera2 and your NPU's runtime. The vision loop needs to hold ~60 fps |
| AI HAT+ / Hailo-8L | Yes | Any accelerator, or CPU inference if you accept fewer frames. Fewer observations means a worse fit — see [`algorithm.md`](algorithm.md) |
| Camera | **Global shutter only** | See above |
| Lens | Yes, but recalibrate | Wider sees the object longer; narrower resolves depth better. Ours is 133.5° diagonal |
| Motors | Yes | Update `WHEEL_MAX_SPEED_MPS`, `ENCODER_COUNTS_PER_REV`, and the PID gains |
| BTS7960 | Yes | Any driver that takes two PWM inputs per motor and handles the stall current |
| Omni wheels | Size, yes; type, no | Must be omni (or mecanum with different kinematics). Update `WHEEL_DIAMETER_M` |
| Pico | Yes | Any MCU with hardware quadrature decoding (or fast enough interrupts) and 6 PWM channels. We use PIO for encoders |
| Batteries | Yes | Keep the two rails separate |

---

## Before you order

Three things will cost you a rebuild if you get them wrong:

1. **Global shutter camera.** Rolling shutter corrupts the fit in a way that
   looks like bad tuning rather than bad hardware.
2. **Encoder VCC at 3.3 V.** See above.
3. **Three identical motors.** The kinematics assumes all three wheels
   behave the same. Mixed motors or mixed gear ratios will make the robot
   curve under load, and it is genuinely hard to diagnose after the fact.

Assembly conventions — wheel angles, camera orientation, what `+X` and `+Y`
mean — are in [`hardware.md`](hardware.md). **Read that before you glue
anything down**, especially the wheel numbering: which physical motor is M1
determines what "forward" means to the whole software stack.
