# Robot parameters and their sources

The machine-readable values are in [`robot_parameters.yaml`](../src/hexapod_description/config/robot_parameters.yaml). This file records where each one came from.

| Label | Meaning |
|---|---|
| **MEASURED** | Taken from the STL files by [`measure_geometry.py`](../scripts/measure_geometry.py), which fails if a value drifts by more than 0.1 mm |
| **DERIVED** | Calculated from measured parts and how they fit together; not a single direct measurement |
| **PUBLISHED** | Stated in the RookiDroid repo |
| **DATASHEET** | Stated by a servo datasheet or vendor |
| **ESTIMATED** | My assumption. Replace it with a real measurement when you have the robot |
| **NOT MEASURED** | Unknown |

Sources: RookiDroid commit `625d150` ([repo](https://github.com/rookidroid/hexapod)); `mochi.json` means `software/path_tool/robots/mochi.json`.

## Which robot this models

The brief asked for "RookiDroid v2, branch v2, MG92B". Those don't match the repo:

- There is no `v2` branch, only a `v2.0` tag, and its README says it uses **21G servos** and sends MG92B builds elsewhere.
- On `main`, the MG92B robot is **Mochi** (36/44/85 mm). The 21G robot is **Nougat** (38/54/97 mm).

**This model is Mochi**, because the servo decides what physically fits and it is the robot with MG92B files today. If you actually have a Nougat, the geometry and servo data must be redone.

## Frames and joint signs

`base_link`: x forward, y left, z up. Origin on the body centre axis, at the underside of `body_base`.

Each leg frame is `base_link` rotated by the leg's yaw, with x pointing outward. Axes are the same on all six legs, so one IK solution serves them all:

| Joint | Axis in leg frame | Positive motion | Zero |
|---|---|---|---|
| coxa | +z | foot swings counter-clockwise, seen from above | leg radial |
| femur | −y | foot rises | femur level |
| tibia | −y | foot swings outward | tibia vertical |

Zero is the RookiDroid calibration pose (`robots/mochi/images/calibration_1.jpg`). Leg order follows `mochi.json`:

| Leg | Name | Yaw | Hip x, y (mm) |
|---|---|---|---|
| 1 | front right | −30° | 70.84, −40.90 |
| 2 | centre right | −90° | 0, −81.80 |
| 3 | rear right | −150° | −70.84, −40.90 |
| 4 | front left | +30° | 70.84, 40.90 |
| 5 | centre left | +90° | 0, 81.80 |
| 6 | rear left | +150° | −70.84, 40.90 |

Left legs mirror only the bodies: meshes, centre of mass, inertia and collision shapes. The firmware's `legScale` flips left femur and tibia servo direction, which the hardware interface will have to apply.

## Geometry

| Parameter | Value | Label | How it was obtained |
|---|---|---|---|
| Hip radius | 81.80 mm | MEASURED | Six 7 mm bearing seats in `body_base`, 60° apart, 0.000 mm spread. `mochi.json`: 81.80 |
| Coxa length | 36.00 mm | MEASURED | `joint_bottom` pivot to cross hole 18.0 mm, twice. `mochi.json`: 36.0 |
| Femur length | 43.60 mm | MEASURED | `leg_bottom` bearing seat spacing. `mochi.json`: 43.6 |
| Tibia length | **83.46 mm** | DERIVED | See below. `mochi.json`: **85.22** |
| Hip height above base underside | 20.765 mm | DERIVED | `joint_cross` half-extent 20.765, with the pin-side plate against the base. Pin stack leaves about 0.06 mm gap |
| Body, arm tip to arm tip | 183.60 mm | MEASURED | `body_base` bounding box |
| Body, other axis | 167.26 mm | MEASURED | `body_base` bounding box |
| Arm height | 28.80 mm | MEASURED | Base 5.0 + `body_servo_side` 20.8 + top 3.0 |
| Arm width | 22.40 mm | MEASURED | Side plates centred at ±9.2 mm, 4 mm thick |
| Body height with dome | 89.80 mm | DERIVED | Arm 28.8 + battery plate 3.0 + dome 58.0 |
| Femur block | 64.6 × 28.8 × 22.4 mm | MEASURED | Offset 6.36 mm away from the horn side |

**Tibia length.** The foot plates only reach 60 mm past the pivot. The tip is on `foot_ground`, stacked as 20.00 (pivot to plate start) + 56.46 (plate start to notch floor) + 7.00 (tip beyond floor) = 83.46 mm. This is 1.76 mm (2.1%) shorter than `mochi.json`, and I couldn't find out why. The STL value is in the model. Measure it with calipers on the real robot, because a wrong value here shifts foot height directly.

**Fit check.** Parts that should touch in the assembled model do so within 0.002–0.012 mm. The standby pose in RViz matches RookiDroid's `hexapod_model.jpg`.

**Not modelled.** The assembled foot sits 1.02 mm to the pin side of the leg plane. The kinematic model keeps the foot on the plane so that IK stays planar; the meshes show the true position. Foot plate position has about 2 mm of uncertainty, so this offset is within the uncertainty.

## Servo: MG92B

| Parameter | Value | Label | Source and notes |
|---|---|---|---|
| Mass | 13.8 g | DATASHEET | [Akizuki datasheet](https://akizukidenshi.com/goodsaffix/mg92b.pdf). [Adafruit](https://www.adafruit.com/product/2307): 14 g |
| Size | 22.8 × 12 × 31 mm | DATASHEET | Datasheet. MEASURED pockets are 23.4 × 12.5 mm, which fits. Adafruit's 36 mm length is a conflict; it likely includes the mounting ears |
| Stall torque | 3.1 kgf·cm (0.304 N·m) at 5.0 V | DATASHEET | Adafruit lists this at 4.8 V |
| Stall torque | **3.5 kgf·cm (0.343 N·m) at 6.0 V** | DATASHEET | Used for `effort`. All sources agree |
| Speed | 0.13 s/60° (8.06 rad/s) at 5.0 V | DATASHEET | Adafruit lists this at 4.8 V |
| Speed | **0.08 s/60° (13.09 rad/s) at 6.0 V** | DATASHEET | Used for `velocity`. No load, so optimistic |
| Travel | 160° at 500–2400 µs; 150° at 700–2300 µs | DATASHEET | Adafruit and RookiDroid say about 180° |
| Voltage range | 5.0–6.6 V | DATASHEET | |
| Servo rail voltage | ? | NOT MEASURED | 2S pack is 7.4–8.4 V and the board has a DC-DC module, but its output is undocumented. **Measure it**: 6 V ratings are an assumption |

**Rejected source.** [TowerPro's product page](https://towerpro.com.tw/product/mg92b-robot-servo-360-rotation/) lists 0.78 and 0.48 s/60°, about six times slower than the datasheet. That page is titled as a 360° servo. Its other values match the datasheet, so one of the two speeds is wrong or they are different products. Time a real servo to settle this: it directly affects the gait speed limit.

**Calibration risk.** The firmware maps 102–512 PCA9685 ticks to 180° (`mochi.json`, README). At the firmware's 50 Hz (`SERVO_PWM_FREQ`) that is about 500–2500 µs. The datasheet gives 160° for 500–2400 µs, so real angles may be about 10% smaller than the firmware assumes. Check this before comparing simulation with hardware.

## Joint limits

| Joint | Limit | Why |
|---|---|---|
| Coxa | ±45° | PUBLISHED design range. MEASURED contact with the body at ±62° |
| Femur | ±75° | PUBLISHED design range. MEASURED contact at ±94°; datasheet travel about ±80° |
| Tibia | −50° to +75° | **Lower limit MEASURED**: tibia hits the femur after −50°, tighter than the published ±75°. No contact on the + side |

`mochi.json` declares the limits as `coxia 45, femur 75, tibia 75`, but its IK code never reads them. They are a stated range, not an enforced one.

Measured limits come from [`sweep_limits.py`](../scripts/sweep_limits.py), a 1° mesh collision sweep with printed parts and servo bodies. Horns, screws and wires are not included, so real limits may be slightly tighter.

**Limits that depend on other joints** (a URDF can't express these, so simulation or the controller must handle them):

- Neighbouring legs touch when each swings about 39–41° toward the other, inside the coxa limit.
- At femur −75°, the tibia reaches the body after only −26°.

**Poses**, converted from `mochi.json` (PUBLISHED):

| Pose | Coxa | Femur | Tibia | Foot, relative to hip |
|---|---|---|---|---|
| neutral | 0° | 0° | 0° | 79.6 mm out, 83.5 mm down |
| standby | 0° | +30° | −15° | 95.4 mm out, 58.8 mm down |
| laydown | 0° | +65° | 0° | 130.1 mm out, 4.2 mm up |

## Masses (all ESTIMATED)

Printed parts: density × (shell + 20% × interior), with volume and area MEASURED from each STL.

- Shell: 0.8 mm walls/top/bottom, from area × thickness. This is an assumption and is approximate on thin parts.
- PLA 1.24 g/cm³. Foot tips in TPU 1.21 g/cm³, as the README recommends.

| Part | Qty | Volume (cm³) | Area (cm²) | Each (g) |
|---|---|---|---|---|
| `body_base` | 1 | 47.39 | 394.50 | 43.06 |
| `body_top` | 1 | 19.67 | 187.18 | 19.73 |
| `body_battery` | 1 | 52.45 | 348.73 | 40.68 |
| `body_head` | 1 | 172.78 | 589.11 | 89.60 |
| `body_side` | 6 | 2.50 | 25.79 | 2.67 |
| `body_servo_side` | 12 | 2.03 | 19.09 | 2.02 |
| `joint_top` | 12 | 1.36 | 13.54 | 1.41 |
| `joint_bottom` | 12 | 1.44 | 12.43 | 1.34 |
| `joint_cross` | 6 | 4.05 | 35.39 | 3.81 |
| `leg_bottom` | 6 | 5.34 | 39.95 | 4.50 |
| `leg_top` | 6 | 2.64 | 27.83 | 2.86 |
| `leg_side` | 12 | 4.34 | 38.35 | 4.12 |
| `foot_bottom` | 6 | 2.24 | 23.04 | 2.38 |
| `foot_top` | 6 | 2.11 | 23.85 | 2.42 |
| `foot_ground` | 6 | 7.31 | 56.24 | 6.27 |
| `foot_tip` | 6 | 0.67 | 6.15 | 0.64 |

Quantities are from the Mochi README. The body gets all 6 coxa servos, each femur block holds 2 servos, and the tibia holds none.

Non-printed parts (ESTIMATED; hardware quantities PUBLISHED, split per link by me):

| Item | Mass each | Notes |
|---|---|---|
| M2×12 screw + nut | 0.47 g | 180 total; per leg 8 coxa, 12 femur, 7 tibia; the remaining 18 on the body |
| M2×6 screw | 0.20 g | 36 total, 2 per servo |
| M4×6 pin | 0.59 g | 18 total |
| MR74-2RS bearing | 0.50 g | 18 total |
| 18650 cell | 46 g | 2 cells, typical cell mass |
| Controller board | 30 g | Guess from its photo |
| Toggle switch | 10 g | Guess |
| Controller-side wiring | 15 g | Guess. Servo wires are counted in servo mass |

Component positions inside the body are also ESTIMATED.

| Link | Mass (g) |
|---|---|
| base_link | 477.0 |
| coxa | 14.3 |
| femur (with 2 servos) | 50.6 |
| tibia | 15.0 |
| foot | 0.6 |
| **One leg** | **80.5** |
| **Robot** | **959.9** |

**Weigh the robot when built.** Inertias are computed from the same parts, so they carry the same error.

**Torque margin.** In tripod stance at standby, holding still takes about 0.19 N·m at each femur, or 54% of the 6 V stall torque (0.068 N·m at the tibia). This is a static figure. Accelerations, faster gaits and any heavier real build reduce the margin quickly.

## Known gaps

- **No IMU in the Mochi parts list.** `imu_link` is a placeholder at the body centre. Set its position once you choose a sensor and mount.
- **Servo bodies and horns are not in the meshes**, as RookiDroid only provides printed parts. They are included in mass and inertia.
- **`body_battery` and `body_head` yaw** relative to the legs is unconfirmed (visual and small inertia effect only).
- **Collision shapes are simplified.** The foot uses a cylinder, as requested. Sphere feet are usually more stable for contact, so revisit this if contacts jitter.
- **Joint friction and damping are not set.** They need identifying on real servos.

## Licence

The meshes in `hexapod_description/meshes` derive from RookiDroid's **GPL-3.0** STL files, so the package is marked `GPL-3.0-only`. This matters if you distribute the project. The Siedmiu repo has no licence file, so nothing is copied from it.

## What the Siedmiu repo contributed

Its URDF is useful for structure but describes a different, larger robot, so **none of its numbers are used**. It repeats properties per leg (945 lines) with a separate axis for each. This package instead reuses one macro and keeps axes identical in the leg frame. Its package layout (ros2_control and Gazebo in separate xacro files) is a good pattern for later steps.
