# Robot parameters and their sources

The machine-readable values are in [`robot_parameters.yaml`](../src/hexapod_description/config/robot_parameters.yaml). This file records where each one came from. The previous robot (RookiDroid Mochi) is at git tag `mochi-v1`.

| Label | Meaning |
|---|---|
| **MEASURED** | Taken from the STL files by [`measure_geometry.py`](../scripts/measure_geometry.py), which fails if a value drifts by more than 0.1 mm |
| **DERIVED** | Calculated from measured parts and how they fit together |
| **PUBLISHED** | Stated by the original project or the build guide |
| **DATASHEET** | Stated by a component seller or datasheet |
| **ESTIMATED** | An assumption. Replace it with a measurement once you have the robot |
| **NOT MEASURED** | Unknown |

## Which robot this models

The **MakeYourPet hexapod**, with the part selection used by the [Robot Lk build guide](https://robotlk.com/build-your-own-18dof-hexapod-robot/). 18 x MG996R, with our own electronics: ESP32 + 2 x PCA9685 + Raspberry Pi 5.

- **Source of truth:** [`github.com/MakeYourPet/hexapod`](https://github.com/MakeYourPet/hexapod), **MIT licence**, commit `7d8fc80`, cloned to `refs/makeyourpet_hexapod`. The meshes in `meshes/makeyourpet` derive from it and are committed with its licence.
- **Robot Lk zip:** used only to see which parts it chose. All 16 files are byte-identical to the MIT source (`measure_geometry.py` checks this if the zip is present). It stays out of git.
- **A mix of two designs.** The coxa, femur, idler, ring, ms-holder, rod, limitter and tip come from the repo's `Chipo` folder (femur 60 mm), and the frame, tibia, shield, covers, battery bar and calibration parts from its main `STL` folder (tibia about 134 mm). Neither published config matches exactly, so this robot's numbers are measured here.

**From the guide, and how they differ from what we use:**

| Guide | This project |
|---|---|
| Pimoroni Servo 2040 | ESP32 + 2 x PCA9685 + Pi 5 |
| Buck converter and 5 V relay | 6 V BEC and a high-current switch: see [`power_budget.md`](power_budget.md) |
| 2 x 8 mm self-tapping screws, M2 x 8 | The same. Screw holes are a Ø1.7 pilot in a Ø3 clearance |
| No bearings: printed idlers and rings | The same |
| 3S 2500 mAh | Sized below |
| 15% infill | 3 walls, 10% infill for mass estimates, as asked |

## Frames and joint signs

`base_link`: x forward, y left, z up. Origin on the body centre, at the underside of the frame, where the coxa servo flanges sit.

Each leg frame is `base_link` rotated by the leg's yaw, with x pointing outward. The axes are the same on all six legs:

| Joint | Axis in leg frame | Positive motion | Zero | Servo centre (DERIVED) |
|---|---|---|---|---|
| coxa | +z | foot swings counter-clockwise, seen from above | leg along its frame arm | 0° for now (see below) |
| femur | −y | foot rises | hub-to-hub line level | +35.2° |
| tibia | −y | foot swings outward | knee-to-foot line vertical | −21.4° |

| Leg | Name | Yaw | Hip x, y (mm) | Original project name |
|---|---|---|---|---|
| 1 | front right | −45° | 83.15, −62.95 | R1 |
| 2 | centre right | −90° | 0, −81.75 | R2 |
| 3 | rear right | −135° | −83.15, −62.95 | R3 |
| 4 | front left | +45° | 83.15, 62.95 | L1 |
| 5 | centre left | +90° | 0, 81.75 | L2 |
| 6 | rear left | +135° | −83.15, 62.95 | L3 |

STL +y is the robot's front (decided). Left-hand parts are placed directly, and right legs are their mirror image, as the guide does in the slicer.

## Geometry

| Parameter | Value | Label | How it was obtained |
|---|---|---|---|
| Coxa servo patterns | 48 x 10 mm holes; centres (±71.5, 0) and (±55.7, ±75.9) in the STL | MEASURED | 24 holes in `frame` |
| Servo shaft offset | 10.25 mm from the pattern centre | MEASURED | `calibration-arm` ring against the `calibration-ruler` hole pattern |
| Hip positions | See table above | DERIVED | Pattern centre + shaft offset, outward. Spacings 125.9 / 166.3 / 163.5 mm; PUBLISHED 126 / 167 / 163 |
| Coxa length | 43.00 mm | MEASURED | Horn screw circle centre to servo pattern. PUBLISHED 43 |
| Femur length | 60.00 mm | MEASURED | Hub centres. PUBLISHED 60 (Chipo), 80 (main) |
| Tibia length | 133.15 mm | DERIVED | Knee shaft to foot tip, through the tibia, `ms-holder`, rod and tip. PUBLISHED 134 (main) |
| Hip height | 3.25 mm above the frame underside | DERIVED | Servo stack below |
| Frame | 198 x 205.4 x 16 mm | MEASURED | Bounding box |
| Body stack | z −42 (bars) to +28.2 mm (top cover) | DERIVED | Cover pillars on the frame underside. Battery bar position is ESTIMATED |

**How the assembly was worked out:**

- **The frame STL is upside down.** Its screw holes open at z = 16, and the guide says frame servos go in from the underside. So that face is really the bottom, and the servo body rises through the frame.
- **Servo stack.** Flange to shaft end 11 mm, plus flange to wire-side end 30 mm, idler cap 4 mm and ring 3 mm, gives 48 mm: exactly the gap between the coxa arms.
- **Femur direction.** The horn screw pattern at the `x = 0` hub is rotated 35.2°, matching the guide's "35° femur", so that hub joins the coxa.
- **Knee.** Of the two possible shaft positions on the tibia's servo, only the upper one gives the PUBLISHED length (133.15 vs 134 mm) and attach angle (68.6° vs 68°).
- **Legs are planar.** Following the stack from coxa pivot to foot, the foot's lateral offset comes to exactly 0, so the library's planar convention holds.

**Coxa servo centre.** The project publishes −8°, but the horn screws on the coxa are aligned with its arm. The model uses 0° for now (decided), to be calibrated on the real servos.

**Servo model.** No MG996R model has been supplied, so the servos are our own boxes from datasheet dimensions (40.7 x 19.7 mm body, 54 mm across the flange). The shaft positions are measured; the body height on the shaft side (8 mm) is ESTIMATED.

## Servo: MG996R

| Parameter | Value | Label | Notes |
|---|---|---|---|
| Mass | 55 g | DATASHEET | Consistent across sources |
| Stall torque | 11 kgf·cm (1.08 N·m) at 6 V; 9.4 kgf·cm at 4.8 V | DATASHEET | Used for `effort`. Some sellers claim 13 kgf·cm |
| Speed | 0.14 s/60° (7.48 rad/s) at 6 V; 0.17 s/60° at 4.8 V | DATASHEET | Used for `velocity`. No load, so optimistic |
| Running current | 0.5–0.9 A at 6 V | DATASHEET | |
| Stall current | 2.5 A at 6 V | DATASHEET | One seller says 1.4 A |
| Travel | 180° | From the brief | Not independently confirmed |
| Voltage range | 4.8–7.2 V | DATASHEET | One seller says 6.6 V maximum. Never connect 3S directly |

These come from seller and community pages ([components101](https://components101.com/motors/mg996r-servo-motor-datasheet), [Tinkered](https://www.tinkered.ai/components/servo-mg996r-straight), [ESPBoards](https://www.espboards.dev/sensors/mg996r/)), not an original TowerPro datasheet, which I couldn't obtain. MG996R clones vary, so measure stall current on your own servos.

## Joint limits

From [`sweep_limits.py`](../scripts/sweep_limits.py): 1° mesh collision steps with printed parts and servo bodies. Horns, screws and wires are not included, so real limits may be tighter.

| Joint | Limit | Mesh contact | Servo travel window | Deciding factor |
|---|---|---|---|---|
| Coxa | ±55° | Centre legs +59 / −84° (mirrored on the left); corner legs about ±93–101° | ±90° | Femur hits the body on centre legs |
| Femur | −54° to +95° | +97° (coxa); −61 to −76° (foot on body, at standby tibia) | −54.8 to +125.2° | Servo travel below, coxa above |
| Tibia | −90° to +68° | −96° (coxa); +71° (femur) | −111.4 to +68.6° | IK below (see next line), servo travel above |

The tibia stops at −90° although contact is at −96°, because below −90° the closed-form IK has two answers.

**Limits that depend on other joints** (a URDF can't express these):

- **Neighbouring legs.** Centre and corner legs touch when each swings 30–33° toward the other. Front pair 60–64°, rear pair 63–66°, both beyond the coxa limit. `leg_collision` catches this at runtime.
- **Tibia range changes with femur.** The tibia's lower limit is −47° at femur −55°, −80° at femur 0°, −94° at femur 30° and −64° at femur 90°.
- **Foot against body.** Low femur with a folded tibia hits the body (for example femur −39° at tibia −60°). The runtime check does not cover this.

**Capsules.** Femur 32 mm, upper tibia 20 mm (to 95 mm below the knee) and lower tibia 4 mm, fitted by [`calibrate_leg_collision.py`](../scripts/calibrate_leg_collision.py) on 8,000 random close poses. On held-out poses it missed 9 of 1,024 contacts (shallow, under 4.4 mm of capsule gap) and gave 4.6% false alarms.

**Poses** (ours, not published):

| Pose | Coxa | Femur | Tibia | Foot relative to hip |
|---|---|---|---|---|
| neutral | 0° | 0° | 0° | 103 mm out, 133 mm down |
| standby | 0° | +46.2° | −30.8° | 120 mm out, 85 mm down; body underside 82 mm up, bars 40 mm up |
| laydown | 0° | +77.0° | −38.2° | 140 mm out; body resting on the battery bars |

## Masses (ESTIMATED)

Printed parts: PLA 1.24 g/cm³, 3 walls (1.2 mm shell) and 10% infill, with volume and area MEASURED from each STL. Run `generate_model.py -v` for every part.

| Item | Each (g) | Qty | Where | Source |
|---|---|---|---|---|
| Frame | 81.7 | 1 | body | STL |
| Bottom cover | 41.6 | 1 | body | STL |
| Top cover | 27.6 | 1 | body | STL |
| Battery bar | 3.6 | 2 | body | STL. The guide's parts list says 1; the project's drawing shows 2 |
| Coxa | 13.0 | 6 | coxa | STL |
| Femur | 18.2 | 6 | femur | STL |
| Tibia + shield | 12.3 + 16.5 | 6 | tibia | STL |
| Foot switch parts (holder, rod, limitter) | about 5 | 6 | tibia | STL |
| Idler + ring | 2.1 | 18 | with each servo | STL |
| MG996R | 55 | 18 | 6 body, 6 coxa, 6 tibia | DATASHEET |
| M2 x 8 screws | 0.2 | about 230 | all | ESTIMATED from holes |
| Microswitch, rubber cap, leg wiring | 14 | 6 | tibia and foot | ESTIMATED |
| LiPo 3S 2200 mAh | 185 | 1 | under body | ESTIMATED, typical pack |
| 6 V 20 A BEC | 60 | 1 | under body | ESTIMATED, none chosen |
| Pi 5 + Active Cooler | 46 + 30 | 1 | body | ESTIMATED. Raspberry Pi publishes neither mass |
| Pi tray | 20 | 1 | body | ESTIMATED, not designed |
| ESP32 | 10 | 1 | under body | ESTIMATED |
| PCA9685 | 10 | 2 | body | ESTIMATED |
| 5 V 5 A buck | 25 | 1 | body | ESTIMATED |
| IMU | 3 | 1 | body | ESTIMATED |
| Servo power relay | 35 | 1 | body | DATASHEET ([JD2912](https://www.noco-evco.com/images/products/JD2912/JD2912_datasheet.pdf)) |
| Main wiring, fuse | 50 | 1 | body | ESTIMATED |

| Link | Mass (g) |
|---|---|
| base_link | 997 |
| coxa (with servo) | 72 |
| femur | 20 |
| tibia (with servo) | 107 |
| foot | 1 |
| **One leg** | **199** |
| **Robot** | **2192** |

Centre of mass at standby: (0.3, −0.8, 10.4) mm in `base_link`. **Weigh the robot when built.**

## Fitting the electronics

[`check_bays.py`](../scripts/check_bays.py) tests boxes against the real meshes. It fits with **no shell change**, but not in one layer: an Arduino Mega fits on the floor alone, whereas these boards total far more.

| Item | Size (mm) | Where |
|---|---|---|
| 2 x PCA9685 | 62.5 x 25.4 x 16 | On the bottom cover floor |
| 5 V 5 A buck | 45 x 25 x 15 | On the floor |
| IMU | 20.3 x 15.6 x 3.2 | On a new tray at the body centre, under the Pi |
| Pi 5 + Active Cooler | 85 x 56; top at z = 27.6 mm | On the tray, on 5 mm standoffs. Fan headroom is open air only while the top is uncovered |
| LiPo | 105 x 34 x 24 | Under the body, through the battery bars |
| ESP32 | 51 x 28 x 13 | Under the body, in the pocket beside the LiPo |
| 6 V BEC | up to 50 x 28 x 20 | Under the body, in the other pocket |
| Servo power relay | 28 x 28 x 25 | On the top cover's rear bar. See [`power_budget.md`](power_budget.md) |

Sources for the Pi: official [mechanical drawing](https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf) and [Active Cooler drawing](https://datasheets.raspberrypi.com/cooling/raspberry-pi-active-cooler-mechanical-drawing.pdf). Sizes for the other boards are typical values, so check them against what you buy.

**Limits of this layout:**

- The Pi needs a new printed tray (not a shell change, but it isn't designed yet).
- The ports top out at z = 29.9 mm and the fan headroom at 35.6 mm, above the top cover (28.2 mm). A flat lid or phone on top would need about 8 mm of spacers.
- The ESP32 and BEC sit exposed under the body, about 40 mm above the ground at standby.
- A larger BEC, or the 2–3 regulators you mentioned, would not fit in the pocket and needs a printed mount.
- The floor boards have no clearance for servo-connector bending in this check. Check with the real wires.

## Torque and power

See [`power_budget.md`](power_budget.md). At standby, holding a tripod uses 51% of femur stall torque.

## Foot contact

Each foot has one 3-pin microswitch, as in the original design.

**Mechanism, from the parts.** In `ms-holder`, two guides at z = 8–10 and 15–17 (MEASURED) carry the rod. Two side plates 12 mm apart at z = 25–34 form the switch pocket, directly above the rod. When the foot presses, the rod lifts the limitter into the switch. There is no screw mount: the pocket is plain, so glue is likely. The tip can rise only 1.0 mm before it meets the holder (MEASURED), which also means the loaded leg is about 1 mm shorter than the 133.15 mm in the model.

**In the URDF.** Each `leg_N_foot_link` is kept as a separate link and carries a Gazebo contact sensor on its collision. Topic: `/foot_contact/leg_N`. Tested in Gazebo Sim 8 with a drop onto a plane: all six reported contact between their own foot and the ground. A contact sensor detects any touch, whereas the real switch needs enough force to move the rod.

**Parameters for step 4** (ESTIMATED, in `foot_contact` in the YAML). The simulated switch should apply them to the sensor's contact force:

| Parameter | Default | Basis |
|---|---|---|
| `sim_force_threshold` | 1.5 N | Middle of [KW11 force codes](https://www.switch-china.com/kw11-micro-switch-series-product/) of 0.75, 1.5 and 2.5 N. Your switch's value is not measured |
| `sim_release_force` | 0.5 N | Assumed, so that the signal doesn't chatter |
| `sim_delay` | 10 ms | About 5 ms debounce plus 5 ms sampling |

Rod friction is not included. Weigh down a real foot to measure the actual trigger force.

**On the ESP32**, from `foot_contact` in the YAML. These pins avoid the strapping pins and the input-only pins that have no pull-ups. GPIO 21/22 stay free for I2C (PCA9685 and IMU) and 16/17 for the Pi's UART.

| Leg | Name | Project name | ESP32 GPIO |
|---|---|---|---|
| 1 | front right | R1 | 18 |
| 2 | centre right | R2 | 19 |
| 3 | rear right | R3 | 23 |
| 4 | front left | L1 | 25 |
| 5 | centre left | L2 | 26 |
| 6 | rear left | L3 | 27 |

**Wiring:** switch C to the GPIO with `INPUT_PULLUP`, NO to GND, NC unused. Pressed reads LOW. Debounce in firmware (about 5 ms). A broken wire reads as "no contact", so check all six in the startup test. This differs from the original, which drives the signal from NC (GND) and NO (3V3).

## Known gaps

- **MG996R model not supplied.** Servo visuals are placeholder boxes.
- **Foot is rigid in the model.** The real rod moves 1 mm, and the guide adds an 8 mm rubber cap whose effect on length isn't modelled.
- **Coxa servo centre:** to calibrate on real servos.
- **Joint friction, damping and servo response are not set.** They need measuring.
- **No IMU chosen.** `imu_link` is a placeholder position.
