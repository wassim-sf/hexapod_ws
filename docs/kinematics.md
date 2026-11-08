# Kinematics and gaits

How `hexapod_kinematics` works. Robot values come from [`robot_parameters.yaml`](../src/hexapod_description/config/robot_parameters.yaml); joint signs are defined in [`robot_parameters.md`](robot_parameters.md#frames-and-joint-signs).

## Leg forward kinematics

Each leg frame has its origin at the hip, x outward at zero coxa angle, and z up. With coxa, femur and tibia lengths `c`, `f`, `t`, and joint angles `q1`, `q2`, `q3`:

```
radial = c + f·cos(q2) + t·sin(q2 + q3)
up     =     f·sin(q2) − t·cos(q2 + q3)

x = radial·cos(q1)
y = radial·sin(q1)
z = up
```

To get base_link coordinates, rotate by the leg's yaw and add the hip position: `body_to_leg` and `leg_to_body` do this for one point, and `feet_fk` for all six legs.

## Closed-form inverse kinematics

Given a foot at `(x, y, z)` in the leg frame:

1. **Coxa.** `q1 = atan2(y, x)` and `radial = hypot(x, y)`. If `x < 0` the foot is behind the coxa axis, so use `q1 = atan2(−y, −x)` and a negative radial. This keeps `|q1| ≤ 90°`, and the joint limit check then decides whether the pose is allowed.
2. **Reduce to two links.** Let `a = radial − c` and `b = z`. The femur and tibia must reach `(a, b)` from the femur axis, at distance `D = hypot(a, b)`.
3. **Reachability.** A solution exists only if `|f − t| ≤ D ≤ f + t`. If not, the foot is reported as unreachable with its distance from the workspace, and the angles returned are for the nearest reachable point in that direction.
4. **Elbow.** The tibia direction is `q2 + q3 − 90°` measured from the femur's zero, so the elbow angle between femur and tibia is `e = q3 − 90°`. The law of cosines gives `cos(e) = (D² − f² − t²) / (2·f·t)`.
5. **Choose the branch.** With the tibia limited to between −90° and +90°, `e` is always between −180° and 0°, so `e = −acos(…)` is the only branch inside the limits. Then `q3 = e + 90°`. A robot's YAML must keep the tibia within that range for IK to be single-valued.
6. **Femur.** `q2 = atan2(b, a) − atan2(t·sin(e), f + t·cos(e))`.

IK then FK agrees within 0.01 mm over 20,000 random points. A foot exactly on the coxa axis has no defined `q1`; `atan2` returns 0 there.

## Body pose IK

`body_pose_ik` keeps the feet fixed in the world and moves the body. With rotation `R = Rz(yaw)·Ry(pitch)·Rx(roll)` and body position `p`, each foot in the body frame is:

```
foot_body = Rᵀ·(foot_world − p)
```

Each foot then goes through the leg IK. Zero translation and rotation means the standby pose, with the feet on `z = 0` and base_link at the stance height.

## Checks

Each check returns `Issue` objects with a kind and a readable message.

| Kind | Triggered when |
|---|---|
| `unreachable` | Foot outside the workspace, with the distance in mm |
| `joint_limit` | Angle outside the YAML limits |
| `leg_collision` | Neighbouring leg capsules overlap |
| `servo_speed` | A joint must exceed its YAML velocity to move between two poses in the given time |
| `too_few_feet` | Fewer than 3 feet on the ground |
| `unstable` | Centre of mass is outside the support polygon |

### Leg-to-leg collision check

Each leg is modelled as three capsules on the leg's plane. Neighbouring legs, found from the ring of leg yaws, collide when any pair of capsules overlaps.

| Capsule | From | To | Radius | Source |
|---|---|---|---|---|
| Femur | Femur axis | Knee | 32 mm | Fitted to mesh contact |
| Upper tibia | Knee | 95 mm down | 20 mm | Fitted to mesh contact |
| Lower tibia | 95 mm down | Foot | 4 mm | Fitted to mesh contact |

[`calibrate_leg_collision.py`](../scripts/calibrate_leg_collision.py) compares this model with collision detection on the real meshes for 8,000 random poses where neighbouring legs are close, across all pair types. Half the poses are used to fit the radii, and half are held out.

| | Fit poses | Held-out poses |
|---|---|---|
| True contacts missed | 11 of 1039 | 9 of 1024 |
| False alarms | 127 of 2961 (4.3%) | 138 of 2976 (4.6%) |

When both legs swing toward each other, the capsules trigger within about 3° of the meshes: side pairs at 29–32° against 30–33° for the meshes, front and rear pairs at 63–67° against 60–66°.

**What this check does not cover:**

- Non-neighbouring legs.
- A foot or tibia hitting the body. For example, the foot reaches the body at femur −39° with the tibia at −60°.
- Horns, screws and wires, which are not in the meshes.

## Static stability

`center_of_mass` adds up the estimated YAML link masses at their positions from FK, mirroring lateral offsets on left legs. `stability_margin` projects it onto the ground and takes the convex hull of the stance feet. The margin is the distance to the nearest hull edge. It is positive inside the polygon, negative outside, and never positive with fewer than 3 feet.

This is a static measure. It ignores accelerations, and the masses are estimates.

## Gait generator

`GaitGenerator.update(dt, command)` advances the gait and accepts a new command each call, like `/cmd_vel`. `generate_gait` runs it at a constant command and checks every sample.

- **Stance.** A stance foot is fixed in the world. In the body frame it follows the exact inverse of the body's motion under a constant twist, so it doesn't slip even when turning.
- **Swing.** The foot moves from its lift-off point to a touchdown point chosen so that it passes its home position halfway through the next stance. Horizontal motion uses a minimum-jerk blend, and height follows `step_height·sin²(πτ)`, so the foot leaves and lands with zero vertical speed.
- **Idle.** With a zero command, swinging feet finish their step and then all feet stay down.

### Parameters

| Parameter | Meaning | Default |
|---|---|---|
| `step_length` | Largest foot travel during one stance | 30 mm |
| `step_height` | Swing foot clearance | 15 mm |
| `cycle_period` | Time for one full cycle | Tripod 2 s, ripple 3 s, wave 6 s |
| `duty_factor` | Fraction of the cycle a foot is on the ground | Tripod 0.55, ripple 0.70, wave 0.85 |

The body speed and the stride are linked by `stride = speed × period × duty`. A command that would exceed `step_length` at any foot is scaled down, keeping its direction, and the sample is marked `saturated`. So the fastest straight-line speed is `step_length / (period × duty)`.

### Gaits

| Gait | Pattern | Feet down at minimum |
|---|---|---|
| Tripod | Legs 1, 3, 5 alternate with legs 2, 4, 6 | 3 |
| Ripple | Each side steps rear to front, with the sides half a cycle apart | 4 |
| Wave | One leg at a time: right side rear to front, then left side | 5 |

Duty factor sets the feet down, so lowering it reduces those numbers. A tripod below 0.5 reports `too_few_feet`.

### Results at the default parameters

These are from `generate_gait` with no issues on any row. Commands above a gait's top speed are scaled down.

| Gait | Top speed | Command | Applied | Min margin | Peak joint speed (coxa, femur, tibia) |
|---|---|---|---|---|---|
| Tripod | 27.3 mm/s | Forward 15 mm/s | 15.0 mm/s | 80.7 mm | 0.28, 1.08, 0.79 rad/s |
| Tripod | | Sideways 15 mm/s | 15.0 mm/s | 83.8 mm | 0.20, 1.11, 0.86 rad/s |
| Tripod | | Turn 0.1 rad/s | 0.100 rad/s | 86.2 mm | 0.42, 1.04, 0.67 rad/s |
| Ripple | 14.3 mm/s | Forward 15 mm/s | 14.3 mm/s | 77.5 mm | 0.52, 1.15, 0.96 rad/s |
| Ripple | | Sideways 15 mm/s | 14.3 mm/s | 82.5 mm | 0.37, 1.21, 1.12 rad/s |
| Ripple | | Turn 0.1 rad/s | 0.064 rad/s | 83.3 mm | 0.52, 1.05, 0.69 rad/s |
| Wave | 5.9 mm/s | Forward 15 mm/s | 5.9 mm/s | 79.3 mm | 0.52, 1.15, 0.97 rad/s |
| Wave | | Sideways 15 mm/s | 5.9 mm/s | 83.8 mm | 0.37, 1.21, 1.12 rad/s |
| Wave | | Turn 0.1 rad/s | 0.026 rad/s | 82.8 mm | 0.52, 1.05, 0.69 rad/s |

**Why these defaults are slow.** Holding a tripod stance uses about 51% of femur stall torque. Peak joint speed is about 16% of the servo's no-load 7.48 rad/s.

**Limits of these results.** The generator does not model torque, so a gait with no warnings can still overload a servo.

## Simulator

The simulator uses Viser. It places the STL meshes directly with the library's `link_frames` instead of loading the URDF, so it doesn't need `xacro` or ROS, and the view uses the same FK as everything else. Plotly Dash wasn't needed. The meshes come from `visual.mesh_subdir` in the YAML.

- **Forward kinematics and leg patterns.** There is no ground model here, so the body stays level and the feet within 3 mm of the lowest foot count as supporting. The sliders go past the joint limits so that you can see the warnings.
- **Servo speed in slider modes.** Dragging a slider has no real timing, so the check compares the start pose (set when you enter the mode or press the button) with the current pose over the chosen move time.
- **Gait playback.** The speed check uses the 20 ms sample time.
