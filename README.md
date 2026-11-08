# Hexapod workspace

ROS 2 Jazzy description of an 18-DOF hexapod, based on the RookiDroid Mochi (MG92B servos). It is the shared robot model for a classical kinematics controller and a reinforcement learning policy.

All geometry, masses and limits live in [`robot_parameters.yaml`](src/hexapod_description/config/robot_parameters.yaml). Sources and estimates are listed in [`docs/robot_parameters.md`](docs/robot_parameters.md).

## Build and run

Needs Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. [`scripts/install_deps.sh`](scripts/install_deps.sh) installs them.

```bash
cd ~/hexapod_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch hexapod_description display.launch.py
```

This opens RViz and a slider for each joint. Use `gui:=false` to drive `/joint_states` from another node.

## Simulator

A browser simulator with four modes: forward kinematics, body pose IK, leg patterns and gait playback. It needs no ROS. One-time setup:

```bash
python3 -m venv .venv
.venv/bin/pip install -e src/hexapod_kinematics -e src/hexapod_sim
```

Then run it and open http://localhost:8080:

```bash
.venv/bin/python -m hexapod_sim
```

It listens on this machine only. Add `--host 0.0.0.0` to allow other machines, and `--port` to change the port.

| Forward kinematics | Body pose IK |
|---|---|
| ![Forward kinematics mode with a tibia limit warning](docs/images/mode_fk.png) | ![Body pose IK mode](docs/images/mode_body.png) |

| Leg patterns | Gait playback |
|---|---|
| ![Leg patterns mode at the laydown pose](docs/images/mode_pattern.png) | ![Gait playback mode, tripod gait](docs/images/mode_gait.png) |

| Unreachable feet and limits | Servo speed |
|---|---|
| ![Red warnings for unreachable feet and joint limits](docs/images/warning_unreachable.png) | ![Red warnings for servo speed](docs/images/warning_speed.png) |

## Kinematics library

[`hexapod_kinematics`](src/hexapod_kinematics) is pure Python with numpy and PyYAML, and has no ROS imports. It reads all robot values from `robot_parameters.yaml`. The maths and gait parameters are in [`docs/kinematics.md`](docs/kinematics.md).

```python
from hexapod_kinematics import RobotModel, GaitParams, VelocityCommand, generate_gait, body_pose_ik, write_csv

model = RobotModel.load()
pose = body_pose_ik(model, translation=(0.01, 0, 0), rpy=(0, 0.05, 0))
print(pose.q, pose.issues)

gait = generate_gait(model, GaitParams(gait='tripod'), VelocityCommand(vx=0.015))
print(gait.summary())
write_csv(model, gait, 'tripod.csv')
```

To export a gait from the command line:

```bash
.venv/bin/python -m hexapod_kinematics --gait ripple --vx 0.01 --cycles 3 -o ripple.csv
```

The CSV has time, 18 joint angles, world-frame foot positions, foot contacts, body pose and stability margin, in SI units and radians.

## Tests

```bash
.venv/bin/pip install pytest
.venv/bin/python -m pytest src/hexapod_kinematics src/hexapod_sim

# Operate the real UI in headless Chrome and refresh docs/images (needs Google Chrome):
.venv/bin/pip install playwright
.venv/bin/python scripts/capture_sim_screenshots.py
```

`colcon` does not build `hexapod_kinematics` or `hexapod_sim`, because they are plain Python packages. Under ROS, make the library importable with `export PYTHONPATH=$HOME/hexapod_ws/src/hexapod_kinematics:$PYTHONPATH`.

## Check the model

```bash
xacro src/hexapod_description/urdf/hexapod.urdf.xacro > /tmp/hexapod.urdf && check_urdf /tmp/hexapod.urdf

# All 18 joints: run the launch with gui:=false rviz:=false in another terminal, then:
python3 scripts/verify_joints.py
```

## Regenerate meshes and parameters

Only needed if the CAD or the estimates change. `refs/` is not tracked.

```bash
git clone https://github.com/rookidroid/hexapod refs/rookidroid_hexapod
git -C refs/rookidroid_hexapod checkout 625d150
python3 -m venv .venv && .venv/bin/pip install -r scripts/requirements.txt

.venv/bin/python scripts/measure_geometry.py   # compare the model with the STLs
.venv/bin/python scripts/generate_model.py     # write the meshes and the YAML
.venv/bin/python scripts/sweep_limits.py       # collision sweep, takes about 11 minutes
.venv/bin/python scripts/calibrate_leg_collision.py   # refit leg collision capsules (needs hexapod_kinematics)
```

`generate_model.py` overwrites the YAML, so change values in the script rather than in the YAML. Limits from the sweep and the capsule radius are entered in the script by hand.

## Licence

GPL-3.0, because the meshes derive from RookiDroid's GPL-3.0 STL files.
# hexapod_ws
