# Hexapod workspace

ROS 2 Jazzy description of an 18-DOF hexapod with MG996R servos, an ESP32 with 2 x PCA9685, and a Raspberry Pi 5. It is the shared robot model for a classical kinematics controller and a reinforcement learning policy. The previous robot, a RookiDroid Mochi, is at git tag `mochi-v1`.

## Model files are not in this repo

The robot's STL files are for personal use, so they and every mesh derived from them are in `.gitignore` and must never be committed:

- `refs/emre_hexapod/`: Emre Kalem's hexapod, MakerWorld 1973965, under MakerWorld's Standard Digital File License (personal use only, no redistribution, including remixes).
- `refs/mg996r/`: MG996R model from a GrabCAD user upload.
- `src/hexapod_description/meshes/emre/` and `docs/images/`: meshes and renders generated from them.

Our own code and the numbers measured from the parts are committed. To get the meshes on a fresh clone, get the STL files yourself, put them in `refs/emre_hexapod/`, and run `generate_model.py` as shown under [Regenerate meshes and parameters](#regenerate-meshes-and-parameters).

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

It needs the generated meshes. It listens on this machine only. Add `--host 0.0.0.0` to allow other machines, and `--port` to change the port.

To create screenshots of each mode in `docs/images/` (not committed, because they show the meshes), run `scripts/capture_sim_screenshots.py`.

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

`refs/` is not tracked. `makeyourpet_hexapod` is only needed for the cross-check against its published config.

```bash
git clone https://github.com/makeyourpet/hexapod refs/makeyourpet_hexapod
python3 -m venv .venv && .venv/bin/pip install -r scripts/requirements.txt -e src/hexapod_kinematics -e src/hexapod_sim

.venv/bin/python scripts/generate_model.py            # write meshes/emre and the YAML
.venv/bin/python scripts/measure_geometry.py          # compare the model with the STLs
.venv/bin/python scripts/sweep_limits.py              # joint limits from mesh contact
.venv/bin/python scripts/calibrate_leg_collision.py   # refit leg collision capsules
.venv/bin/python scripts/check_bays.py                # do the electronics fit?
.venv/bin/python scripts/power_budget.py              # torque margin, regulators, battery
```

`generate_model.py` overwrites the YAML, so change values in the script rather than in the YAML. Limits from the sweep and the capsule radii are entered in the script by hand.

## Licence

Our code is GPL-3.0. The STL and MG996R files are not ours and are not covered by it: see [Model files are not in this repo](#model-files-are-not-in-this-repo).
