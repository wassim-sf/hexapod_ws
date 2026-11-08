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
```

`generate_model.py` overwrites the YAML, so change values in the script rather than in the YAML. Limits from the sweep are entered in the script by hand.

## Licence

GPL-3.0, because the meshes derive from RookiDroid's GPL-3.0 STL files.
