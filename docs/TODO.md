# TODO

## Not verified in simulation

The final step 3 launch test covers only the tripod gait on the flat world: stand, walk, turn and a 60 s walk.

- [ ] Ripple gait.
- [ ] Wave gait.
- [ ] Sit-down. `test_7_sits_down` exists but is skipped in `test_walking.launch.py`. Remove the skip when checking this.
- [ ] Fall detection. No fall happened in the test, so `fallen` and the logger's trigger have only been seen at 0.
- [ ] The other worlds: `slope_10`, `rough`, `step_2cm`, `low_friction`.
- [ ] Debug trajectory controller (`debug_trajectory:=true`).

## Small issues

- [ ] `real_time_factor: 0` through Gazebo's `/world/flat/set_physics` is accepted (`data: true`) but has no effect: the test still reports a real-time factor of 0.99. Faster-than-real-time runs need another way, for example a launch argument that writes the value into the temporary world.
- [ ] `colcon test` fails on `hexapod_controller` and `hexapod_kinematics` with `NO TESTS RAN`. The README has the commands that do run them.
- [ ] `pytest` under a sourced ROS environment fails on the `launch_testing_ros` plugin and on a missing `lark` module, unless plugin autoload is disabled.
- [ ] `foot_switch_sim.py` prints a `KeyboardInterrupt` traceback and exits with -2 on shutdown. Harmless so far.
- [ ] True speed is 8% below the command at 0.05 m/s (0.0460 m/s). The test allows 20%.
- [ ] The gait sweep (`config/gait_sweep.yaml`) was stopped before its results were written up. The gait parameters stay at their defaults.
