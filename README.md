# ROSAIA LEGO — two single-actuator continuum arms

This repository is the ROS 2 Jazzy starting point for controlling two
deformable arms. Each arm has one primary motor; the antagonistic motor used by
the original ROSAIA platform is not part of this system.

The current implementation contains the motor/encoder/current bridge, the
visual-state frontend, a timestamp-aware experiment recorder, and the first
non-actuating resolved-rate controller. Jacobian identification and learned
control remain separate stages so they can be validated by offline replay
before actuation.

## Current architecture

```text
/motor_command (Int16MultiArray: [arm_1_primary, arm_2_primary])
                         |
                         v
              bluno_motor_bridge
                         |
                  USB serial ASCII
                         |
                         v
        Bluno + Makerfabs H-Bridge shield
             axis 0 -> M2
             axis 1 -> M1

RealSense RGB-D -> aruco_shape_tracker
           -> rosaia_data_acquisition (CSV sessions)
           -> /arm_1/aruco_state, /arm_2/aruco_state
           -> rosaia_learning MLP + analytic J(q)
           -> rosaia_shape_control
           -> /arm_N/encoder_velocity_reference
```

The detailed, provisional hardware contract is in
[`CONTROL_BOARD_BLUNO_MAKERFABS_ROS2_JAZZY.md`](CONTROL_BOARD_BLUNO_MAKERFABS_ROS2_JAZZY.md).
It is a living document: physical measurements take precedence and changes to
the wiring must be recorded there.

The operational commands are collected in the standalone
[`docs/operator_guide.html`](docs/operator_guide.html) guide. Calibration,
dataset design, model selection, and validation are specified in
[`docs/TRAINING_PIPELINE.md`](docs/TRAINING_PIPELINE.md).

## Workspace

The ROS workspace is in `ros2_ws/`. Build it with:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The checked-in configurations use the USB and RGB device paths verified during
bring-up. Before connecting motor power, review
`ros2_ws/src/bluno_motor_bridge/config/controller.yaml` and follow the bring-up
log.

Run the bridge after configuration:

```bash
ros2 launch bluno_motor_bridge bluno_motor_bridge.launch.py
```

Run the visual frontend and the non-actuating resolved-rate controller with:

```bash
ros2 launch aruco_shape_tracker aruco_shape_tracker.launch.py
ros2 launch rosaia_shape_control shape_controller.launch.py
```

Record a named synchronized experiment session with:

```bash
scripts/rosaia.sh recorder calibration_01
```

Generate a controlled quasi-static dataset using firmware-bounded encoder
steps with `scripts/rosaia.sh sweep`. The recorder and sweep run in separate
terminals so the raw data stream starts before actuation.

The helper `scripts/rosaia.sh` also provides build, test, bridge, vision,
excitation generation, MLP training, learned-Jacobian inference, status,
preview, visualization, and stop commands.

After placing both arms at mechanical home, the complete two-arm calibration
and concept-demonstration workflow is available as:

```bash
scripts/rosaia.sh workflow start --enable-motion
scripts/rosaia.sh workflow calibrate --label demo_5v6
scripts/rosaia.sh workflow demo
```

The calibration command resets encoder and camera references, records separate
training and validation sweeps, audits them, trains both MLPs, hot-reloads the
dashboard, and returns each arm home after its sweep. Keep the camera and base
markers fixed for the resulting models.

Open the live two-arm dashboard with:

```bash
scripts/rosaia.sh dashboard
```

It displays ArUco shapes, encoders, currents and applied PWM at
`http://127.0.0.1:8765`. Goal publication is disabled by default. Candidate
models can be loaded for non-actuating reachable-shape projection with:

```bash
scripts/rosaia.sh dashboard \
  arm_1_model:=$PWD/data/models/arm1_candidate \
  arm_2_model:=$PWD/data/models/arm2_candidate
```

The shape controller does not publish `motor_command`; it produces bounded
encoder-velocity references for the future low-level velocity loop.

For a low-level command (only after completing the safety checks):

```bash
ros2 topic pub --once /motor_command std_msgs/msg/Int16MultiArray \
  "{data: [0, 0]}"
```

## Safety boundary

- Startup and stale commands resolve to `STOP`.
- Both motor axes are enabled in the current firmware and ROS configuration.
- Firmware independently enforces its watchdog and provisional encoder limits.
- The Bluno firmware watchdog remains mandatory; host-side checks do not
  replace it.
- Encoder CPR, gearbox ratio, current-sensor sensitivity, current limits, and
  physical signs are intentionally not guessed.
