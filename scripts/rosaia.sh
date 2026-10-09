#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"

source_ros() {
  # ROS setup scripts legitimately reference variables that may be unset.
  set +u
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    source "$WORKSPACE/install/setup.bash"
  fi
  set -u
}

usage() {
  sed -n '/^Commands:/,$p' <<'EOF'
Usage: scripts/rosaia.sh COMMAND [ARGS]

Commands:
  build                 Build all ROS 2 packages with symlink install
  test                  Run package tests and print their results
  bridge                Start the Bluno serial bridge
  velocity-controller   Start the encoder-velocity PWM controller
  vision                Start the RealSense RGB-D ArUco tracker
  recorder [NAME]       Record synchronized CSV data under data/sessions
  audit [ARGS]          Audit recorded sessions before MLP training
  excitation [ARGS]     Generate a non-actuating step/Fourier reference CSV
  profile [ARGS]        Execute a return-home encoder-velocity profile
  sweep [ARGS]          Execute a quasi-static firmware-bounded encoder sweep
  train [ARGS]          Train one MLP from complete recorded sessions
  jacobian [ARGS]       Publish learned Jacobians from encoder positions
  controller            Start the non-actuating shape controller
  dashboard             Open the multimodal web dashboard backend
  view                  Open the annotated image with rqt_image_view
  status                Show devices, relevant nodes and topic rates
  stop                  Publish a zero command once
  step AXIS MOTOR_COUNTS PWM
                        Send one bounded move; sign selects motor direction
  preview               Run the direct single-camera ArUco preview
  workflow [ARGS]       Bring-up, calibrate/train, and open the demo
  position-test [ARGS]  Test controlled q target and automatic return home
  dual-position-test    Move both q targets together and return home
EOF
}

command="${1:-help}"
shift || true

case "$command" in
  build)
    source_ros
    cd "$WORKSPACE"
    colcon build --symlink-install
    ;;
  test)
    source_ros
    cd "$WORKSPACE"
    colcon test --event-handlers console_direct+
    colcon test-result --verbose
    ;;
  bridge)
    source_ros
    exec ros2 launch bluno_motor_bridge bluno_motor_bridge.launch.py "$@"
    ;;
  velocity-controller)
    source_ros
    exec ros2 launch bluno_motor_bridge encoder_velocity_controller.launch.py "$@"
    ;;
  vision)
    source_ros
    exec ros2 launch aruco_shape_tracker aruco_shape_tracker.launch.py "$@"
    ;;
  recorder)
    source_ros
    session_name="${1:-$(date +%Y%m%d_%H%M%S)}"
    cd "$REPO_ROOT"
    exec ros2 launch rosaia_data_acquisition session_recorder.launch.py \
      session_name:="$session_name" output_root:="$REPO_ROOT/data/sessions"
    ;;
  audit)
    source_ros
    cd "$REPO_ROOT"
    exec ros2 run rosaia_learning audit_sessions "$@"
    ;;
  excitation)
    source_ros
    cd "$REPO_ROOT"
    exec ros2 run rosaia_learning generate_excitation "$@"
    ;;
  profile)
    source_ros
    exec ros2 run rosaia_learning play_velocity_profile "$@"
    ;;
  sweep)
    cd "$REPO_ROOT"
    exec "$REPO_ROOT/scripts/run_quasistatic_sweep.sh" "$@"
    ;;
  train)
    source_ros
    cd "$REPO_ROOT"
    exec ros2 run rosaia_learning train_mlp "$@"
    ;;
  jacobian)
    source_ros
    exec ros2 launch rosaia_learning jacobian_publisher.launch.py "$@"
    ;;
  controller)
    source_ros
    exec ros2 launch rosaia_shape_control shape_controller.launch.py "$@"
    ;;
  dashboard)
    source_ros
    exec ros2 launch rosaia_dashboard dashboard.launch.py "$@"
    ;;
  view)
    source_ros
    if ! command -v rqt_image_view >/dev/null 2>&1; then
      echo "rqt_image_view is not installed (package: ros-jazzy-rqt-image-view)." >&2
      exit 2
    fi
    exec rqt_image_view /aruco/annotated/compressed
    ;;
  status)
    source_ros
    echo "Serial device:"
    ls -l /dev/serial/by-id/usb-2341_0043-if00 2>/dev/null || echo "  not found"
    echo "RGB devices:"
    v4l2-ctl --list-devices 2>/dev/null || echo "  v4l2-ctl unavailable"
    echo "Relevant ROS nodes:"
    ros2 node list 2>/dev/null | grep -E 'bluno|aruco|shape|recorder' || true
    echo "Relevant ROS topics:"
    ros2 topic list 2>/dev/null | grep -E 'motor|encoder|aruco|arm_|shape' || true
    ;;
  stop)
    source_ros
    ros2 topic pub --once --wait-matching-subscriptions 0 \
      /motor_command std_msgs/msg/Int16MultiArray \
      '{data: [0, 0]}'
    ;;
  step)
    source_ros
    if [[ $# -ne 3 ]]; then
      echo "Usage: scripts/rosaia.sh step AXIS MOTOR_SIGNED_COUNTS PWM" >&2
      exit 2
    fi
    ros2 topic pub --once /motor_step_command std_msgs/msg/Int16MultiArray \
      "{data: [$1, $2, $3]}"
    ;;
  preview)
    cd "$REPO_ROOT"
    exec python3 scripts/single_aruco_preview.py "$@"
    ;;
  workflow)
    source_ros
    cd "$REPO_ROOT"
    exec python3 "$REPO_ROOT/scripts/rosaia_workflow.py" "$@"
    ;;
  position-test)
    cd "$REPO_ROOT"
    exec python3 "$REPO_ROOT/scripts/run_controlled_position_test.py" "$@"
    ;;
  dual-position-test)
    cd "$REPO_ROOT"
    exec python3 "$REPO_ROOT/scripts/run_dual_position_test.py" "$@"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    echo "Unknown command: $command" >&2
    usage >&2
    exit 2
    ;;
esac
