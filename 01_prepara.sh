#!/usr/bin/env bash
# Prepare the workspace on Ubuntu 24.04 with ROS 2 Jazzy.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == --help ]]; then
  echo 'Usage: bash 01_prepara.sh [--test]'
  exit 0
fi
[[ $# -eq 0 || ( $# -eq 1 && $1 == --test ) ]] || exit 2
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'ROS 2 Jazzy is required.' >&2; exit 1; }
if ! dpkg-query -W -f='${Status}' librealsense2-dev 2>/dev/null | grep -q 'install ok installed'; then
  echo 'Missing build dependency. Run: sudo apt-get install librealsense2-dev' >&2
  exit 1
fi
bash "$repo/scripts/rosaia.sh" build
if [[ ${1:-} == --test ]]; then
  bash "$repo/scripts/rosaia.sh" test
  python3 -m pytest -q "$repo/scripts/test_rosaia_workflow.py"
fi
echo 'Workspace ready. Start with: bash 02_avvia.sh'
