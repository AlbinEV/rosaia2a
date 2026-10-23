#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
case "${1:-start}" in
  start)
    bash "$repo/scripts/rosaia.sh" workflow start --without-velocity
    python3 "$repo/scripts/start_final_ui.py"
    ;;
  stop)
    python3 "$repo/scripts/start_final_ui.py" --stop
    bash "$repo/scripts/rosaia.sh" workflow stop
    ;;
  *) echo 'Uso: bash 03_demo_finale.sh [start|stop]'; exit 2 ;;
esac
