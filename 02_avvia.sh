#!/usr/bin/env bash
# Default startup displays live state without enabling dashboard motor actions.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
case "${1:-start}" in
  start)
    bash "$repo/scripts/rosaia.sh" workflow start
    echo 'Dashboard: http://127.0.0.1:8765'
    ;;
  motion)
    bash "$repo/scripts/rosaia.sh" workflow start --enable-motion
    echo 'Dashboard: http://127.0.0.1:8765 - motor actions enabled'
    ;;
  stop) bash "$repo/scripts/rosaia.sh" workflow stop ;;
  status) bash "$repo/scripts/rosaia.sh" status ;;
  --help|-h)
    echo 'Usage: bash 02_avvia.sh [start|motion|stop|status]'
    ;;
  *) echo 'Unknown action. Use --help.' >&2; exit 2 ;;
esac
