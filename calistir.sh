#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

echo "[abdu] Eski ABDÜ süreçleri kapatılıyor (Yavuz'a dokunulmaz)..."
pkill -TERM -f 'lib/abdu_control/inspect_mission' >/dev/null 2>&1 || true
sleep 1
pkill -9 -f 'gzserver /tmp/abdu_gazebo' >/dev/null 2>&1 || true
pkill -9 -f 'abdu_inspect_mission' >/dev/null 2>&1 || true
pkill -9 -f 'lib/abdu_control/inspect_mission' >/dev/null 2>&1 || true
pkill -9 -f 'abdu_teleop' >/dev/null 2>&1 || true
pkill -9 -f 'lib/abdu_control/teleop' >/dev/null 2>&1 || true
pkill -9 -f 'abdu.rviz' >/dev/null 2>&1 || true
pkill -9 -f 'abdu_robot_state_publisher' >/dev/null 2>&1 || true
pkill -9 -f 'spawn_entity.py -entity abdu_rov' >/dev/null 2>&1 || true
sleep 1

set +u
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.bash ]]; then
  source /usr/share/gazebo/setup.bash
fi
export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11346
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
else
  echo "Önce derleyin: colcon build --symlink-install"
  exit 1
fi
set -u

echo "[abdu] Su altı ortamı ve ROV başlatılıyor: ${*:-otonom=true rviz:=true}"

RVIZ_ARG="rviz:=true"
for arg in "$@"; do
  case "$arg" in
    rviz:=*) RVIZ_ARG="" ;;
  esac
done

ros2 launch abdu_gazebo abdu_gazebo.launch.py "$@" $RVIZ_ARG
