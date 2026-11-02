#!/usr/bin/env bash
# Start the whole simulation in separate terminal windows (Ubuntu desktop / GNOME).
#
#   ./scripts/run_all.sh                 # scenario "long"  (drone re-plans around the blockage)
#   ./scripts/run_all.sh short           # scenario "short" (blockage clears, drone resumes)
#   ./scripts/run_all.sh none            # no blockage
#   ./scripts/run_all.sh long --fresh    # also wipe saved ArduPilot params (-w); use after
#                                        # "PreArm: Motors: Check frame class and type"
#
# Stop everything with ./scripts/stop_all.sh
set -u
SCENARIO="${1:-long}"
case "$SCENARIO" in long|short|none) ;; *) echo "scenario must be long, short or none"; exit 1;; esac
FRESH=""
for a in "$@"; do [ "$a" = "--fresh" ] && FRESH="-w"; done

REPO="$(cd "$(dirname "$0")/.." && pwd)"
AP="${ARDUPILOT_HOME:-$HOME/ardupilot}"
PARAMS="--add-param-file=$AP/Tools/autotest/default_params/copter.parm --add-param-file=$AP/Tools/autotest/default_params/gazebo-iris.parm"

if ! command -v gnome-terminal >/dev/null; then
  echo "gnome-terminal not found - start the 4 commands from README.md manually."; exit 1
fi
for f in "$AP/Tools/autotest/default_params/gazebo-iris.parm" "$REPO/worlds/medical_delivery.sdf"; do
  [ -f "$f" ] || { echo "Missing $f (is ArduPilot in $AP? set ARDUPILOT_HOME otherwise)"; exit 1; }
done

"$REPO/scripts/stop_all.sh" >/dev/null 2>&1
sleep 1
rm -f "$REPO/logs/state.json"

# bash -i so ~/.bashrc is loaded (PATH for sim_vehicle.py, GZ_SIM_* variables, ROS)
term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -ic "$*; echo; echo '[$title stopped - press Enter to close]'; read"
}

echo "[1/4] Gazebo world"
term "1 Gazebo" "cd '$REPO' && gz sim -v4 -r worlds/medical_delivery.sdf --render-engine ogre"
sleep 10
echo "[2/4] ArduPilot SITL ${FRESH:+(fresh parameters)}"
term "2 ArduPilot SITL" "cd ~ && sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map $FRESH $PARAMS --out=udp:127.0.0.1:14560"
sleep 3
echo "[3/4] Live map"
term "3 Live map" "cd '$REPO' && python3 scripts/live_map.py"
sleep 2
echo "[4/4] Mission (scenario: $SCENARIO) - it waits for ArduPilot to be ready by itself"
term "4 Mission" "cd '$REPO' && python3 scripts/mission.py --scenario $SCENARIO"
echo "Started. Results will be in $REPO/logs/ when the mission completes."
