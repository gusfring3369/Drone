#!/usr/bin/env bash
# Stop every simulation process started by run_all.sh (or by hand).
pkill -f "scripts/mission.py"   2>/dev/null
pkill -f "scripts/live_map.py"  2>/dev/null
pkill -f sim_vehicle            2>/dev/null
pkill -f arducopter             2>/dev/null
pkill -f mavproxy               2>/dev/null
pkill -f "gz sim"               2>/dev/null
echo "All simulation processes stopped."
