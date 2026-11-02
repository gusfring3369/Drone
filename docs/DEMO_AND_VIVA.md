# Demo video, live demo and viva preparation

## 1. Demo video (must be **under 2 minutes**)

### What to put on screen
- **Left half:** the Gazebo 3D view. The camera follows the drone automatically when `mission.py` starts.
- **Right half:** the **live map** window (`scripts/live_map.py`), which shows the route, re-plan, blockage, state and events.
- **Optional:** the Mission terminal in a corner, so viewers can read `STATE -> HOLD` and `REPLAN`.

### Script (scenario `long`)
| Time | Show | Say / caption |
|---|---|---|
| 0:00–0:12 | Gazebo world and live map before takeoff | "Q8: medical package delivery. Hospital (red pad) to remote village clinic (green pad). Grey buildings are 25–40 m tall; we cruise at 15 m, so we must fly around them." |
| 0:12–0:30 | Takeoff, blockage tower appears, drone follows the blue A\* route | "A\* plans a collision-free route on a grid with a 4 m safety margin. A temporary blockage appears on the route." |
| 0:30–0:55 | Drone stops about 25 m before the blockage, **HOLD**, sensor circle on the map | "The forward range sensor detects the blockage 25 m ahead. The drone holds position and waits up to 20 s, since the blockage may be temporary." |
| 0:55–1:15 | **REPLAN**: green detour, the drone flies around | "Still blocked after 20 s, so it re-plans with A\* from its current position, treating the blockage as an obstacle." |
| 1:15–1:35 | Lands at the village, "PACKAGE DELIVERED", takes off and flies home | "Package delivered. The return route uses what it learned and avoids the blockage if it's still there." |
| 1:35–1:55 | Final `logs/flight_map.png` and `logs/events.txt` | "Geofence and RTL at 45 m provide failsafes. Total: about 370 m, one re-plan." |

Optionally, add a 10 s clip of `--scenario short`, where the blockage clears after 8 s and the drone **resumes** its original route. Speed up the dull parts (takeoff, landing) 2× in the editor to stay under 2 minutes.

### How to record
**Easiest: on Windows, recording the VMware window**
- **Xbox Game Bar:** press **Win+Alt+R** to start and stop recording the focused window. Files go to `Videos\Captures`.
- **Or the Snipping Tool** (Windows 11): use **Record**, then select the area.

**Inside Ubuntu**
- **GNOME built-in recorder:** press **Ctrl+Shift+Alt+R** to start and stop. Files go to `~/Videos`. Some versions stop automatically after 30 s, in which case use OBS.
- **OBS Studio:**
```bash
sudo apt install obs-studio -y
```

**Trim, speed up and caption** with Clipchamp (built into Windows 11), DaVinci Resolve or Kdenlive. Export as **MP4, 1080p**.

**Where to put the video:**
- Store it in the repo as `media/demo_video.mp4`. **GitHub blocks files over 100 MB**, so compress it if needed:
```bash
ffmpeg -i in.mp4 -vcodec libx264 -crf 28 media/demo_video.mp4
```
- Or upload it to Google Drive or YouTube (unlisted) and paste the link into `media/README.md`.

## 2. Live demo (on or before 07.12.2026)
**The day before:**
- Run the whole thing once: `./scripts/run_all.sh long`, then `./scripts/stop_all.sh`, then `./scripts/run_all.sh short`.
- Plug the laptop in; on battery the GPU slows down a lot.
- Close Chrome and other heavy apps, on Windows and in the VM.

**At the demo:**
1. Start the VM, and open a terminal with Ctrl+Alt+T.
2. Run:
```bash
cd ~/Drone && ./scripts/run_all.sh long
```
3. Narrate using the table above.
4. If asked "what if the blockage goes away?", run:
```bash
./scripts/stop_all.sh && ./scripts/run_all.sh short
```

**Backup plan:** if Gazebo misbehaves, show the same mission without Gazebo, using plain SITL and the live map, as described in README §3. Or show the recorded video.

## 3. Likely viva questions (with answers)
**Q: Why A\*?**
It is complete and optimal on a grid with an admissible heuristic. We use the octile distance, which never overestimates on an 8-connected grid. It's also fast enough to re-plan in flight: a 160 m × 170 m area at 1 m resolution is about 27k cells.

**Q: Why inflate the obstacles?**
The drone is not a point. It has a size (about 0.5 m), position error and overshoot when braking. The 4 m margin keeps the planned path safely away from walls. It's the configuration-space idea: shrink the robot to a point and grow the obstacles instead.

**Q: Why simplify the path?**
A\* gives a staircase of grid cells. Line-of-sight shortcutting keeps only the corners, so ArduPilot gets about 5 smooth straight legs instead of 180 tiny ones.

**Q: How is the blockage detected?**
With a simulated forward-looking range sensor (think lidar or sonar) of 25 m range. Each control tick, we sample the remaining route up to 25 m ahead of the drone and check whether it enters the blockage footprint plus the margin. On a real drone this would be lidar, radar, ultrasonic or depth-camera data, as in Unit 3.

**Q: Why wait instead of immediately re-planning?**
The blockage is *temporary*. If it clears quickly, waiting keeps the shortest route. If not, the detour costs extra distance and battery. We trade a bounded wait (20 s) against a detour. That is the `short` scenario versus the `long` one.

**Q: What if no detour exists?**
`PlanningError` is raised, and the drone plans a route home and aborts the delivery.

**Q: How do you command the drone?**
Over MAVLink with `pymavlink`, in **GUIDED** mode:
- `SET_POSITION_TARGET_LOCAL_NED` for waypoints
- `MAV_CMD_NAV_TAKEOFF` for takeoff
- `MAV_CMD_DO_SET_MODE` for mode changes (LAND, RTL)
- `PARAM_SET` for the fence, speed and RTL altitude
- Position comes back as `LOCAL_POSITION_NED`.

**Q: What keeps the drone within boundaries?**
Three layers:
1. The planner only plans inside the map bounds.
2. ArduPilot's **geofence**: `FENCE_TYPE=3` is a 220 m circle plus a 50 m maximum altitude, and a breach triggers RTL.
3. `RTL_ALT` is 45 m, higher than the tallest building (40 m), so even a straight-line failsafe RTL can't hit anything.

**Q: What are NED and ENU?**
- **NED** (North-East-Down) is the frame ArduPilot uses. The planner works in north/east metres from home.
- **ENU** (East-North-Up) is the frame Gazebo uses. So a building at north = N, east = E is placed in Gazebo at x = E, y = N. `generate_world.py` does this conversion.

**Q: How do the simulators connect?**
Gazebo simulates the physics and the iris drone. The `ardupilot_gazebo` plugin exchanges sensor data and motor commands with ArduPilot SITL as JSON over UDP. SITL runs the real ArduCopter firmware. MAVProxy forwards MAVLink to QGC on UDP 14550 and to our script on UDP 14560.

**Q: Where is ROS?**
ROS2 Humble and the `ros_gz` bridge are installed per the course guide. Our mission talks to ArduPilot directly over MAVLink, which is simpler and the standard ArduPilot interface. A natural extension would be a ROS2 node publishing a real Gazebo lidar topic (`/scan`), replacing the simulated range sensor.

**Q: Real-world issues?**
- GPS error or multipath near buildings
- Wind
- Communication loss, which needs the CNPC link and failsafes from Unit 3 part 2
- Battery margin, which needs energy-aware planning
- Payload release, which needs a gripper or servo (`MAV_CMD_DO_GRIPPER`)
- Regulations: BVLOS (beyond visual line of sight) rules
