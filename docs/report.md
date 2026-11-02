# CS655 Drone Technologies: Assignment Report (Draft)
**M.Tech (CSE), I Year, July 2026**

> This is a draft. Fill in the names, check every section against what your team actually did, then copy it into the Word/PDF document you submit.

## 1. Team details
| Roll No. | Name |
|---|---|
| 206126035 | _<name>_ |
| 206126033 | _<name>_ |
| 206126034 | Sriram |

## 2. Question number and problem statement
**Q8.** Simulate a drone delivering a medical package to a remote location while avoiding obstacles and handling a temporary route blockage.

## 3. Design

### 3.1 Scenario
A delivery quadcopter is based at a **City Hospital** (home, 0 m N / 0 m E). It must deliver a medical package to a **Remote Village Clinic** at 130 m N / 100 m E. Between the two are six buildings and towers, 25–40 m tall, so the drone, flying at 15 m, must fly **around** them, not over them. During the flight, a **temporary blockage** appears on the planned route, 12 m × 12 m × 30 m. It stands for a crane, a fallen tree or a temporary no-fly zone.

The flight boundary is a 160 m × 170 m planning area plus an ArduPilot geofence (220 m circle, 50 m maximum altitude).

### 3.2 System architecture
```
 config/mission.json  (single source of truth: map, obstacles, blockage, speeds, fence)
        |                         |
        v                         v
 generate_world.py         planner.py (grid + A*)
        |                         |
        v                         v
 Gazebo Harmonic world  <--  mission.py (state machine) --MAVLink (pymavlink)--> ArduPilot SITL (ArduCopter)
  (buildings, pads,          |   spawns/removes blockage       ^  flight control, EKF, geofence, RTL
   iris drone, blockage)     |   via gz service                |  JSON physics link
        ^                    v                                 |
        +---------------- logs/state.json --> live_map.py      +-- QGroundControl (map, telemetry)
```

### 3.3 Path planning (pre-flight, global)
1. **Occupancy grid** at 1 m resolution over the flight boundary.
2. **Obstacle inflation.** Each building footprint is grown by a 4 m safety margin. This accounts for the drone's size and position error.
3. **A\* search** on an 8-connected grid, using the octile-distance heuristic. Diagonal moves are not allowed to cut corners.
4. **Path simplification** by greedy line-of-sight shortcutting. The ~180 grid cells become about 5 straight-line waypoints, which are sent to ArduPilot as GUIDED-mode position targets (`SET_POSITION_TARGET_LOCAL_NED`).

### 3.4 Handling the temporary blockage (dynamic, local re-planning)
This follows Unit 3: *"pre-flight optimal reference trajectory … during flight, in the presence of unknown or dynamic threat information, local trajectory optimisation is performed dynamically."*
- **Detection.** A simulated forward-looking range sensor checks the remaining route up to 25 m ahead of the drone. If the route enters the blockage (plus the safety margin), the blockage is detected.
- **Reaction.** The drone stops and **holds position**.
- **Decision.** Because the blockage is *temporary*, the drone **waits up to 20 s**:
  - **If it clears in time,** the drone resumes the original, shortest route.
  - **If it doesn't,** the drone re-plans with A\* from its current position, treating the blockage as an obstacle, and continues on the detour.
  - **If no detour exists,** the drone returns home.

  This trades a short wait against a longer detour.
- **Return leg.** It is planned with the knowledge gained: the blockage is avoided if it is still there.

### 3.5 Mission state machine
```
PREFLIGHT -> TAKEOFF -> FOLLOW_PATH --blockage within sensor range--> HOLD
HOLD --cleared before timeout--> FOLLOW_PATH (resume)
HOLD --timeout--> REPLAN -> FOLLOW_PATH (detour)
FOLLOW_PATH @ village -> LAND -> DELIVERED -> TAKEOFF -> FOLLOW_PATH (return) -> LAND -> COMPLETE
any error / Ctrl+C -> RTL (return to launch at 45 m, above every building)
```

### 3.6 Safety and boundaries
| Feature | Setting |
|---|---|
| Geofence | `FENCE_TYPE=3` (circle + maximum altitude), radius 220 m, maximum altitude 50 m, breach action RTL |
| `RTL_ALT` | 45 m, above the tallest building (40 m), so a failsafe RTL in a straight line is collision-free |
| Planner boundary | The planner never leaves the predefined flight area |
| Safety margin | 4 m around every obstacle |

## 4. Tools used
| Tool | Purpose |
|---|---|
| ArduPilot SITL (ArduCopter) | Flight-controller firmware running in simulation |
| Gazebo Harmonic + `ardupilot_gazebo` | 3D physics world and quadcopter model |
| QGroundControl 5.0.8 | Ground-control station: map, telemetry, modes |
| Python 3, `pymavlink` | MAVLink mission control |
| `matplotlib` | Live map and flight-map images |
| ROS2 Humble + `ros_gz` bridge | Installed per the course guide; available for extensions such as a simulated lidar |

## 5. Environment / platform
| Item | Details |
|---|---|
| Host | Windows 11, Intel i7-9750H, 24 GB RAM, NVIDIA GTX 1660 Ti |
| Guest | Ubuntu 22.04.5 LTS in VMware Workstation Pro (13 GB RAM, 8 cores, 3D acceleration) |
| Simulation location | ArduPilot default home (CMAC, Canberra). Local NED frame: north/east metres from the hospital |

## 6. Execution procedure
See `README.md`. In short:
1. Start the Gazebo world.
2. Start ArduPilot SITL with `--out=udp:127.0.0.1:14560`.
3. Optionally start QGroundControl.
4. Start `live_map.py`.
5. Run `mission.py --scenario long`, or `--scenario short`.

## 7. Results
Add these after your runs:
- `docs/plan_preview.png`: the A\* route and the re-plan.
- `logs/flight_map.png` from a `long` run and from a `short` run.
- A Gazebo screenshot showing the blockage tower and the drone holding position.
- `logs/events.txt` timeline.

The offline logic test (`tests/test_mission.py`) gave:

| Scenario | Distance flown | Waiting at blockage | Re-plans |
|---|---|---|---|
| none | 356 m | 0 s | 0 |
| short | 357 m | 8 s | 0 |
| long | 371 m | 20 s | 1 |

## 8. Demo video (under 2 minutes)
1. **0:00–0:15:** the world in Gazebo, plus the A\* route on the live map.
2. **0:15–0:40:** takeoff, and following the route around the buildings.
3. **0:40–1:15:** the blockage appears, the drone holds, waits, re-plans and flies the detour. Optionally a short clip of the `short` scenario, where it resumes.
4. **1:15–1:45:** it lands at the village ("PACKAGE DELIVERED") and returns home.
5. **1:45–2:00:** the final flight map and events log.

## 9. Contribution of each member
_Edit this to match what each member actually did._

| Roll No. | Contribution |
|---|---|
| 206126035 | |
| 206126033 | |
| 206126034 (Sriram) | |
