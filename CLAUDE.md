# CLAUDE.md: full project context (for Claude, other LLMs and new humans)

Read this first. It records everything about this project: what the assignment demands, what has been built, how to run it, every environment problem already solved, and what is still left to do.

---

## 1. What this is
- **Course:** CS655 Drone Technologies, M.Tech (CSE) I Year, July 2026, assignment worth 20 marks.
- **Team (Question 8):**
  - 206126035: name unknown, **TODO**
  - 206126033: name unknown, **TODO**
  - 206126034: **Sriram**, the repo owner and the person who worked with Claude
- **Problem statement (Q8):** *Simulate a drone delivering a medical package to a remote location while avoiding obstacles and handling a temporary route blockage.*
- **Deadlines:**
  - **live demo on or before 07.12.2026**
  - **softcopy submission by 10.12.2026 before 11 am**, on Google Classroom (class code `kztl4rbm`; link in `docs/ASSIGNMENT.md`)
- **Required deliverables** (verbatim mapping in `docs/ASSIGNMENT.md`):
  1. Document: team (roll no. and name), question no., problem statement, design, tools used, environment/platform. This is `docs/CS655_Q8_Report.docx`, built from `docs/report.md`.
  2. A demo video under 2 minutes. **Not yet recorded.** The plan is in `docs/DEMO_AND_VIVA.md`; put the file or link in `media/`.
  3. Procedures for execution, all files and code, and a README: `README.md`, `docs/SETUP_GUIDE.md`, `scripts/`.
  4. Contribution of each member: report §9. **TODO:** the team must fill it in truthfully.
- **Course material** (original files): `docs/course_material/`. That's the assignment PDF, the install guide `.docx`, and the Unit 3 and Unit 4 slides.

## 2. Current status (as of 2 Nov 2026)
**Done:**
- **Full code:** planner, mission, Gazebo world, live map, run/stop scripts, offline tests.
- **Tests:** `tests/test_mission.py` passes for the scenarios none, short and long.
- **MAVLink layer:** verified against a fake UDP autopilot.
- **Owner's environment:** Windows 11 with VMware and an Ubuntu 22.04 VM. ROS2 Humble ✅, Gazebo Harmonic ✅ (`--render-engine ogre`), ArduPilot SITL plain takeoff ✅, `ardupilot_gazebo` plugin built.
- **First Gazebo + SITL mission run:** it hit `PreArm: Motors: Check frame class and type`. That was diagnosed and fixed (see §6, issue 1). The fixed command is in `README.md` and `scripts/run_all.sh`. The user later said "the project is done".

**Still TODO for the team (Claude can't do these):**
1. Add the two teammates' names (`docs/report.md`, `docs/ASSIGNMENT.md`, this file), then rebuild the `.docx` with `./tools/build_report.sh`.
2. Write the contributions in report §9 truthfully.
3. Record the demo video (under 2 minutes) and put it in `media/`, or put a link in `media/README.md`.
4. Optionally, add real-run outputs (`logs/flight_map.png`, a Gazebo screenshot) to `docs/results/sitl_gazebo/` and reference them in report §7.
5. Submit on Classroom by 10.12.2026 11:00, and do the live demo by 07.12.2026.

## 3. Repository map
```
CLAUDE.md / AGENTS.md         this context (AGENTS.md points here)
README.md                     user-facing overview + how to run
config/mission.json           SINGLE SOURCE OF TRUTH for the scenario (NED metres)
scripts/planner.py            occupancy grid, obstacle inflation, A*, line-of-sight simplify, helpers
scripts/mission.py            MavlinkDrone (pymavlink), GazeboWorld (gz service), Blockage (sensor sim), Mission (state machine)
scripts/viz.py                matplotlib drawing helpers
scripts/live_map.py           live 2-D map, polls logs/state.json
scripts/generate_world.py     writes worlds/medical_delivery.sdf + models/blockage.sdf from the config
scripts/run_all.sh            opens 4 gnome-terminals: Gazebo, SITL (with the param-file fix), live map, mission
scripts/stop_all.sh           pkills everything
tests/test_mission.py         offline test: FakeDrone (kinematic) + FakeClock, scenarios none/short/long
worlds/, models/              GENERATED; don't edit by hand, run generate_world.py
tools/build_report.sh         pandoc docs/report.md -> docs/CS655_Q8_Report.docx
docs/report.md                report source (title block + sections 1-9)
docs/CS655_Q8_Report.docx     generated report for submission
docs/ASSIGNMENT.md            assignment brief transcribed, plus a deliverables checklist
docs/SETUP_GUIDE.md           tested install path with every fix (Windows -> VMware -> Ubuntu 22.04 -> stack)
docs/DEMO_AND_VIVA.md         video script, recording how-to, live-demo checklist, viva Q&A
docs/results/offline_test/    flight maps + event logs from the offline tests
docs/course_material/         original course files (PDF, install guide, slides)
media/                        put the demo video (or a link) here
logs/                         runtime output (git-ignored): state.json, flight_log.csv, events.txt, flight_map.png
```

## 4. Design in brief
**Frames:**
- The config, planner and mission use **local NED**: `north` and `east` in metres from home, which is the hospital at (0, 0). Altitude is positive up in code; NED z = −alt when sent to ArduPilot.
- **Gazebo is ENU:** `generate_world.py` puts a building at north = N, east = E at Gazebo `x = E, y = N`. The `iris_with_ardupilot` model spawns at the origin with yaw 90°, as in `iris_runway.sdf`.
- The world's `spherical_coordinates` matches the SITL default home (CMAC −35.363262, 149.165237, 584 m).

**Planner:**
- 1 m grid over `map_bounds`, which also serves as the flight boundary.
- Boxes are inflated by `safety_margin` (4 m).
- A\*: 8-connected, octile heuristic, no corner cutting. If the start cell is blocked, it moves to the nearest free cell.
- Greedy line-of-sight simplification.
- `plan()` raises `PlanningError` when no path exists.

**The blockage:**
- A 12 m × 12 m × 30 m box placed on the original route at `route_fraction` 0.5, at about (78.3 N, 34.3 E).
- It is spawned in Gazebo at takeoff, using `gz service /world/medical_delivery/create` with `EntityFactory` and `sdf_filename`.
- Detection is a simulated range sensor: `Blockage.sensed_ahead()` samples the remaining route up to `sensor_range` (25 m) ahead.
- On detection, the drone holds and waits `wait_timeout` (20 s).
  - Scenario `short` clears the blockage 8 s after detection, so the drone resumes.
  - Scenario `long` clears it after 60 s, so the drone re-plans after 20 s.
- Clearing removes the model with the `/remove` service.

**Mission flow:**
1. PREFLIGHT: `wait_ready` waits for `LOCAL_POSITION_NED`; `configure` sets `WPNAV_SPEED`, `RTL_ALT`, `FENCE_*`.
2. A\* plan, then GUIDED, arm and takeoff to 15 m.
3. Spawn the blockage, then `fly_route` with hold, resume or re-plan.
4. LAND (wait for disarm), DELIVERED, wait 5 s.
5. Take off, plan the return (avoiding the blockage if it's still active), `fly_route`, LAND, COMPLETE.

Any exception or Ctrl+C triggers RTL. Logs are written to `logs/`.

**MAVLink:**
- The script connects with `udpin:127.0.0.1:14560`; SITL needs `--out=udp:127.0.0.1:14560`.
- Messages used:
  - `SET_POSITION_TARGET_LOCAL_NED` with type mask `0b0000110111111000` (position only)
  - `COMMAND_LONG`: `DO_SET_MODE` with ArduCopter custom-mode numbers (GUIDED 4, LAND 9, RTL 6), `COMPONENT_ARM_DISARM`, `NAV_TAKEOFF`, `SET_MESSAGE_INTERVAL`
  - `PARAM_SET`
- The script sends its own heartbeat as a GCS with sysid 245.

**Camera:** at start, `mission.py` calls `/gui/follow` (`StringMsg` "iris_with_ardupilot") and `/gui/follow/offset` (`Vector3d` −15, 0, 8). It can be disabled with `--no-follow`, and is configured in the `camera` section of the config.

## 5. How to run (Ubuntu 22.04 VM, after `docs/SETUP_GUIDE.md`)
```bash
cd ~/Drone && ./scripts/run_all.sh long        # or short / none ; add --fresh to wipe SITL params (-w)
./scripts/stop_all.sh
python3 tests/test_mission.py                  # offline, no simulator
python3 scripts/planner.py                     # prints route, writes docs/plan_preview.png
python3 scripts/generate_world.py              # after editing config/mission.json
./tools/build_report.sh                        # after editing docs/report.md (needs pandoc)
```
These are the manual commands, one per terminal:
```bash
cd ~/Drone && gz sim -v4 -r worlds/medical_delivery.sdf --render-engine ogre
cd ~ && sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --out=udp:127.0.0.1:14560
cd ~/Drone && python3 scripts/live_map.py
cd ~/Drone && python3 scripts/mission.py --scenario long
```

## 6. Environment issues already solved (don't rediscover these)
1. **`PreArm: Motors: Check frame class and type`** with Gazebo. Current ArduPilot `master` no longer passes `--defaults` from `sim_vehicle.py`. The SITL binary resolves the embedded defaults from `@ROMFS/vehicleinfo.json` by the `--model` name, and `JSON` has no entry, so no params load and `FRAME_CLASS` stays 0. See `resolve_defaults_from_romfs` in `libraries/AP_HAL_SITL/SITL_cmdline.cpp`. **Fix:** `--add-param-file=.../copter.parm --add-param-file=.../gazebo-iris.parm`, plus `-w` once.
2. **WSL2 failed:** no GPU passthrough (`llvmpipe`, no NVIDIA libraries in `/usr/lib/wsl/lib`), and the default distro was Ubuntu 26.04. The project switched to a **VMware Workstation Pro** VM; `glxinfo` there shows SVGA3D, OpenGL 4.3.
3. **The VM was installed offline,** so `apt` only had the CD-ROM source, giving "no installation candidate". Fix: rewrite `/etc/apt/sources.list` with the jammy archive (SETUP_GUIDE §2.2). VMware NAT also needed Virtual Network Editor → Restore Defaults.
4. **`gz-harmonic` not found:** the OSRF key and source line weren't written. Re-run them (SETUP_GUIDE §4).
5. **The Gazebo window flickers in the VM:** always use `--render-engine ogre`.
6. **MAVProxy commands typed before the prompt was ready did nothing.** Wait for `STABILIZE>`.
7. **PowerShell `bcdedit {current}`** needs quotes (`'{current}'`). A `safeboot minimal` flag was once set during a dual-boot attempt; the user was told to remove it with `bcdedit /deletevalue '{current}' safeboot`.

**Owner's machine:** Lenovo Legion Y540 (81SX), i7-9750H, 24 GB RAM, GTX 1660 Ti, Windows 11. The VM has 13 GB RAM, 8 vCPU and 3D acceleration on. Linux user `sam`, hostname `Sam`.

## 7. Working with the owner (Sriram)
- A student. They want **exact copy-paste commands, one per step**, short explanations, and no long theory unless asked. They get frustrated by multi-option answers, so give one recommended path.
- They can't easily paste long logs; screenshots are common. Ask for the specific terminal (Terminal 2 = SITL, Terminal 4 = mission).
- **Git:** the development branch is `claude/wizardly-bohr-mxzqfb` on `gusfring3369/Drone`. The user's VM clones with `git clone -b claude/wizardly-bohr-mxzqfb https://github.com/gusfring3369/Drone.git`.
- **Academic integrity:** remind them to understand the code, to write the contributions truthfully, and to follow the course's AI-use policy.

## 8. Ideas if more work is requested
- A real Gazebo `gpu_lidar` on the iris, bridged with `ros_gz_bridge` to a ROS2 `/scan` topic, replacing the simulated sensor in `Blockage.sensed_ahead()`.
- Moving obstacles; battery-aware route choice; a payload gripper (`MAV_CMD_DO_GRIPPER`) instead of landing.
- A battery-failsafe demo (`BATT_FS_LOW_ACT=2`).
- A ROS2 node wrapper that publishes the mission state.
