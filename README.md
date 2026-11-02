# Medical Package Delivery Drone (CS655 Drone Technologies, Q8)

> **Problem statement (Q8):** *Simulate a drone delivering a medical package to a remote location while avoiding obstacles and handling a temporary route blockage.*

**Team:**

| Roll No. | Name |
|---|---|
| 206126035 | TODO |
| 206126033 | TODO |
| 206126034 | Sriram |

**Course:** M.Tech (CSE) I Year, July 2026.

**Deadlines:** live demo by **07.12.2026**; softcopy by **10.12.2026, 11 am**.

New here, or an AI assistant? Read **[CLAUDE.md](CLAUDE.md)** for the complete project context.

## Assignment deliverables
| # | Required by the brief | Where | Status |
|---|---|---|---|
| 1 | Document: team details, Q no., problem statement, design, tools, environment | [`docs/CS655_Q8_Report.docx`](docs/CS655_Q8_Report.docx) (source: [`docs/report.md`](docs/report.md)) | ✅ Add teammates' names |
| 2 | Demo video, under 2 minutes | [`media/`](media/), plan in [`docs/DEMO_AND_VIVA.md`](docs/DEMO_AND_VIVA.md) | ⏳ To record |
| 3 | Execution procedure, all files and code, README | This README, [`docs/SETUP_GUIDE.md`](docs/SETUP_GUIDE.md), [`scripts/`](scripts/) | ✅ |
| 4 | Contribution of each member | Report §9 | ⏳ Team to fill in |
| – | Live demo checklist and viva Q&A | [`docs/DEMO_AND_VIVA.md`](docs/DEMO_AND_VIVA.md) | ✅ |
| – | Assignment brief and course files | [`docs/ASSIGNMENT.md`](docs/ASSIGNMENT.md), [`docs/course_material/`](docs/course_material/) | ✅ |

## What the simulation does
A quadcopter (ArduPilot SITL in Gazebo) takes off from the **City Hospital** and flies an **A\*** route around six buildings (25–40 m tall; it cruises at 15 m) to the **Remote Village Clinic**. It lands to deliver the package, then flies back.

During the outbound flight, a **temporary blockage** appears on the route: an orange tower standing for a crane, a fallen tree or a temporary no-fly zone. The drone handles it like this:
1. A simulated forward range sensor **detects** it 25 m ahead.
2. The drone **holds** position.
3. It **waits** up to 20 s:
   - **If the blockage clears in time,** the drone resumes the original route (`--scenario short`).
   - **If it's still there,** the drone **re-plans** a detour with A\* (`--scenario long`).

The return leg avoids the blockage if it's still present. A geofence (220 m circle and 50 m ceiling) and an RTL altitude of 45 m, above every building, act as failsafes.

![A* route (blue), blockage (red), re-planned detour (green)](docs/plan_preview.png)

| Scenario | What happens | Offline test result |
|---|---|---|
| `long` (default) | Blockage outlasts the 20 s wait, so the drone re-plans | 371 m, 20 s wait, 1 re-plan |
| `short` | Blockage clears after 8 s, so the drone resumes | 357 m, 8 s wait |
| `none` | No blockage | 356 m |

## Tools and platform
| Component | Used |
|---|---|
| OS | Ubuntu 22.04 LTS in a VMware Workstation Pro VM, with 3D acceleration, on a Windows 11 laptop |
| Simulator | Gazebo Harmonic (gz-sim 8), with `--render-engine ogre` in the VM |
| Flight controller | ArduPilot ArduCopter SITL + `ardupilot_gazebo` plugin |
| Ground station | QGroundControl 5.0.8 (optional) |
| Middleware | ROS2 Humble + `ros_gz` bridge, installed per the course guide |
| Mission code | Python 3: `pymavlink` (MAVLink), `matplotlib` |

## Repository layout
```
CLAUDE.md, AGENTS.md         full project context for AI assistants / new contributors
config/mission.json          the scenario: hospital, village, buildings, blockage, speed, fence, camera
scripts/planner.py           occupancy grid + A* + path simplification
scripts/mission.py           MAVLink mission state machine (takeoff, follow, hold, re-plan, land, return)
scripts/live_map.py          live 2-D map of the mission
scripts/generate_world.py    builds the Gazebo world + blockage model from the config
scripts/run_all.sh           starts everything (4 terminals) | scripts/stop_all.sh stops everything
scripts/viz.py               plotting helpers
tests/test_mission.py        offline test with a simulated drone (no simulator needed)
worlds/, models/             generated Gazebo files
tools/build_report.sh        rebuilds the Word report from docs/report.md
docs/                        report (.md + .docx), setup guide, demo/viva notes, assignment brief, results, course files
media/                       demo video (or a link to it)
logs/                        created at run time (git-ignored)
```

---

## 1. One-time setup
Follow **[docs/SETUP_GUIDE.md](docs/SETUP_GUIDE.md)**. It's the course install guide plus every fix we needed: VMware VM, apt sources, Gazebo flicker, and the ArduPilot "frame class" bug.

Then get this project and its Python packages. Run each command on its own:
```bash
cd ~ && git clone -b claude/wizardly-bohr-mxzqfb https://github.com/gusfring3369/Drone.git
```
```bash
sudo apt install python3-matplotlib -y
```
```bash
cd ~/Drone && python3 tests/test_mission.py
```
The test should print 3× `PASS`. `pymavlink` comes with ArduPilot's prerequisites. If it's missing, install both packages with pip:
```bash
python3 -m pip install --user pymavlink matplotlib
```

## 2. Run the simulation

### Easiest: one command
```bash
cd ~/Drone && ./scripts/run_all.sh long
```
This opens 4 terminals: Gazebo, ArduPilot SITL, the live map and the mission. Other forms:
- **`./scripts/run_all.sh short`:** the "wait and resume" scenario.
- **`./scripts/run_all.sh none`:** no blockage.
- **`./scripts/run_all.sh long --fresh`:** also wipes saved ArduPilot parameters. Use it if you see `Check frame class and type`.

Stop everything with:
```bash
cd ~/Drone && ./scripts/stop_all.sh
```

### Manual: one terminal each, in this order
**Terminal 1: Gazebo**
```bash
cd ~/Drone && gz sim -v4 -r worlds/medical_delivery.sdf --render-engine ogre
```
**Terminal 2: ArduPilot SITL.** Wait for `pre-arm good`. Add `-w` once if you ever see `Check frame class and type`.
```bash
cd ~ && sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --out=udp:127.0.0.1:14560
```
**Terminal 3: live map**
```bash
cd ~/Drone && python3 scripts/live_map.py
```
**Terminal 4: mission.** Use `--scenario short` or `--scenario none` for the other scenarios.
```bash
cd ~/Drone && python3 scripts/mission.py --scenario long
```
**Optional: QGroundControl.** It connects on UDP 14550 by itself.
```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

Why the two `--add-param-file` options: current ArduPilot `master` looks up its built-in default parameters by the `--model` name (`JSON`) and so loads none. `FRAME_CLASS` then stays 0, and arming fails with `PreArm: Motors: Check frame class and type`. The details are in SETUP_GUIDE §7.

### What you should see
1. **PREFLIGHT:** the script sets the speed, RTL altitude and geofence. The Gazebo camera starts **following the drone**; use `--no-follow` to turn that off.
2. **TAKEOFF** to 15 m, and the **orange blockage tower** appears on the route.
3. **FOLLOW_PATH:** the drone flies the A\* waypoints around the buildings.
4. **HOLD:** at about 25 m from the blockage it stops. The live map shows the sensor circle.
5. **After the wait:**
   - **`long`:** after 20 s, **REPLAN**. A green detour appears and the drone flies around the blockage.
   - **`short`:** the blockage disappears after 8 s and the drone resumes.
6. **LAND** at the village, **PACKAGE DELIVERED**, then a 5 s pause.
7. **TAKEOFF**, the return route, **LAND** at the hospital, and **COMPLETE**.

**Outputs** go to `logs/`:
- `flight_map.png`: the route, the blockage and the actual track
- `flight_log.csv`: position and state at 4 Hz
- `events.txt`: the mission timeline

To keep the outputs of a run, copy them into `docs/results/sitl_gazebo/`; see [`docs/results/README.md`](docs/results/README.md).

**Running again:** the drone ends landed at the hospital, so you can run `mission.py` again straight away. After any crash or confusion, do `./scripts/stop_all.sh` and start over. Always restart Gazebo and SITL together, Gazebo first.

## 3. Without Gazebo (lighter computers)
The blockage is then only shown on the live map.
```bash
cd ~ && sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14560
```
```bash
cd ~/Drone && python3 scripts/mission.py --no-gazebo --scenario long
```
On Windows with Mission Planner's **Simulation** tab (Multirotor):
```bash
python scripts/mission.py --no-gazebo --connect tcp:127.0.0.1:5762
```

## 4. Changing the scenario
Edit `config/mission.json`: you can move the buildings, the village, the blockage position (`route_fraction`), the sensor range, the wait timeout, the speed, the altitude or the camera offset. Then regenerate the world and the preview:
```bash
python3 scripts/generate_world.py && python3 scripts/planner.py
```

## 5. Rebuilding the report
After editing `docs/report.md` (for example, to add names or contributions), rebuild the Word file. It needs `sudo apt install pandoc`.
```bash
./tools/build_report.sh
```

## 6. Troubleshooting
| Problem | Fix |
|---|---|
| `PreArm: Motors: Check frame class and type` | Use the SITL command above with both `--add-param-file` options, plus `-w` once (or `run_all.sh ... --fresh`). |
| `No heartbeat from the autopilot` | SITL must run with `--out=udp:127.0.0.1:14560`. |
| `iris_with_ardupilot` / `iris_runway.sdf` not found | `GZ_SIM_RESOURCE_PATH` must include `~/ardupilot_gazebo/models` (SETUP_GUIDE §6). Open a new terminal. |
| Gazebo window black or flickering | Keep `--render-engine ogre`, or run headless with `gz sim -s ...`. |
| Drone doesn't arm (`PreArm: ...`) | Wait 30–60 s for GPS and EKF. The script keeps retrying and prints the reason. |
| Blockage doesn't appear in Gazebo | Use `worlds/medical_delivery.sdf`, which has the UserCommands plugin. The mission still works with a virtual blockage. |
| Camera doesn't follow | Gazebo must be open before `mission.py` starts. Otherwise, right-click the drone → Follow. |
| More setup problems | [`docs/SETUP_GUIDE.md`](docs/SETUP_GUIDE.md), quick troubleshooting index at the end |
