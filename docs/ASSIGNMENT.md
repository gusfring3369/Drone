# Assignment brief: CS655 Drone Technologies (July 2026)

Source: [`course_material/Drones_Assignment.pdf`](course_material/Drones_Assignment.pdf), transcribed here so that it is searchable.

- **Programme:** M.Tech (CSE), I Year
- **Course:** CS655 Drone Technologies (July 2026)
- **Assessment:** Assignment, **20 marks**

## Our team: Question 8
| Roll No. | Name |
|---|---|
| 206126035 | _TODO: add name_ |
| 206126033 | _TODO: add name_ |
| 206126034 | Sriram |

> **Q8 problem statement:** *Simulate a drone delivering a medical package to a remote location while avoiding obstacles and handling a temporary route blockage.*

## Instructions from the brief, and where each one is covered in this repo
| # | Instruction (verbatim) | Covered by | Status |
|---|---|---|---|
| 1 | Submit the softcopy of the document containing Team details (Roll No., Name), Question no., Problem statement, Design, Tool Used, Environment / platform | [`CS655_Q8_Report.docx`](CS655_Q8_Report.docx) (generated from [`report.md`](report.md)) | Ready. **TODO:** add the two teammates' names |
| 2 | Demo video (less than 2 minutes) | Recording plan and how-to: [`DEMO_AND_VIVA.md`](DEMO_AND_VIVA.md). Put the final file in `media/` | **TODO:** record it |
| 3 | Set of procedures to be followed for execution with all files & code with read me file | [`../README.md`](../README.md), [`SETUP_GUIDE.md`](SETUP_GUIDE.md), all code in [`../scripts`](../scripts) | Done |
| 4 | Contribution of each member of the team | Section 9 of the report | **TODO:** fill in truthfully |
| 5 | Submission of softcopy: **10.12.2026 before 11 am** | | |
| 6 | Submission details: https://classroom.google.com/c/ODk3MDYwNzQ0MDgw?cjc=yqdffn6w · Class code: **kztl4rbm** (the PDF also shows `yqdffn6w`) | | |
| 7 | Show **live demo on or before 07.12.2026** | Demo checklist: [`DEMO_AND_VIVA.md`](DEMO_AND_VIVA.md) | |
| 8 | Use any simulator / open software / tool of your choice | We used ArduPilot SITL + Gazebo Harmonic + QGroundControl + Python (pymavlink), with ROS2 Humble installed per the course guide | Done |

## All problem statements in the brief (for context)
| Q | Team | Problem statement |
|---|---|---|
| 1 | 206126013, 206126009, 206126032 | Simulate a drone equipped with a camera to capture crop images. Integrate an image classification or object detection model to identify potentially diseased plants and mark their locations on a field map. |
| 2 | 206126019, 206126022, 206126017 | Simulate a drone patrolling a forest environment to identify fire locations using simulated thermal-camera data. |
| 3 | 206126002, 206126003, 206126027 | Develop a drone simulation that surveys a virtual wildlife reserve, detects animals using camera data, and estimates the number of animals observed without repeatedly counting the same animal. |
| 4 | 206126005, 206126031, 206126023 | Simulate a drone monitoring a road intersection. Use computer vision to count vehicles and classify traffic density as low, medium, or high. |
| 5 | 206126016, 206126001, 206126012 | Simulate a delivery drone transporting a package between specified locations while avoiding obstacles and respecting predefined flight boundaries. |
| 6 | 206126011, 206126025 | Develop a drone simulation that surveys a parking lot, detects occupied and vacant parking spaces, and generates a parking availability report. |
| 7 | 206126018, 206126008, 206126007 | Develop a simulation in which a drone searches a predefined disaster zone for simulated human targets. |
| **8** | **206126035, 206126033, 206126034** | **Simulate a drone delivering a medical package to a remote location while avoiding obstacles and handling a temporary route blockage.** |
| 9 | 206126030, 206126006, 206126014 | Simulate a drone surveying a solar farm and detecting panels with artificially introduced visual anomalies. |
| 10 | 206126010, 206126024, 206126029 | Simulate a drone navigating through a room containing furniture and moving obstacles. Implement a collision-avoidance method. |
| 11 | 306426001, 206126020, 206126021 | Simulate a drone surveying a public park or beach. Use object detection to identify litter in camera images and create a map of detected waste locations. |
| 12 | 206126004, 206126015 | Simulate a drone to capture aerial images or video of public places, such as railway stations, stadiums, festivals, or campus events. Integrate a computer vision model to detect people, estimate crowd density, and identify regions with high crowd concentration. |
| 13 | 406326003, 406326004, 411125053 | Develop an autonomous navigation system for a drone and also add the feature to return the drone to home when the battery is low (provide alert). |

## Course material in this repo
Everything is in [`course_material/`](course_material):

| File | Contents |
|---|---|
| `Drones_Assignment.pdf` | The assignment brief above |
| `install_Guide.docx` | Official setup: Ubuntu 22.04, ROS2 Humble, Gazebo Harmonic, ArduPilot SITL, `ardupilot_gazebo`, QGroundControl. Our tested version with fixes is in [`SETUP_GUIDE.md`](SETUP_GUIDE.md). |
| `Unit_3_-_part_1.pptx` | Obstacle detection and avoidance; mission and path planning. The key idea used: a pre-flight optimal reference trajectory, plus dynamic local re-planning when unknown threats appear. |
| `Unit_3_-_part_2.pptx` | Wireless communication on UAVs (CNPC links, data links); energy-aware operation; multi-UAV and swarms |
| `Unit_4_-_Part_1.pptx` | Drone design process; calibration (IMU, compass, radio, ESC, battery); propeller and thrust testing |
| `Unit_4_-_Simulator.pptx` | ROS, Gazebo + ArduPilot plugin, Mission Planner steps (including failsafe and RTL), Mission Planner simulation |

How our design maps to the syllabus:
- **Path planning** (Unit 3): A\* on an occupancy grid with inflated obstacles, which is the pre-flight global plan.
- **Obstacle detection and avoidance** (Unit 3): a simulated forward-looking range sensor, plus hold and re-plan, which is the dynamic local re-planning.
- **Communication** (Unit 3 part 2): the MAVLink link between the mission computer (GCS/companion) and the flight controller, with telemetry to QGC.
- **Simulator, ROS and failsafe** (Unit 4): SITL + Gazebo, with a geofence and RTL failsafe.
