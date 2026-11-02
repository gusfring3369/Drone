# Results

## `offline_test/`
These come from `tests/test_mission.py`: the full mission logic run against a simulated (kinematic) drone, with no ArduPilot or Gazebo needed. Each folder has `flight_map.png` and `events.txt`.

| Scenario | Distance | Waiting at blockage | Re-plans | Outcome |
|---|---|---|---|---|
| none  | 356 m | 0 s  | 0 | Delivered and returned |
| short | 357 m | 8 s  | 0 | Waited, blockage cleared, resumed original route |
| long  | 371 m | 20 s | 1 | Waited, still blocked, flew the A\* detour |

## `sitl_gazebo/`
Copy the outputs of a real run here (ArduPilot SITL + Gazebo). After `mission.py` finishes:
```bash
cd ~/Drone && mkdir -p docs/results/sitl_gazebo/long && cp logs/flight_map.png logs/events.txt logs/flight_log.csv docs/results/sitl_gazebo/long/
```
Also add Gazebo screenshots (PrtScn in Ubuntu), for example of the drone holding in front of the orange blockage. Then commit:
```bash
git add docs/results && git commit -m "Add SITL + Gazebo results" && git push
```
