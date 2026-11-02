#!/usr/bin/env python3
"""
Medical package delivery mission (CS655 assignment, Q8).

The drone takes off from the hospital, follows an A* route around the
buildings to the remote village clinic, lands to deliver the package,
then flies back. A temporary blockage appears on the route during the
outbound flight; the drone detects it with a (simulated) forward-looking
range sensor, hovers and waits, then either
  * resumes the original route if the blockage clears in time, or
  * re-plans around it with A* if it is still there after the wait timeout.

State machine:
  PREFLIGHT -> TAKEOFF -> FOLLOW_PATH --(blockage ahead)--> HOLD
       HOLD --(cleared before timeout)--> FOLLOW_PATH (resume)
       HOLD --(timeout)--> REPLAN -> FOLLOW_PATH (detour)
  FOLLOW_PATH (at village) -> LAND -> DELIVERED -> TAKEOFF -> FOLLOW_PATH (return) -> LAND -> COMPLETE

Usage (see README.md for the full procedure):
  python3 scripts/mission.py --scenario long          # with Gazebo + SITL
  python3 scripts/mission.py --scenario short
  python3 scripts/mission.py --no-gazebo              # plain SITL / Mission Planner SITL
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time

import planner
from planner import Box, PlanningError

# ArduCopter flight-mode numbers
COPTER_MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}


# ==========================================================================
# Vehicle: MAVLink connection to ArduPilot (SITL or real)
# ==========================================================================
class MavlinkDrone:
    """Thin pymavlink wrapper exposing the few commands the mission needs."""

    POSITION_ONLY_MASK = 0b0000110111111000  # use x,y,z; ignore velocity, accel, yaw, yaw-rate

    def __init__(self, url: str, log):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.mavlink = mavutil.mavlink
        self.log = log
        log(f"Connecting to autopilot on {url} ...")
        self.m = mavutil.mavlink_connection(url, source_system=245, source_component=190)
        self._wait_autopilot_heartbeat(timeout=60)
        self.n = self.e = self.d = None
        self.armed = False
        self.custom_mode = None
        self.params: dict[str, float] = {}
        self._last_hb_sent = 0.0
        self._request_message_interval(self.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
        self._request_message_interval(self.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)

    # --- low level -------------------------------------------------------
    def _wait_autopilot_heartbeat(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if hb and hb.type != self.mavlink.MAV_TYPE_GCS and hb.autopilot != self.mavlink.MAV_AUTOPILOT_INVALID:
                self.m.target_system = hb.get_srcSystem()
                self.m.target_component = hb.get_srcComponent()
                self.log(f"Autopilot found (system {self.m.target_system})")
                return
        raise RuntimeError("No heartbeat from the autopilot. Is SITL running with the right --out port?")

    def _command(self, cmd: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def _request_message_interval(self, msg_id: int, hz: float) -> None:
        self._command(self.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def _wait(self, cond, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.update()
            if cond():
                return True
            time.sleep(0.05)
        return False

    def update(self) -> None:
        """Drain incoming MAVLink messages and update the cached vehicle state."""
        now = time.monotonic()
        if now - self._last_hb_sent > 1.0:  # identify ourselves as a companion/GCS
            self.m.mav.heartbeat_send(self.mavlink.MAV_TYPE_GCS, self.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_hb_sent = now
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != self.m.target_system:
                continue
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.n, self.e, self.d = msg.x, msg.y, msg.z
            elif t == "HEARTBEAT" and msg.type != self.mavlink.MAV_TYPE_GCS:
                self.armed = bool(msg.base_mode & self.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.custom_mode = msg.custom_mode
            elif t == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif t == "STATUSTEXT":
                self.log(f"    [autopilot] {msg.text}")

    # --- interface used by the mission ------------------------------------
    def position(self):
        if self.n is None:
            return None
        return (self.n, self.e, -self.d)

    def wait_ready(self, timeout: float = 180) -> None:
        self.log("Waiting for GPS/EKF position (LOCAL_POSITION_NED) ...")
        if not self._wait(lambda: self.n is not None, timeout):
            raise RuntimeError("No local position from the autopilot - EKF not ready / no GPS?")

    def set_param(self, name: str, value: float) -> bool:
        for _ in range(3):
            self.params.pop(name, None)
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(),
                                      float(value), self.mavlink.MAV_PARAM_TYPE_REAL32)
            if self._wait(lambda: name in self.params, 3):
                return abs(self.params[name] - value) < 1e-3 * max(1.0, abs(value))
        return False

    def configure(self, cfg: dict) -> None:
        s = cfg["safety"]
        params = {
            "WPNAV_SPEED": cfg["cruise_speed"] * 100,  # cm/s
            "RTL_ALT": s["rtl_alt"] * 100,               # cm, above the tallest building
            "FENCE_TYPE": 3,                              # max altitude + circle around home
            "FENCE_RADIUS": s["fence_radius"],
            "FENCE_ALT_MAX": s["fence_alt_max"],
            "FENCE_ACTION": 1,                            # breach -> RTL (or LAND)
            "FENCE_ENABLE": 1,
        }
        for name, value in params.items():
            ok = self.set_param(name, value)
            self.log(f"  param {name:<13} = {value:g} {'ok' if ok else '(NOT confirmed)'}")

    def set_mode(self, mode: str) -> None:
        mode_id = COPTER_MODES[mode]
        for _ in range(5):
            self._command(self.mavlink.MAV_CMD_DO_SET_MODE,
                          self.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
            if self._wait(lambda: self.custom_mode == mode_id, 3):
                return
        raise RuntimeError(f"Autopilot refused mode {mode}")

    def arm(self, timeout: float = 120) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._command(self.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if self._wait(lambda: self.armed, 4):
                return
        raise RuntimeError("Could not arm (see the PreArm messages above)")

    def arm_and_takeoff(self, alt: float) -> None:
        self.set_mode("GUIDED")
        self.arm()
        self._command(self.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        if not self._wait(lambda: self.n is not None and -self.d >= alt * 0.92, 60):
            raise RuntimeError("Take-off did not reach the target altitude")

    def goto(self, n: float, e: float, alt: float) -> None:
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component, self.mavlink.MAV_FRAME_LOCAL_NED,
            self.POSITION_ONLY_MASK, n, e, -alt, 0, 0, 0, 0, 0, 0, 0, 0)

    def hold(self, alt: float) -> None:
        self.update()
        self.goto(self.n, self.e, alt)

    def land(self, timeout: float = 120) -> None:
        self.set_mode("LAND")
        if not self._wait(lambda: not self.armed, timeout):
            raise RuntimeError("Landing did not complete (still armed)")


# ==========================================================================
# Gazebo: spawn / remove the blockage model in the running world
# ==========================================================================
class GazeboWorld:
    def __init__(self, world_name: str, enabled: bool, log):
        self.world = world_name
        self.log = log
        self.enabled = enabled and shutil.which("gz") is not None
        if enabled and not self.enabled:
            log("WARNING: 'gz' command not found - blockage will only be virtual (live map).")

    def _call(self, service: str, reqtype: str, req: str) -> bool:
        if not self.enabled:
            return True
        cmd = ["gz", "service", "-s", service, "--reqtype", reqtype,
               "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            return "true" in out.stdout
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _service(self, service: str, reqtype: str, req: str) -> bool:
        return self._call(f"/world/{self.world}/{service}", reqtype, req)

    def follow(self, model: str, offset: tuple[float, float, float]) -> bool:
        """Make the Gazebo GUI camera follow `model` at `offset` (x back/forward, y side, z up)."""
        ok = self._call("/gui/follow", "gz.msgs.StringMsg", f'data: "{model}"')
        self._call("/gui/follow/offset", "gz.msgs.Vector3d",
                   f"x: {offset[0]}, y: {offset[1]}, z: {offset[2]}")
        return ok

    def spawn_box(self, box: Box, sdf_path: str) -> bool:
        # Gazebo ENU: x = east, y = north
        req = (f'sdf_filename: "{sdf_path}", name: "{box.name}", '
               f'pose: {{position: {{x: {box.east}, y: {box.north}, z: 0}}}}')
        ok = self._service("create", "gz.msgs.EntityFactory", req)
        if self.enabled and not ok:
            self.log("WARNING: Gazebo did not confirm the blockage spawn (continuing with virtual blockage).")
        return ok

    def remove(self, name: str) -> bool:
        return self._service("remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')


# ==========================================================================
# Temporary blockage + simulated forward-looking range sensor
# ==========================================================================
class Blockage:
    def __init__(self, box: Box, clear_after: float, margin: float, sensor_range: float,
                 gz: GazeboWorld, sdf_path: str):
        self.box = box
        self.clear_after = clear_after
        self.margin = margin
        self.sensor_range = sensor_range
        self.gz = gz
        self.sdf_path = sdf_path
        self.active = False
        self.detected_at = None
        self.clear_at = None

    def spawn(self) -> None:
        self.gz.remove(self.box.name)  # clean up a leftover from a previous run
        self.gz.spawn_box(self.box, self.sdf_path)
        self.active = True

    def on_detected(self, now: float) -> None:
        if self.detected_at is None:
            self.detected_at = now
            self.clear_at = now + self.clear_after  # scenario injector: when the obstruction goes away

    def update(self, now: float) -> bool:
        """Clear the blockage when its time is up. Returns True at the moment it clears."""
        if self.active and self.clear_at is not None and now >= self.clear_at:
            self.gz.remove(self.box.name)
            self.active = False
            return True
        return False

    def sensed_ahead(self, pos: tuple[float, float], route_ahead: list[tuple[float, float]]):
        """Simulated range sensor: walk along the remaining route up to `sensor_range`
        metres ahead of the drone and report the distance at which the route enters
        the blockage (plus safety margin), or None if the way ahead is clear."""
        if not self.active:
            return None
        travelled = 0.0
        pts = [pos] + list(route_ahead)
        for a, b in zip(pts, pts[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = max(1, int(seg / 0.5))
            for k in range(steps + 1):
                d = travelled + seg * k / steps
                if d > self.sensor_range:
                    return None
                n = a[0] + (b[0] - a[0]) * k / steps
                e = a[1] + (b[1] - a[1]) * k / steps
                if self.box.contains(n, e, self.margin):
                    return d
            travelled += seg
        return None


# ==========================================================================
# Mission
# ==========================================================================
class Mission:
    def __init__(self, cfg: dict, vehicle, gz: GazeboWorld, scenario: str, log_dir: str,
                 clock=time.monotonic, sleep=time.sleep, log=print):
        self.cfg = cfg
        self.v = vehicle
        self.gz = gz
        self.scenario = scenario
        self.log_dir = log_dir
        self.clock = clock
        self.sleep = sleep
        self._print = log
        self.t0 = clock()
        self.state = "INIT"
        self.leg = ""
        self.alt = cfg["cruise_alt"]
        self.home = (cfg["hospital"]["north"], cfg["hospital"]["east"])
        self.village = (cfg["village"]["north"], cfg["village"]["east"])
        self.original_route: list = []
        self.route: list = []
        self.track: list = []
        self.events: list = []
        self.blockage: Blockage | None = None
        self.stats = {"replans": 0, "wait_time": 0.0, "distance": 0.0}
        self._last_pub = -1.0
        self._last_track = -1.0
        os.makedirs(log_dir, exist_ok=True)
        self._csv = open(os.path.join(log_dir, "flight_log.csv"), "w", newline="")
        self._csv_w = csv.writer(self._csv)
        self._csv_w.writerow(["t", "state", "leg", "north", "east", "alt", "blockage_active"])

    # --- logging / telemetry ----------------------------------------------
    def t(self) -> float:
        return self.clock() - self.t0

    def log(self, msg: str) -> None:
        self._print(f"[{self.t():6.1f}s] {msg}")

    def event(self, msg: str) -> None:
        self.events.append(f"{self.t():6.1f}s  {msg}")
        self.log(f"*** {msg}")
        self.publish(force=True)

    def set_state(self, state: str) -> None:
        self.state = state
        self.log(f"STATE -> {state}{' (' + self.leg + ')' if self.leg else ''}")
        self.publish(force=True)

    def publish(self, force: bool = False) -> None:
        """Write the live state for live_map.py and append to the CSV log."""
        now = self.t()
        if not force and now - self._last_pub < 0.25:
            return
        self._last_pub = now
        pos = self.v.position()
        if pos:
            if now - self._last_track >= 0.5:
                if self.track:
                    p = self.track[-1]
                    self.stats["distance"] += math.hypot(pos[0] - p[0], pos[1] - p[1])
                self.track.append([round(pos[0], 2), round(pos[1], 2), round(pos[2], 2)])
                self._last_track = now
            self._csv_w.writerow([f"{now:.2f}", self.state, self.leg, f"{pos[0]:.2f}", f"{pos[1]:.2f}",
                                  f"{pos[2]:.2f}", int(bool(self.blockage and self.blockage.active))])
        b = self.blockage
        state = {
            "t": round(now, 2), "state": self.state, "leg": self.leg, "scenario": self.scenario,
            "drone": list(pos) if pos else None,
            "route": self.route, "original_route": self.original_route, "track": self.track,
            "blockage": None if b is None else {
                "north": b.box.north, "east": b.box.east, "size": b.box.size_n, "active": b.active,
                "sensor_range": b.sensor_range, "detected": b.detected_at is not None},
            "events": self.events[-8:],
        }
        tmp = os.path.join(self.log_dir, "state.json.tmp")
        with open(tmp, "w") as f:
            json.dump(state, f)
        os.replace(tmp, os.path.join(self.log_dir, "state.json"))

    def wait(self, seconds: float) -> None:
        end = self.clock() + seconds
        while self.clock() < end:
            self.tick()
            self.sleep(0.1)

    def tick(self) -> None:
        self.v.update()
        if self.blockage and self.blockage.update(self.clock()):
            self.event("Blockage cleared (removed from the world)")
        self.publish()

    # --- flying a route -----------------------------------------------------
    def fly_route(self, route: list, goal: tuple[float, float]) -> None:
        bcfg = self.cfg["blockage"]
        tol = self.cfg["waypoint_tolerance"]
        self.route = [list(p) for p in route]
        idx = 1
        self.set_state("FOLLOW_PATH")
        self.v.goto(route[idx][0], route[idx][1], self.alt)
        self.log(f"  -> WP{idx} ({route[idx][0]:.1f}, {route[idx][1]:.1f})")
        while True:
            self.tick()
            pos = self.v.position()
            here = (pos[0], pos[1])

            # 1. Obstacle sensing on the route ahead
            dist = self.blockage.sensed_ahead(here, route[idx:]) if self.blockage else None
            if dist is not None:
                self.blockage.on_detected(self.clock())
                self.v.hold(self.alt)
                self.event(f"BLOCKAGE DETECTED {dist:.1f} m ahead at "
                           f"({self.blockage.box.north:.0f} N, {self.blockage.box.east:.0f} E) - holding position")
                self.set_state("HOLD")
                waited_from = self.clock()
                while self.blockage.active and self.clock() - waited_from < bcfg["wait_timeout"]:
                    self.tick()
                    self.sleep(0.1)
                self.stats["wait_time"] += self.clock() - waited_from
                pos = self.v.position()
                here = (pos[0], pos[1])
                if not self.blockage.active:
                    self.event(f"Route clear after {self.clock() - waited_from:.1f} s - resuming original route")
                    self.set_state("FOLLOW_PATH")
                else:
                    self.set_state("REPLAN")
                    self.event(f"Still blocked after {bcfg['wait_timeout']:.0f} s - re-planning with A*")
                    try:
                        route = planner.plan(self.cfg, here, goal, [self.blockage.box])
                    except PlanningError as exc:
                        self.event(f"No detour possible ({exc}) - aborting, returning home")
                        route = planner.plan(self.cfg, here, self.home, [self.blockage.box])
                        goal = self.home
                    self.stats["replans"] += 1
                    self.route = [list(p) for p in route]
                    idx = 1
                    self.event(f"New route: {len(route) - 1} legs, {planner.path_length(route):.0f} m")
                    self.set_state("FOLLOW_PATH")
                self.v.goto(route[idx][0], route[idx][1], self.alt)
                continue

            # 2. Waypoint progress
            wp = route[idx]
            if math.hypot(wp[0] - here[0], wp[1] - here[1]) <= tol:
                idx += 1
                if idx >= len(route):
                    self.log(f"  reached goal ({goal[0]:.0f} N, {goal[1]:.0f} E)")
                    return
                self.v.goto(route[idx][0], route[idx][1], self.alt)
                self.log(f"  -> WP{idx} ({route[idx][0]:.1f}, {route[idx][1]:.1f})")
            self.sleep(0.1)

    # --- whole mission ------------------------------------------------------
    def run(self) -> None:
        cfg = self.cfg
        self.set_state("PREFLIGHT")
        self.v.wait_ready()
        self.v.configure(cfg)

        self.original_route = [list(p) for p in planner.plan(cfg, self.home, self.village)]
        self.log(f"A* route to {cfg['village']['name']}: {len(self.original_route) - 1} legs, "
                 f"{planner.path_length(self.original_route):.0f} m")

        if self.scenario != "none":
            sc = cfg["blockage"]["scenarios"][self.scenario]
            box = planner.blockage_box(cfg, self.original_route)
            self.blockage = Blockage(box, sc["clear_after"], cfg["safety_margin"],
                                     cfg["blockage"]["sensor_range"], self.gz,
                                     os.path.join(planner.ROOT, "models", "blockage.sdf"))
            self.log(f"Scenario '{self.scenario}': {sc['description']}")

        self.leg = "outbound"
        self.set_state("TAKEOFF")
        self.v.arm_and_takeoff(self.alt)
        if self.blockage:
            self.blockage.spawn()
            self.event(f"Temporary blockage appeared on the route at "
                       f"({self.blockage.box.north:.0f} N, {self.blockage.box.east:.0f} E)")
        self.fly_route([tuple(p) for p in self.original_route], self.village)

        self.set_state("LAND")
        self.v.land()
        self.set_state("DELIVERED")
        self.event(f"PACKAGE DELIVERED to {cfg['village']['name']}")
        self.wait(cfg["delivery"]["ground_wait"])

        self.leg = "return"
        self.set_state("TAKEOFF")
        self.v.arm_and_takeoff(self.alt)
        pos = self.v.position()
        known = [self.blockage.box] if self.blockage and self.blockage.active else []
        ret = planner.plan(cfg, (pos[0], pos[1]), self.home, known)
        self.log(f"Return route: {len(ret) - 1} legs, {planner.path_length(ret):.0f} m"
                 f"{' (avoiding known blockage)' if known else ''}")
        self.fly_route(ret, self.home)

        self.set_state("LAND")
        self.v.land()
        self.leg = ""
        self.set_state("COMPLETE")
        self.event(f"Mission complete: {self.stats['distance']:.0f} m flown, "
                   f"{self.stats['wait_time']:.0f} s waiting at blockage, {self.stats['replans']} re-plan(s)")

    def close(self) -> None:
        self.publish(force=True)
        self._csv.close()
        with open(os.path.join(self.log_dir, "events.txt"), "w") as f:
            f.write("\n".join(self.events) + "\n")
        try:
            save_flight_map(self.cfg, self, os.path.join(self.log_dir, "flight_map.png"))
            self.log(f"Saved {os.path.join(self.log_dir, 'flight_map.png')}")
        except ImportError:
            self.log("matplotlib not installed - flight map image skipped")


def save_flight_map(cfg: dict, mission: Mission, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import viz
    fig, ax = plt.subplots(figsize=(8, 8))
    viz.draw_static(ax, cfg)
    viz.draw_path(ax, mission.original_route, "original A* route", color="tab:blue", style=":", lw=1.5)
    if mission.blockage:
        viz.draw_blockage(ax, mission.blockage.box, cfg["safety_margin"], active=True)
    if mission.track:
        ax.plot([p[1] for p in mission.track], [p[0] for p in mission.track], "-", color="darkorange",
                lw=2.2, label="actual flight track")
    ax.legend(loc="lower right", fontsize=8)
    ax.set_title(f"Medical delivery flight - scenario '{mission.scenario}'")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="Medical package delivery drone mission")
    ap.add_argument("--connect", default="udpin:127.0.0.1:14560",
                    help="MAVLink connection (default: udpin:127.0.0.1:14560; "
                         "Mission Planner SITL on Windows: tcp:127.0.0.1:5762)")
    ap.add_argument("--scenario", choices=["short", "long", "none"], default="long",
                    help="short: blockage clears while waiting; long: drone re-plans; none: no blockage")
    ap.add_argument("--no-gazebo", action="store_true", help="do not spawn the blockage in Gazebo")
    ap.add_argument("--no-follow", action="store_true", help="do not make the Gazebo camera follow the drone")
    ap.add_argument("--config", default=planner.DEFAULT_CONFIG)
    ap.add_argument("--log-dir", default=os.path.join(planner.ROOT, "logs"))
    args = ap.parse_args()

    cfg = planner.load_config(args.config)
    gz = GazeboWorld(cfg["world_name"], enabled=not args.no_gazebo, log=print)
    if gz.enabled and not args.no_follow:
        cam = cfg.get("camera", {})
        if gz.follow(cam.get("follow_model", "iris_with_ardupilot"), tuple(cam.get("follow_offset", [-15, 0, 8]))):
            print("Gazebo camera is following the drone")
        else:
            print("Could not set Gazebo camera follow (is the Gazebo window open?)")
    drone = MavlinkDrone(args.connect, log=print)
    mission = Mission(cfg, drone, gz, args.scenario, args.log_dir)
    try:
        mission.run()
        return 0
    except KeyboardInterrupt:
        mission.log("Interrupted by user - switching to RTL")
        drone.set_mode("RTL")
        return 1
    except Exception as exc:  # keep the vehicle safe on any failure
        mission.log(f"ERROR: {exc} - switching to RTL")
        try:
            drone.set_mode("RTL")
        except Exception:
            pass
        return 1
    finally:
        if mission.blockage and mission.blockage.active:
            gz.remove(mission.blockage.box.name)
        mission.close()


if __name__ == "__main__":
    sys.exit(main())
