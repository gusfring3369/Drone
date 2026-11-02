#!/usr/bin/env python3
"""
Offline test of the mission logic with a simulated (kinematic) drone and a fake clock.
No ArduPilot/Gazebo needed - runs in a few seconds:

    python3 tests/test_mission.py
"""
from __future__ import annotations

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import planner  # noqa: E402
from mission import GazeboWorld, Mission  # noqa: E402


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def sleep(self, dt):
        self.now += dt
        if self.now > 1000.0 + 900:
            raise RuntimeError("mission took too long (stuck?)")


class FakeDrone:
    """Point-mass quadcopter: flies straight at `speed` toward the current target."""

    def __init__(self, clock: FakeClock, speed: float):
        self.clock = clock
        self.speed = speed
        self.n = self.e = self.alt = 0.0
        self.target = None
        self.last = clock()
        self.max_alt = 0.0

    def _integrate(self):
        dt = self.clock() - self.last
        self.last = self.clock()
        if self.target is None or dt <= 0:
            return
        tn, te, ta = self.target
        dn, de, da = tn - self.n, te - self.e, ta - self.alt
        dist = math.sqrt(dn * dn + de * de + da * da)
        step = min(dist, self.speed * dt)
        if dist > 1e-9:
            self.n += dn / dist * step
            self.e += de / dist * step
            self.alt += da / dist * step
        self.max_alt = max(self.max_alt, self.alt)

    def update(self):
        self._integrate()

    def position(self):
        return (self.n, self.e, self.alt)

    def wait_ready(self):
        pass

    def configure(self, cfg):
        pass

    def arm_and_takeoff(self, alt):
        self.alt = alt
        self.target = (self.n, self.e, alt)
        self.last = self.clock()

    def goto(self, n, e, alt):
        self._integrate()
        self.target = (n, e, alt)

    def hold(self, alt):
        self._integrate()
        self.target = (self.n, self.e, alt)

    def land(self):
        self._integrate()
        self.alt = 0.0
        self.target = None
        self.clock.sleep(10)
        self.last = self.clock()


def run(scenario: str):
    cfg = planner.load_config()
    clock = FakeClock()
    drone = FakeDrone(clock, cfg["cruise_speed"])
    gz = GazeboWorld(cfg["world_name"], enabled=False, log=lambda *_: None)
    logdir = tempfile.mkdtemp(prefix=f"mission_{scenario}_")
    lines = []
    m = Mission(cfg, drone, gz, scenario, logdir, clock=clock, sleep=clock.sleep, log=lines.append)
    m.run()
    m.close()
    return cfg, m, drone, lines, logdir


def check_track_clear(cfg, mission):
    boxes = planner.obstacles_from_config(cfg)
    for n, e, alt in mission.track:
        for b in boxes:
            assert not b.contains(n, e), f"track point ({n},{e}) inside {b.name}"


def main():
    failures = 0
    for scenario in ["none", "short", "long"]:
        cfg, m, drone, lines, logdir = run(scenario)
        try:
            assert m.state == "COMPLETE", m.state
            assert any("PACKAGE DELIVERED" in e for e in m.events)
            check_track_clear(cfg, m)
            home = (cfg["hospital"]["north"], cfg["hospital"]["east"])
            assert math.hypot(drone.n - home[0], drone.e - home[1]) < 2.0, "did not return home"
            if scenario == "none":
                assert m.stats["replans"] == 0 and m.blockage is None
            if scenario == "short":
                assert any("BLOCKAGE DETECTED" in e for e in m.events)
                assert any("resuming original route" in e for e in m.events)
                assert m.stats["replans"] == 0
                assert 7.0 <= m.stats["wait_time"] <= 9.5, m.stats
            if scenario == "long":
                assert any("BLOCKAGE DETECTED" in e for e in m.events)
                assert any("re-planning" in e for e in m.events)
                assert m.stats["replans"] == 1
                # the detour must never enter the blockage footprint
                b = m.blockage.box
                assert not any(b.contains(p[0], p[1]) for p in m.track), "flew through blockage"
            assert os.path.exists(os.path.join(logdir, "flight_log.csv"))
            print(f"PASS  scenario={scenario:<5}  distance={m.stats['distance']:.0f} m  "
                  f"wait={m.stats['wait_time']:.1f} s  replans={m.stats['replans']}  logs={logdir}")
        except AssertionError as exc:
            failures += 1
            print(f"FAIL  scenario={scenario}: {exc}")
            print("\n".join(lines[-30:]))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
