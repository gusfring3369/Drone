# AGENTS.md

All project context for AI coding agents lives in **[CLAUDE.md](CLAUDE.md)**. Read it first.
It covers the assignment, team, deliverables, status, design, how to run and test, and the environment issues already solved.

Quick commands:
```bash
python3 tests/test_mission.py        # offline test of the mission logic (no simulator needed)
python3 scripts/planner.py           # print the A* route, write docs/plan_preview.png
python3 scripts/generate_world.py    # regenerate the Gazebo world after editing config/mission.json
./tools/build_report.sh              # rebuild docs/CS655_Q8_Report.docx from docs/report.md
./scripts/run_all.sh long            # full simulation (Ubuntu 22.04 + Gazebo + ArduPilot)
```
