# Multi-robot task allocation

**Start here: [step-by-step workflow](docs/WORKFLOW.md).** This project converts instructions into structured navigation tasks, assigns robots by map route distance and queued work, and reassigns failed tasks locally without another LLM request.

For normal use, open `scenes/environment.ttt` in CoppeliaSim, keep simulation stopped, and run:

```bash
cd ~/minor
python3 main.py --model gemini-2.5-flash
```

At `fleet>` enter a task. After completion, enter another task. `/status` shows robots, `/stations` lists destinations, and `/quit` ends the session. `run_session.py` remains an equivalent entry point. Configure the Gemini API key in your environment; see `.env.example` for variable names (the application does not automatically load that file).

## Folder responsibilities

| Folder | Purpose | What you normally do |
|---|---|---|
| `scenes/` | CoppeliaSim scene | Open and save `environment.ttt` here |
| `maps/mapping_fixed/` | Current approved map: YAML, PGM, NPZ | Keep it; no rescan needed unless geometry changes |
| `config/` | Robot paths, destinations/arrival slots, example missions | Edit robot names or named destination coordinates |
| `results/` | Generated scans, paths, session logs | Inspect outputs; each session gets its own folder |
| `robot_navigation/` | Python implementation | Development only |
| `docs/` | Workflow, setup, research design, checklist | Begin with `WORKFLOW.md` |
| `tests/` | Automated regression checks | Run after code changes |
| `requirements/` | Dependency lists | Install through root `requirements.txt` |
| `legacy/` | Explicit launcher for the original demo | Historical reference, not the current workflow |

The root Python files are command entry points, not duplicate implementations:

| Command | Use |
|---|---|
| `main.py` / `run_session.py` | Everyday natural-language fleet session |
| `map_environment.py` | Scan or extend a map |
| `inspect_robot.py`, `choose_goal.py`, `navigate_map.py` | Individual robot diagnostics/navigation |
| `allocate_tasks.py` | Generate an allocation without moving by default |
| `run_fleet.py` | Execute a saved mission once |

## Setup and validation

```bash
cd ~/minor
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 -m unittest discover -s tests -v
```

CoppeliaSim must provide the ZMQ Remote API. Use only one robot controller at a time. Mapping uses simulator poses; navigation uses wheel odometry and local LiDAR matching against the saved map. This is not a full loop-closing SLAM system.

Two-robot navigation, persistent sessions, and reassignment have been tried live. Shared arrival slots and planning-margin recovery need further live verification. Five-to-ten-robot operation is future work; passing offline tests does not establish reliability for all scenes.

See [documentation index](docs/README.md), [review rehearsal](docs/REVIEW_DEMO.md), [project checklist](docs/PROJECT_TODO.md), and [research design](docs/RESEARCH_DESIGN.md).

## Five-robot setup

Fleet cruising speed now defaults to 0.18 m/s. Follow [FIVE_ROBOTS.md](docs/FIVE_ROBOTS.md) to create a separate five-robot scene, run the short simultaneous-motion test, and launch a five-robot session. This configuration has offline integration coverage; live physics/localization and congested routes still need verification.
