# Multi-Robot Natural Language Navigation

An LLM-powered multi-robot navigation system that converts plain English instructions into coordinated robot movements inside [CoppeliaSim](https://www.coppeliarobotics.com/). Two Pioneer p3dx robots navigate a scene with obstacle avoidance, partition-wall bypass routing, and automatic task reassignment when a robot gets stuck.

## How It Works

1. You type a natural language instruction (e.g. *"Send Robot A to the charging station and Robot B to the loading zone"*).
2. A **Gemini LLM** decomposes the instruction into a per-robot task plan (JSON).
3. The plan is **validated** against a grounding table of known landmark coordinates.
4. Both robots execute their tasks **concurrently** inside CoppeliaSim, with:
   - Proportional heading control
   - Ultrasonic obstacle avoidance (left/right steering)
   - Automatic routing around a partition wall
   - Stall detection → task reassignment to the other robot

## Prerequisites

| Requirement | Details |
|---|---|
| **Python** | 3.10 or newer |
| **CoppeliaSim** | Version 4.6+ (EDU or Player both work). Download from [coppeliarobotics.com/downloads](https://www.coppeliarobotics.com/downloads). |
| **CoppeliaSim ZMQ Remote API** | Ships with CoppeliaSim 4.6+. The Python client is installed via pip (see below). |
| **Google AI API key** | The planner calls Gemini. Get a free key at [aistudio.google.com/apikeys](https://aistudio.google.com/apikeys). |

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/Shiva-Shankar-Dev/minor.git
cd minor
```

### 2. Create a virtual environment and install dependencies

```bash
python3 -m venv venv
source venv/bin/activate        # Linux / macOS
# venv\Scripts\activate          # Windows
```

Install the Python packages that `main.py` actually uses:

```bash
pip install numpy google-genai coppeliasim-zmqremoteapi-client
```

> **Note:** The `requirements.txt` in this repo lists dependencies for an alternative approach to the project (Anthropic/OpenAI, matplotlib, pygame, etc.). It is **not** used by `main.py`. The three packages above are everything `main.py` needs.

### 3. Set your Google AI API key

The `google-genai` client reads the key from the environment variable `GOOGLE_API_KEY`:

```bash
export GOOGLE_API_KEY="your-key-here"        # Linux / macOS
# set GOOGLE_API_KEY=your-key-here            # Windows CMD
# $env:GOOGLE_API_KEY="your-key-here"         # Windows PowerShell
```

You can also put this in a `.env` file and load it with `python-dotenv`, but `main.py` does not do that automatically — the environment variable is all it checks.

### 4. Open the CoppeliaSim scene

1. Launch CoppeliaSim.
2. Open the scene file (`.ttt`) that comes with this project — see the section below on which file you need.
3. **Do not press the Play button** — `main.py` starts the simulation programmatically when it's ready.

### 5. Run

```bash
python main.py
```

You'll see:

```
==================================================
 Multi-Robot Natural Language Navigation CLI
==================================================

Enter multi-robot instruction (or 'q' to quit):
```

Type an instruction like:

- `Send Robot A to the charging station and Robot B to the loading zone`
- `Both robots go to the left corridor`
- `Robot A patrol charging station then loading zone, Robot B go to left corridor`

The robots will start moving in the CoppeliaSim window, and progress is logged to the terminal and to `execution_log.csv`.

## The CoppeliaSim Scene File (`.ttt`)

`main.py` expects a very specific scene layout:

| Element | Expected hierarchy path |
|---|---|
| Robot A | `/Robot_A` — Pioneer p3dx with `/Robot_A/leftMotor`, `/Robot_A/rightMotor`, `/Robot_A/ultrasonicSensor[0..15]` |
| Robot B | `/Robot_B` — same structure under `/Robot_B` |
| Partition wall | Vertical wall at X ≈ 2.55, spanning Y from ≈ −2.4 to ≈ +2.475 |
| Landmarks | `charging_station` at (0, 5), `left_corridor` at (5, 0), `loading_zone` at (4.5, 4.5) — floor markers/zones at these coordinates |

**The `.ttt` scene file is required to run this project.** It is not auto-generated — you must share it alongside the code. Yes, you can (and should) add it to the GitHub repo. CoppeliaSim `.ttt` files are binary, but Git handles them fine. Just:

```bash
git add your_scene.ttt
git commit -m "Add CoppeliaSim scene file"
git push
```

There is no `.ttt` file in the repo yet. If you're a collaborator, ask the project owner for the scene file.

## Project Structure

```
minor/
├── main.py              # Everything — LLM planner, motion control, concurrent execution
├── requirements.txt     # Dependencies for an alternative approach (not used by main.py)
├── .env.example         # Template for an alternative approach's config
├── .gitignore
├── execution_log.csv    # Generated at runtime — task/event log
└── README.md
```

## Known Landmarks

| Name | Coordinates (X, Y) | Description |
|---|---|---|
| `charging_station` | (0.0, 5.0) | Top-left area |
| `left_corridor` | (5.0, 0.0) | Bottom-right area |
| `loading_zone` | (4.5, 4.5) | Top-right area |

## Troubleshooting

| Problem | Fix |
|---|---|
| `ConnectionRefusedError` on startup | CoppeliaSim isn't running, or the ZMQ remote API add-on isn't loaded. Make sure CoppeliaSim is open with the scene loaded before running `main.py`. |
| `"Proximity sensor handle not resolved"` | The robot names or sensor paths in the scene don't match what `main.py` expects. Check that your scene has `/Robot_A/ultrasonicSensor[2]`, `[3]`, `[4]`, `[5]` (and same for Robot_B). |
| `ServerError` / 503 from Gemini | Gemini is rate-limited or temporarily down. The script retries up to 3 times automatically. If it keeps failing, wait a minute and try again. |
| Robots drive through the wall | The wall geometry constants in `main.py` (lines 98–106) may not match your scene. Measure the wall's position and extents in CoppeliaSim and update if needed. |
| `GOOGLE_API_KEY` not found | Make sure you exported the environment variable in the same shell session where you run `python main.py`. |
