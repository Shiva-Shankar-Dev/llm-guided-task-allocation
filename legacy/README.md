# Historical demo

`python3 legacy/run_demo.py` runs the original language-task demo implemented in `robot_navigation/legacy_tasks.py`. It uses different navigation logic and is retained only for reference. Do not run it alongside any current controller.

For the current project, use `python3 main.py --model gemini-2.5-flash` instead. The old `main.py` entry point now launches the persistent fleet session.
