"""Historical controller; not part of the current saved-map fleet workflow."""
from pathlib import Path
import runpy
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
if __name__ == '__main__':
    runpy.run_module('robot_navigation.legacy_tasks', run_name='__main__')
