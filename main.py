"""Compatibility entry point for the original language-task demo."""
import runpy

if __name__ == "__main__":
    runpy.run_module("robot_navigation.legacy_tasks", run_name="__main__")
