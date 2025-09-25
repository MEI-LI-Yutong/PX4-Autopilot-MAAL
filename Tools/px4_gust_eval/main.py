#!/usr/bin/env python3
"""
PX4 Wind Gust Evaluation Framework - Main Entry Point

This is the main entry point for the wind gust evaluation framework.
It provides a simple interface to run the test suite.

Usage:
    python main.py tasks/example_gust_tests.json [--verbose]
"""

import sys
from pathlib import Path
from gust_test_runner import main as runner_main

def main():
    """Main entry point - delegate to gust_test_runner"""
    if len(sys.argv) < 2:
        print("Usage: python main.py <config_file.json> [--verbose]")
        print("\nExample:")
        print("  python main.py tasks/example_gust_tests.json --verbose")
        print("\nAvailable configuration files:")

        tasks_dir = Path(__file__).parent / "tasks"
        if tasks_dir.exists():
            for json_file in tasks_dir.glob("*.json"):
                print(f"  - {json_file}")
        else:
            print("  No task files found in 'tasks' directory")

        sys.exit(1)

    # Pass all arguments to the runner
    runner_main()

if __name__ == "__main__":
    main()
