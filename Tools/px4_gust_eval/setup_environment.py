#!/usr/bin/env python3
"""
Environment Setup and Validation Script for PX4 Wind Gust Evaluation Framework

This script helps setup and validate the environment for running wind gust tests.
It checks dependencies, builds required components, and validates the setup.

Usage:
    python setup_environment.py [--check-only] [--build] [--test-basic]
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path
import psutil

class EnvironmentSetup:
    def __init__(self, px4_root: str):
        self.px4_root = Path(px4_root)
        self.errors = []
        self.warnings = []

    def check_px4_build(self) -> bool:
        """Check if PX4 SITL is built"""
        px4_binary = self.px4_root / "build/px4_sitl_default/bin/px4"
        if px4_binary.exists():
            print("✓ PX4 SITL binary found")
            return True
        else:
            self.errors.append("PX4 SITL binary not found")
            print("✗ PX4 SITL binary not found")
            return False

    def check_gazebo_models(self) -> bool:
        """Check if required Gazebo models exist"""
        tiltrotor_model = self.px4_root / "Tools/simulation/gz/models/tiltrotor"
        if tiltrotor_model.exists():
            print("✓ Tiltrotor model found")
            return True
        else:
            self.errors.append("Tiltrotor model not found")
            print("✗ Tiltrotor model not found")
            return False

    def check_wind_gust_plugin(self) -> bool:
        """Check if wind gust plugin is built"""
        plugin_so = self.px4_root / "build/px4_sitl_default/build_gz_plugins/WindGustSystem/libWindGustSystem.so"
        if plugin_so.exists():
            print("✓ Wind gust plugin built")
            return True
        else:
            self.warnings.append("Wind gust plugin may not be built")
            print("⚠ Wind gust plugin build status unclear")
            return False

    def check_python_dependencies(self) -> bool:
        """Check if required Python packages are available"""
        required_packages = ['mavsdk', 'pymavlink', 'psutil', 'colorama']
        missing = []

        for package in required_packages:
            try:
                __import__(package)
                print(f"✓ {package} available")
            except ImportError:
                missing.append(package)
                print(f"✗ {package} not available")

        if missing:
            self.errors.append(f"Missing Python packages: {', '.join(missing)}")
            return False
        return True

    def check_running_processes(self) -> bool:
        """Check for conflicting running processes"""
        conflicts = []
        for proc in psutil.process_iter(['name']):
            if proc.info['name'] in ['px4', 'gzserver', 'gzclient']:
                conflicts.append(proc.info['name'])

        if conflicts:
            self.warnings.append(f"Conflicting processes running: {', '.join(set(conflicts))}")
            print(f"⚠ Warning: Found running processes: {', '.join(set(conflicts))}")
            print("  Consider stopping them before running tests:")
            print("  killall px4 gzserver gzclient")
            return False
        else:
            print("✓ No conflicting processes found")
            return True

    def build_px4_sitl(self) -> bool:
        """Build PX4 SITL with required components"""
        print("Building PX4 SITL with Gazebo and wind gust plugin...")
        try:
            os.chdir(self.px4_root)
            cmd = ["make", "px4_sitl", "gz_tiltrotor_windy"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)

            if result.returncode == 0:
                print("✓ PX4 SITL build successful")
                return True
            else:
                print("✗ PX4 SITL build failed")
                print(f"Error output: {result.stderr}")
                self.errors.append("PX4 SITL build failed")
                return False

        except subprocess.TimeoutExpired:
            print("✗ PX4 SITL build timed out")
            self.errors.append("PX4 SITL build timed out")
            return False
        except Exception as e:
            print(f"✗ PX4 SITL build error: {e}")
            self.errors.append(f"PX4 SITL build error: {e}")
            return False

    def install_python_deps(self) -> bool:
        """Install Python dependencies"""
        print("Installing Python dependencies...")
        try:
            # Use uv if available, otherwise pip
            if subprocess.run(["which", "uv"], capture_output=True).returncode == 0:
                cmd = ["uv", "pip", "install", "-r", "requirements.txt"]
            else:
                cmd = ["pip", "install", "-r", "requirements.txt"]

            # Create requirements.txt if it doesn't exist
            requirements_file = Path("requirements.txt")
            if not requirements_file.exists():
                with open(requirements_file, 'w') as f:
                    f.write("mavsdk>=3.10.2\n")
                    f.write("pymavlink>=2.4.49\n")
                    f.write("psutil\n")
                    f.write("colorama\n")

            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                print("✓ Python dependencies installed")
                return True
            else:
                print("✗ Failed to install Python dependencies")
                print(f"Error: {result.stderr}")
                return False

        except Exception as e:
            print(f"✗ Error installing Python dependencies: {e}")
            return False

    def run_basic_test(self) -> bool:
        """Run a basic validation test"""
        print("Running basic validation test...")
        try:
            cmd = ["python", "main.py", "tasks/basic_validation_tests.json", "--verbose"]
            result = subprocess.run(cmd, timeout=600)  # 10 minute timeout

            if result.returncode == 0:
                print("✓ Basic validation test passed")
                return True
            else:
                print("✗ Basic validation test failed")
                return False

        except subprocess.TimeoutExpired:
            print("✗ Basic validation test timed out")
            return False
        except Exception as e:
            print(f"✗ Error running basic test: {e}")
            return False

    def run_full_check(self) -> bool:
        """Run full environment check"""
        print("=" * 60)
        print("PX4 Wind Gust Evaluation Framework - Environment Check")
        print("=" * 60)

        all_good = True

        print("\n1. Checking PX4 SITL build...")
        all_good &= self.check_px4_build()

        print("\n2. Checking Gazebo models...")
        all_good &= self.check_gazebo_models()

        print("\n3. Checking wind gust plugin...")
        self.check_wind_gust_plugin()  # Non-critical

        print("\n4. Checking Python dependencies...")
        all_good &= self.check_python_dependencies()

        print("\n5. Checking for conflicting processes...")
        self.check_running_processes()  # Non-critical

        return all_good

    def print_summary(self):
        """Print setup summary"""
        print("\n" + "=" * 60)
        print("SETUP SUMMARY")
        print("=" * 60)

        if self.errors:
            print(f"\nERRORS ({len(self.errors)}):")
            for error in self.errors:
                print(f"  ✗ {error}")

        if self.warnings:
            print(f"\nWARNINGS ({len(self.warnings)}):")
            for warning in self.warnings:
                print(f"  ⚠ {warning}")

        if not self.errors:
            print("\n✓ Environment setup appears to be correct!")
            print("\nYou can now run wind gust tests:")
            print("  python main.py tasks/basic_validation_tests.json")
            print("  python main.py tasks/example_gust_tests.json --verbose")
        else:
            print(f"\n✗ Found {len(self.errors)} errors that need to be resolved")
            print("\nSuggested actions:")
            if "PX4 SITL binary not found" in self.errors:
                print("  - Run: python setup_environment.py --build")
            if any("Python package" in error for error in self.errors):
                print("  - Install dependencies: uv pip install -r requirements.txt")


def main():
    parser = argparse.ArgumentParser(description="Setup PX4 Wind Gust Evaluation Environment")
    parser.add_argument("--px4-root", default="/home/raiot/Programing/PX4-Autopilot-MAAL",
                       help="PX4 root directory")
    parser.add_argument("--check-only", action="store_true",
                       help="Only check environment, don't build anything")
    parser.add_argument("--build", action="store_true",
                       help="Build PX4 SITL and install dependencies")
    parser.add_argument("--test-basic", action="store_true",
                       help="Run basic validation test after setup")

    args = parser.parse_args()

    setup = EnvironmentSetup(args.px4_root)

    if args.build and not args.check_only:
        print("Building environment...")
        os.chdir(Path(args.px4_root) / "Tools/px4_gust_eval")
        setup.install_python_deps()
        setup.build_px4_sitl()

    # Always run check
    success = setup.run_full_check()

    if args.test_basic and success:
        os.chdir(Path(args.px4_root) / "Tools/px4_gust_eval")
        setup.run_basic_test()

    setup.print_summary()

    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()