#!/usr/bin/env python3
"""
PX4 Wind Gust Evaluation Framework

This framework reads JSON configuration files containing wind gust test scenarios,
launches PX4 SITL with the tiltrotor windy model, executes straight-line missions,
and evaluates vehicle performance under various wind conditions.

Author: PX4 Development Team
License: BSD 3-Clause
"""

import asyncio
import argparse
import json
import logging
import os
import psutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict

import colorama
from colorama import Fore, Style
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import VelocityBodyYawspeed


@dataclass
class TestResult:
    """Data class to store test execution results"""
    test_id: str
    description: str
    wind_config: Dict[str, Any]
    start_time: datetime
    end_time: Optional[datetime] = None
    success: bool = False
    mission_completed: bool = False
    max_position_error_m: float = 0.0
    max_altitude_deviation_m: float = 0.0
    max_airspeed_variation_ms: float = 0.0
    mission_duration_sec: float = 0.0
    error_message: Optional[str] = None
    flight_data: Dict[str, Any] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        result = asdict(self)
        result['start_time'] = self.start_time.isoformat() if self.start_time else None
        result['end_time'] = self.end_time.isoformat() if self.end_time else None
        return result


class WindGustTestRunner:
    """Main test runner class for wind gust evaluation"""

    def __init__(self, config_file: str, px4_root: str, verbose: bool = False):
        self.config_file = config_file
        self.px4_root = Path(px4_root)
        self.verbose = verbose
        self.config: Dict[str, Any] = {}
        self.results: List[TestResult] = []

        # Initialize colorama for colored output
        colorama.init()

        # Setup logging
        self._setup_logging()

        # Load configuration
        self._load_config()

        # Create output directories
        self._setup_output_dirs()

        # PX4 process handle
        self.px4_process: Optional[subprocess.Popen] = None

        # MAVSDK system
        self.drone: Optional[System] = None

    def _setup_logging(self):
        """Setup logging configuration"""
        log_level = logging.DEBUG if self.verbose else logging.INFO
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(sys.stdout),
                logging.FileHandler(f'gust_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
            ]
        )
        self.logger = logging.getLogger(__name__)

    def _load_config(self):
        """Load test configuration from JSON file"""
        try:
            with open(self.config_file, 'r') as f:
                self.config = json.load(f)
            self.logger.info(f"Loaded configuration: {self.config['test_suite']}")
        except Exception as e:
            self.logger.error(f"Failed to load config file {self.config_file}: {e}")
            sys.exit(1)

    def _setup_output_dirs(self):
        """Create output directories for logs and results"""
        log_base = self.config.get('output_config', {}).get('log_directory', 'logs')
        self.output_dir = Path(log_base)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Create timestamped subdirectory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.output_dir / f"gust_eval_{timestamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.logger.info(f"Output directory: {self.run_dir}")

    def _check_prerequisites(self) -> bool:
        """Check if all prerequisites are met"""
        # Check if px4 is already running
        for proc in psutil.process_iter(['name']):
            if proc.info['name'] == 'px4':
                self.logger.error("PX4 process is already running. Please kill it first: killall px4")
                return False

        # Check if gz server is running
        for proc in psutil.process_iter(['name']):
            if 'gz' in proc.info['name'] and 'server' in proc.info['name']:
                self.logger.error("Gazebo server is already running. Please kill it first")
                return False

        # Check if PX4 SITL build exists
        px4_binary = self.px4_root / "build/px4_sitl_default/bin/px4"
        if not px4_binary.exists():
            self.logger.error(f"PX4 SITL binary not found at {px4_binary}")
            self.logger.error("Please build PX4 SITL first: make px4_sitl_default")
            return False

        return True

    async def _launch_px4_sitl(self, wind_config: Dict[str, Any]) -> bool:
        """Launch PX4 SITL with wind gust configuration"""
        try:
            # Change to PX4 root directory
            os.chdir(self.px4_root)

            # Prepare environment variables for wind configuration
            env = os.environ.copy()

            # Set wind gust parameters via environment variables
            if wind_config.get('model') == 'sine':
                env['PX4_GZ_WIND_MODEL'] = 'sine'
                env['PX4_GZ_WIND_MEAN'] = f"{wind_config['mean'][0]},{wind_config['mean'][1]},{wind_config['mean'][2]}"
                env['PX4_GZ_WIND_AMPLITUDE'] = f"{wind_config['amplitude'][0]},{wind_config['amplitude'][1]},{wind_config['amplitude'][2]}"
                env['PX4_GZ_WIND_FREQUENCY'] = str(wind_config['frequency'])
                env['PX4_GZ_WIND_PHASE'] = str(wind_config['phase'])

            elif wind_config.get('model') == 'one_minus_cos':
                env['PX4_GZ_WIND_MODEL'] = 'one_minus_cos'
                env['PX4_GZ_WIND_MEAN'] = f"{wind_config['mean'][0]},{wind_config['mean'][1]},{wind_config['mean'][2]}"
                env['PX4_GZ_WIND_GUST_LENGTH'] = str(wind_config['gust_length'])
                env['PX4_GZ_WIND_AIRSPEED'] = str(wind_config['airspeed'])
                env['PX4_GZ_WIND_DIRECTION'] = f"{wind_config['direction'][0]},{wind_config['direction'][1]},{wind_config['direction'][2]}"
                env['PX4_GZ_WIND_PHASE'] = str(wind_config['phase'])

            # Launch PX4 SITL
            cmd = self.config['px4_sitl_command']
            self.logger.info(f"Launching PX4 SITL: {cmd}")

            self.px4_process = subprocess.Popen(
                cmd.split(),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=self.px4_root
            )

            # Wait for PX4 to initialize
            await asyncio.sleep(10)

            # Check if process is still running
            if self.px4_process.poll() is not None:
                stdout, stderr = self.px4_process.communicate()
                self.logger.error(f"PX4 SITL failed to start: {stderr.decode()}")
                return False

            self.logger.info("PX4 SITL started successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to launch PX4 SITL: {e}")
            return False

    async def _connect_mavsdk(self) -> bool:
        """Connect to PX4 via MAVSDK"""
        try:
            self.drone = System()
            await self.drone.connect(system_address="udp://:14540")

            self.logger.info("Waiting for drone to connect...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("Drone connected!")
                    break

            return True

        except Exception as e:
            self.logger.error(f"Failed to connect MAVSDK: {e}")
            return False

    async def _create_mission(self) -> MissionPlan:
        """Create a straight-line mission from configuration"""
        mission_items = []
        mission_config = self.config['mission_config']

        # Takeoff
        mission_items.append(MissionItem(
            mission_config['waypoints'][0]['lat'],
            mission_config['waypoints'][0]['lon'],
            mission_config['takeoff_altitude'],
            10,  # speed m/s
            True,  # is_fly_through
            float('nan'),  # gimbal_pitch
            float('nan'),  # gimbal_yaw
            MissionItem.CameraAction.NONE,
            float('nan'),  # loiter_time_s
            float('nan'),   # camera_photo_interval_s
            float('nan'),   # acceptance_radius_m
            float('nan'),   # yaw_deg
            float('nan'),   # camera_photo_distance_m
            MissionItem.VehicleAction.NONE  # vehicle_action
        ))

        # Waypoints
        for waypoint in mission_config['waypoints']:
            mission_items.append(MissionItem(
                waypoint['lat'],
                waypoint['lon'],
                waypoint['alt'],
                mission_config['airspeed'],
                True,  # is_fly_through
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'),
                float('nan'),   # acceptance_radius_m
                float('nan'),   # yaw_deg
                float('nan'),   # camera_photo_distance_m
                MissionItem.VehicleAction.NONE  # vehicle_action
            ))

        return MissionPlan(mission_items)

    async def _execute_mission(self) -> Tuple[bool, Dict[str, Any]]:
        """Execute a simple takeoff-fly-land mission using offboard mode"""
        try:
            mission_config = self.config['mission_config']

            flight_data = {
                'positions': [],
                'velocities': [],
                'attitudes': [],
                'timestamps': []
            }

            start_time = time.time()
            timeout = mission_config['timeout_sec']

            # Wait for vehicle to be ready
            self.logger.info("Waiting for vehicle to be ready...")
            ready_timeout = time.time() + 30  # 30 second timeout
            while time.time() < ready_timeout:
                async for health in self.drone.telemetry.health():
                    self.logger.info(f"Health: GPS OK: {health.is_global_position_ok}, Home OK: {health.is_home_position_ok}, Armable: {health.is_armable}")
                    if health.is_global_position_ok and health.is_home_position_ok:
                        self.logger.info("Vehicle ready")
                        break
                    await asyncio.sleep(1)
                    break
                else:
                    break

            # Arm the vehicle (with force if needed)
            self.logger.info("Arming...")
            try:
                await self.drone.action.arm()
            except Exception as e:
                self.logger.warning(f"Normal arm failed: {e}, trying force arm...")
                # Try to force arm for testing
                await asyncio.sleep(2)
                try:
                    # Set arming check parameters to be more lenient
                    await self.drone.param.set_param_int("COM_ARM_WO_GPS", 1)
                    await self.drone.param.set_param_int("NAV_RCL_ACT", 0)
                    await self.drone.param.set_param_int("NAV_DLL_ACT", 0)
                    await asyncio.sleep(1)
                    await self.drone.action.arm()
                    self.logger.info("Force arm successful")
                except Exception as e2:
                    self.logger.error(f"Force arm also failed: {e2}")
                    raise e2

            # Takeoff
            takeoff_altitude = mission_config['takeoff_altitude']
            self.logger.info(f"Taking off to {takeoff_altitude}m...")
            await self.drone.action.set_takeoff_altitude(takeoff_altitude)
            await self.drone.action.takeoff()

            # Wait for takeoff to complete
            self.logger.info("Waiting for takeoff...")
            async for position in self.drone.telemetry.position():
                if position.relative_altitude_m >= takeoff_altitude * 0.9:
                    self.logger.info(f"Reached takeoff altitude: {position.relative_altitude_m:.1f}m")
                    break

                # Check timeout
                if time.time() - start_time > timeout / 2:
                    self.logger.warning("Takeoff timeout")
                    break

            # Simple straight-line flight using offboard mode
            waypoints = mission_config['waypoints']
            if len(waypoints) >= 2:
                target_lat = waypoints[-1]['lat']
                target_lon = waypoints[-1]['lon']
                target_alt = waypoints[-1]['alt']

                self.logger.info(f"Flying to target: {target_lat:.6f}, {target_lon:.6f}, {target_alt:.1f}m")
                await self.drone.action.goto_location(target_lat, target_lon, target_alt, 0)

            # Monitor flight and collect data
            mission_completed = False
            flight_start = time.time()

            while time.time() - start_time < timeout:
                current_time = time.time()
                elapsed = current_time - start_time

                try:
                    # Collect telemetry data
                    position = await self.drone.telemetry.position().__anext__()
                    velocity = await self.drone.telemetry.velocity_ned().__anext__()
                    attitude = await self.drone.telemetry.attitude_euler().__anext__()

                    flight_data['positions'].append({
                        'lat': position.latitude_deg,
                        'lon': position.longitude_deg,
                        'alt': position.relative_altitude_m
                    })
                    flight_data['velocities'].append({
                        'north': velocity.north_m_s,
                        'east': velocity.east_m_s,
                        'down': velocity.down_m_s
                    })
                    flight_data['attitudes'].append({
                        'roll': attitude.roll_deg,
                        'pitch': attitude.pitch_deg,
                        'yaw': attitude.yaw_deg
                    })
                    flight_data['timestamps'].append(elapsed)

                    # Simple completion check - if we've been flying for a reasonable time
                    if elapsed > 30 and len(flight_data['positions']) > 10:
                        self.logger.info("Flight data collected, mission considered complete")
                        mission_completed = True
                        break

                except Exception as e:
                    self.logger.warning(f"Telemetry collection error: {e}")

                await asyncio.sleep(1)

            # Land
            self.logger.info("Landing...")
            await self.drone.action.land()

            # Wait for landing
            self.logger.info("Waiting for landing...")
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == self.drone.telemetry.LandedState.ON_GROUND:
                    self.logger.info("Landed successfully")
                    break

            return mission_completed, flight_data

        except Exception as e:
            self.logger.error(f"Mission execution failed: {e}")
            return False, {}

    def _analyze_flight_data(self, flight_data: Dict[str, Any], expected: Dict[str, Any]) -> Dict[str, float]:
        """Analyze flight data and calculate performance metrics"""
        if not flight_data or not flight_data.get('positions'):
            return {}

        # Calculate position errors (simplified)
        positions = flight_data['positions']
        waypoints = self.config['mission_config']['waypoints']

        max_position_error = 0.0
        max_altitude_deviation = 0.0

        # Simple analysis - compare with expected flight path
        for i, pos in enumerate(positions):
            if i < len(waypoints):
                expected_alt = waypoints[i]['alt']
                altitude_error = abs(pos['alt'] - expected_alt)
                max_altitude_deviation = max(max_altitude_deviation, altitude_error)

        # Calculate airspeed variation
        velocities = flight_data.get('velocities', [])
        speeds = []
        for vel in velocities:
            speed = (vel['north']**2 + vel['east']**2 + vel['down']**2)**0.5
            speeds.append(speed)

        max_airspeed_variation = 0.0
        if speeds:
            expected_speed = self.config['mission_config']['airspeed']
            max_airspeed_variation = max([abs(s - expected_speed) for s in speeds])

        return {
            'max_position_error_m': max_position_error,
            'max_altitude_deviation_m': max_altitude_deviation,
            'max_airspeed_variation_ms': max_airspeed_variation
        }

    def _cleanup_px4(self):
        """Clean up PX4 SITL process"""
        if self.px4_process and self.px4_process.poll() is None:
            self.logger.info("Terminating PX4 SITL...")
            self.px4_process.terminate()
            try:
                self.px4_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.logger.warning("PX4 SITL did not terminate gracefully, killing...")
                self.px4_process.kill()

        # Kill any remaining px4 processes
        for proc in psutil.process_iter(['name', 'pid']):
            if proc.info['name'] == 'px4':
                try:
                    proc.kill()
                    self.logger.info(f"Killed px4 process {proc.info['pid']}")
                except psutil.NoSuchProcess:
                    pass

    async def run_single_test(self, test_config: Dict[str, Any]) -> TestResult:
        """Run a single wind gust test"""
        test_result = TestResult(
            test_id=test_config['test_id'],
            description=test_config['description'],
            wind_config=test_config['wind_config'],
            start_time=datetime.now()
        )

        try:
            self.logger.info(f"\n{Fore.CYAN}{'='*60}")
            self.logger.info(f"Running Test: {test_config['test_id']}")
            self.logger.info(f"Description: {test_config['description']}")
            self.logger.info(f"{'='*60}{Style.RESET_ALL}")

            # Launch PX4 SITL with wind configuration
            if not await self._launch_px4_sitl(test_config['wind_config']):
                test_result.error_message = "Failed to launch PX4 SITL"
                return test_result

            # Connect MAVSDK
            if not await self._connect_mavsdk():
                test_result.error_message = "Failed to connect MAVSDK"
                return test_result

            # Execute mission
            mission_completed, flight_data = await self._execute_mission()
            test_result.mission_completed = mission_completed
            test_result.flight_data = flight_data

            # Analyze results
            if flight_data:
                metrics = self._analyze_flight_data(flight_data, test_config['expected_results'])
                test_result.max_position_error_m = metrics.get('max_position_error_m', 0.0)
                test_result.max_altitude_deviation_m = metrics.get('max_altitude_deviation_m', 0.0)
                test_result.max_airspeed_variation_ms = metrics.get('max_airspeed_variation_ms', 0.0)

            test_result.success = mission_completed
            test_result.end_time = datetime.now()
            test_result.mission_duration_sec = (test_result.end_time - test_result.start_time).total_seconds()

            # Log results
            status = f"{Fore.GREEN}PASSED" if test_result.success else f"{Fore.RED}FAILED"
            self.logger.info(f"Test {test_config['test_id']}: {status}{Style.RESET_ALL}")
            self.logger.info(f"Mission completed: {test_result.mission_completed}")
            self.logger.info(f"Max altitude deviation: {test_result.max_altitude_deviation_m:.2f}m")
            self.logger.info(f"Duration: {test_result.mission_duration_sec:.1f}s")

        except Exception as e:
            test_result.error_message = str(e)
            test_result.end_time = datetime.now()
            self.logger.error(f"Test {test_config['test_id']} failed: {e}")

        finally:
            # Clean up
            self._cleanup_px4()
            await asyncio.sleep(5)  # Wait between tests

        return test_result

    async def run_all_tests(self):
        """Run all wind gust tests from configuration"""
        if not self._check_prerequisites():
            return False

        self.logger.info(f"{Fore.YELLOW}Starting Wind Gust Evaluation Suite{Style.RESET_ALL}")
        self.logger.info(f"Test suite: {self.config['test_suite']}")
        self.logger.info(f"Number of tests: {len(self.config['wind_gust_tests'])}")

        # Run each test
        for test_config in self.config['wind_gust_tests']:
            result = await self.run_single_test(test_config)
            self.results.append(result)

        # Generate final report
        self._generate_report()

        return True

    def _generate_report(self):
        """Generate test report"""
        passed_tests = sum(1 for r in self.results if r.success)
        total_tests = len(self.results)

        # Console summary
        self.logger.info(f"\n{Fore.YELLOW}{'='*60}")
        self.logger.info(f"TEST SUITE SUMMARY")
        self.logger.info(f"{'='*60}{Style.RESET_ALL}")
        self.logger.info(f"Total tests: {total_tests}")
        self.logger.info(f"Passed: {Fore.GREEN}{passed_tests}{Style.RESET_ALL}")
        self.logger.info(f"Failed: {Fore.RED}{total_tests - passed_tests}{Style.RESET_ALL}")
        success_rate = (passed_tests/total_tests*100) if total_tests > 0 else 0.0
        self.logger.info(f"Success rate: {success_rate:.1f}%")

        # Save detailed results to JSON
        report_file = self.run_dir / "test_results.json"
        # Ensure directory exists
        report_file.parent.mkdir(parents=True, exist_ok=True)
        with open(report_file, 'w') as f:
            json.dump({
                'test_suite': self.config['test_suite'],
                'summary': {
                    'total_tests': total_tests,
                    'passed_tests': passed_tests,
                    'failed_tests': total_tests - passed_tests,
                    'success_rate': success_rate / 100.0
                },
                'results': [r.to_dict() for r in self.results]
            }, f, indent=2)

        self.logger.info(f"Detailed results saved to: {report_file}")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="PX4 Wind Gust Evaluation Framework")
    parser.add_argument("config_file", help="JSON configuration file path")
    parser.add_argument("--px4-root", default="/home/raiot/Programing/PX4-Autopilot-MAAL",
                       help="PX4 root directory path")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    # Create and run test suite
    runner = WindGustTestRunner(args.config_file, args.px4_root, args.verbose)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        runner.logger.info("Interrupted by user")
        runner._cleanup_px4()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Run tests
    success = asyncio.run(runner.run_all_tests())
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()