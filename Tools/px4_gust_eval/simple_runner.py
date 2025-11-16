#!/usr/bin/env python3
"""
Simplified PX4 Wind Gust Test Runner

Goals:
- Minimal, reliable process handling (inspired by test/mavsdk_tests/mavsdk_test_runner.py)
- Small, clear mission flow (arm → takeoff → wait → land)
- JSON-driven configuration while tolerating extra fields
- uv-friendly (no external entrypoint wiring required)

Usage (examples):
- uv run Tools/px4_gust_eval/main.py Tools/px4_gust_eval/tasks/example_gust_tests.json --verbose
- uv run Tools/px4_gust_eval/simple_runner.py Tools/px4_gust_eval/tasks/basic_validation_tests.json
"""

import argparse
import asyncio
import json
import logging
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import psutil  # type: ignore
from mavsdk import System  # type: ignore
import shlex
import math
import xml.etree.ElementTree as ET
import csv
import asyncio.subprocess
import re


# ----------------------------
# Process helpers (minimal)
# ----------------------------

def is_running(process_name: str) -> bool:
    for proc in psutil.process_iter(attrs=["name"]):
        if (proc.info.get("name") or "").lower() == process_name.lower():
            return True
    return False


def check_ready(build_dir: str, simulator: str) -> bool:
    ok = True
    if is_running("px4"):
        print("px4 process already running. Run `killall px4` and retry.")
        ok = False
    if not (Path(build_dir) / "bin/px4").is_file():
        print("PX4 SITL is not built. Build: `make px4_sitl_default` (or CI recipe). Build directory: ", build_dir)
        ok = False
    if simulator == "gazebo":
        # Gazebo and Gazebo-classic
        if is_running("gz") or is_running("gzserver") or is_running("gz-sim"):
            print("Gazebo server appears running (gz/gzserver). Close it and retry.")
            ok = False
        if is_running("gzclient") or is_running("gz-gui"):
            print("Gazebo client appears running (gzclient). Close it and retry.")
            ok = False
    return ok


# ----------------------------
# Wind env mapping (optional)
# ----------------------------

def wind_env_from_config(wind_config: Dict[str, Any]) -> Dict[str, str]:
    """Map a simple wind_config into PX4/Gazebo env vars. Unknown fields are ignored."""
    env: Dict[str, str] = {}
    if not wind_config:
        return env

    model = wind_config.get("model")
    if model:
        env["PX4_GZ_WIND_MODEL"] = str(model)

    def vec3_to_str(key: str, env_key: str):
        v = wind_config.get(key)
        if isinstance(v, list) and len(v) == 3:
            env[env_key] = ",".join(str(x) for x in v)

    vec3_to_str("mean", "PX4_GZ_WIND_MEAN")
    vec3_to_str("amplitude", "PX4_GZ_WIND_AMPLITUDE")
    vec3_to_str("direction", "PX4_GZ_WIND_DIRECTION")

    for k, env_k in (
        ("gust_length", "PX4_GZ_WIND_GUST_LENGTH"),
        ("airspeed", "PX4_GZ_WIND_AIRSPEED"),
        ("frequency", "PX4_GZ_WIND_FREQUENCY"),
        ("phase", "PX4_GZ_WIND_PHASE"),
        ("A0", "PX4_GZ_WIND_A0"),
        ("T", "PX4_GZ_WIND_T"),
    ):
        if k in wind_config:
            env[env_k] = str(wind_config[k])

    return env


# ----------------------------
# Core runner
# ----------------------------

class SimpleGustRunner:
    def __init__(self, config: Dict[str, Any], px4_root: Path, build_dir: Path, verbose: bool) -> None:
        self.config = config
        self.px4_root = px4_root
        self.build_dir = build_dir
        self.verbose = verbose

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(self.config.get("output_config", {}).get("log_directory", "Tools/px4_gust_eval/logs")) / f"run_{ts}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        log_level = logging.DEBUG if verbose else logging.INFO
        logging.basicConfig(
            level=log_level,
            format="%(asctime)s %(levelname)s %(message)s",
            handlers=[
                logging.StreamHandler(sys.stdout),
                logging.FileHandler(self.run_dir / f"gust_run_{ts}.log"),
            ],
        )
        self.logger = logging.getLogger("gust.simple")

        self.proc: Optional[subprocess.Popen] = None
        self.drone: Optional[System] = None
        self._launch_time: float = 0.0
        self._pids_to_kill: set[int] = set()
        self._csv_task: Optional[asyncio.Task] = None
        self._wind_task: Optional[asyncio.Task] = None
        self._latest_wind_ms: Optional[float] = None
        self._latest_wind_xyz: Optional[tuple] = None
        self._wind_last_update: float = 0.0
        self._recording: bool = False
        self._csv_file = None
        self._csv_writer = None
        self.current_setpoint = {"lat": None, "lon": None, "alt_amsl": None}
        self.wind_fill_zero: bool = bool(self.config.get("wind_fill_zero", True))
        self.wind_stale_sec: float = float(self.config.get("wind_stale_sec", 1.0))

    # ---- lifecycle ----
    async def _wait_until_armable(self, timeout_sec: int, require_gps: bool = True) -> bool:
        """Wait until vehicle reports armable. Logs health periodically."""
        assert self.drone is not None
        loop = asyncio.get_event_loop()
        deadline = loop.time() + max(5, timeout_sec)
        last_log = 0.0
        while loop.time() < deadline:
            try:
                health = await self.drone.telemetry.health().__anext__()
            except Exception:
                await asyncio.sleep(0.5)
                continue

            ok_gps = getattr(health, "is_global_position_ok", False)
            ok_home = getattr(health, "is_home_position_ok", False)
            armable = getattr(health, "is_armable", False)

            now = loop.time()
            if now - last_log > 2.0:
                self.logger.info(
                    f"Health: armable={armable} gps_ok={ok_gps} home_ok={ok_home}"
                )
                last_log = now

            if armable and (not require_gps or (ok_gps and ok_home)):
                return True

            await asyncio.sleep(0.8)

        return False

    async def _set_params(self, params: Dict[str, Any]) -> None:
        assert self.drone is not None
        for name, value in params.items():
            try:
                if isinstance(value, float):
                    await self.drone.param.set_param_float(name, float(value))
                else:
                    await self.drone.param.set_param_int(name, int(value))
                await asyncio.sleep(0.05)
            except Exception as e:
                self.logger.debug(f"Param set failed {name}={value}: {e}")

    async def launch_px4(self, env_overrides: Dict[str, str]) -> bool:
        cmd = self.config.get("px4_sitl_command")
        if not cmd:
            self.logger.error("Missing `px4_sitl_command` in config")
            return False

        env = os.environ.copy()
        env.update(env_overrides)

        # Parse command: extract environment variables prefix (KEY=VALUE KEY2=VALUE2 ... actual_command)
        parts = shlex.split(cmd)
        cmd_parts = []
        for part in parts:
            if '=' in part and not part.startswith('-'):
                # Looks like an environment variable
                key, value = part.split('=', 1)
                env[key] = value
            else:
                # Actual command starts here
                cmd_parts.append(part)

        if not cmd_parts:
            self.logger.error("No actual command found after parsing environment variables")
            return False

        try:
            self.logger.info(f"Launching PX4: {' '.join(cmd_parts)} (with env overrides)")
            self.proc = subprocess.Popen(
                cmd_parts,
                cwd=self.px4_root,
                env=env,
                stdout=subprocess.PIPE if self.verbose else subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,  # new process group for easier cleanup
            )
            self._launch_time = time.time()
            await asyncio.sleep(2.0)
            self._track_child_pids()
            self._track_gazebo_pids()
            await asyncio.sleep(8)  # basic startup delay
            if self.proc.poll() is not None:
                out = self.proc.stdout.read().decode("utf-8", errors="ignore") if self.proc.stdout else ""
                self.logger.error(f"PX4 failed to start. Output: {out[-400:]}…")
                return False
            return True
        except Exception as e:
            self.logger.error(f"PX4 start error: {e}")
            return False

    async def connect_mavsdk(self, url: str) -> bool:
        try:
            self.drone = System()
            await self.drone.connect(system_address=url)
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("MAVSDK connected")
                    break
            return True
        except Exception as e:
            self.logger.error(f"MAVSDK connect error: {e}")
            return False

    async def run_simple_mission(self, mission_cfg: Dict[str, Any]) -> bool:
        assert self.drone is not None

        takeoff_alt = float(mission_cfg.get("takeoff_altitude", 10.0))
        flight_sec = int(mission_cfg.get("flight_duration_sec", 30))
        timeout_sec = int(mission_cfg.get("timeout_sec", max(90, 2 * flight_sec)))
        ready_timeout = int(mission_cfg.get("ready_timeout_sec", 60))
        force_arm = bool(mission_cfg.get("force_arm_if_unready", True))
        advance_m = float(mission_cfg.get("advance_distance_m", 100.0))
        advance_axis = str(mission_cfg.get("advance_axis", "ned_y")).lower()  # ned_x (north) or ned_y (east)

        start = asyncio.get_event_loop().time()

        # Pre-arm readiness wait
        ready = await self._wait_until_armable(timeout_sec=ready_timeout, require_gps=True)
        if not ready:
            self.logger.warning("Vehicle not armable within timeout")
            # Optional prearm parameter relaxations for SITL
            if force_arm:
                self.logger.info("Applying minimal prearm parameters for SITL (COM_ARM_WO_GPS=1)…")
                await self._set_params({
                    "COM_ARM_WO_GPS": 1,
                })

        # Arm & takeoff
        try:
            await self.drone.action.set_takeoff_altitude(takeoff_alt)
            await self.drone.action.arm()
            await self.drone.action.takeoff()
        except Exception as e:
            # One retry after relaxing a couple of non-critical failsafes in SITL
            if force_arm:
                self.logger.warning(f"Arm/Takeoff failed: {e}; retrying with relaxed params for SITL…")
                await self._set_params({
                    "COM_ARM_WO_GPS": 1,
                    "NAV_RCL_ACT": 0,
                    "NAV_DLL_ACT": 0,
                })
                await asyncio.sleep(1.0)
                try:
                    await self.drone.action.arm()
                    await self.drone.action.takeoff()
                except Exception as e2:
                    self.logger.error(f"Takeoff failed after retry: {e2}")
                    return False
            else:
                self.logger.error(f"Takeoff failed: {e}")
                return False

        # Wait until airborne and at/near takeoff altitude, before any forward advance
        try:
            loop = asyncio.get_event_loop()
            deadline = loop.time() + max(20, timeout_sec // 2)
            in_air = False
            at_alt = False
            while loop.time() < deadline:
                # Check in_air
                try:
                    ia = await self.drone.telemetry.in_air().__anext__()
                    in_air = bool(ia)
                except Exception:
                    in_air = False

                # Check altitude
                try:
                    pos = await self.drone.telemetry.position().__anext__()
                    if pos.relative_altitude_m >= 0.9 * takeoff_alt:
                        at_alt = True
                except Exception:
                    at_alt = False

                if in_air and at_alt:
                    break
                await asyncio.sleep(0.3)

            if not (in_air and at_alt):
                self.logger.error("Did not reach takeoff altitude within timeout after takeoff command")
                return False

            # Optional short hover to stabilize altitude/attitude before advancing
            hover_sec = int(mission_cfg.get("advance_after_hover_sec", 2))
            if hover_sec > 0:
                hover_deadline = loop.time() + hover_sec
                while loop.time() < hover_deadline:
                    try:
                        vned = await self.drone.telemetry.velocity_ned().__anext__()
                        # Ensure vertical speed is small (stabilized)
                        if abs(getattr(vned, "down_m_s", 0.0)) < 0.5:
                            await asyncio.sleep(0.2)
                        else:
                            # Extend hover window slightly if still moving vertically fast
                            hover_deadline = max(hover_deadline, loop.time() + 0.5)
                            await asyncio.sleep(0.2)
                    except Exception:
                        await asyncio.sleep(0.2)
        except Exception as e:
            self.logger.error(f"Error waiting for takeoff: {e}")
            return False

        # Advance in X (NED North) or Y (NED East) by advance_m, then land
        try:
            # Get current global position for simple geodesic offset
            pos = await self.drone.telemetry.position().__anext__()
            lat0 = float(pos.latitude_deg)
            lon0 = float(pos.longitude_deg)
            # Use absolute altitude (AMSL) for goto_location to avoid ground-hugging
            alt_amsl = float(getattr(pos, "absolute_altitude_m", 0.0))
            if not alt_amsl or math.isnan(alt_amsl):
                # Fallback: approximate with relative + a guessed home AMSL (not ideal, but prevents zero AMSL)
                alt_amsl = max(10.0, float(getattr(pos, "relative_altitude_m", takeoff_alt)) + 450.0)

            dn = advance_m if advance_axis == "ned_x" else 0.0
            de = advance_m if advance_axis == "ned_y" else 0.0

            # Convert local N/E offset to lat/lon
            R = 6378137.0
            d_lat = (dn / R) * 180.0 / math.pi
            d_lon = (de / (R * math.cos(math.radians(lat0)))) * 180.0 / math.pi
            lat_t = lat0 + d_lat
            lon_t = lon0 + d_lon

            # Update current setpoint for logging
            self.current_setpoint = {"lat": lat_t, "lon": lon_t, "alt_amsl": alt_amsl}
            self.logger.info(f"Advancing {advance_m:.1f} m along {advance_axis} to lat={lat_t:.7f} lon={lon_t:.7f} at AMSL {alt_amsl:.1f} m")
            await self.drone.action.goto_location(lat_t, lon_t, alt_amsl, 90.0)

            # Wait until close to target or timeout (half remaining time)
            wait_deadline = asyncio.get_event_loop().time() + max(15, timeout_sec // 2)
            reached = False
            while asyncio.get_event_loop().time() < wait_deadline:
                p = await self.drone.telemetry.position().__anext__()
                # compute ground distance
                d_n = (math.radians(p.latitude_deg - lat_t)) * R
                d_e = (math.radians(p.longitude_deg - lon_t)) * R * math.cos(math.radians((p.latitude_deg + lat_t) / 2.0))
                dist = math.hypot(d_n, d_e)
                if dist < 5.0:
                    reached = True
                    break
                await asyncio.sleep(0.5)
            if not reached:
                self.logger.info("Advance timeout; proceeding to land")
        except Exception as e:
            self.logger.warning(f"Advance move skipped due to error: {e}")

        # Land & wait a bit
        try:
            await self.drone.action.land()
        except Exception as e:
            self.logger.warning(f"Land command failed: {e}")

        await asyncio.sleep(min(30, max(5, flight_sec // 3)))
        return True

    def stop_px4(self) -> None:
        # Try to terminate whole process group (px4 + gazebo)
        if self.proc:
            try:
                if self.proc.poll() is None:
                    self.logger.info("Stopping PX4/Gazebo (process group)")
                    if hasattr(os, "killpg"):
                        os.killpg(self.proc.pid, signal.SIGINT)
                        try:
                            self.proc.wait(timeout=6)
                        except Exception:
                            os.killpg(self.proc.pid, signal.SIGTERM)
                            try:
                                self.proc.wait(timeout=4)
                            except Exception:
                                os.killpg(self.proc.pid, signal.SIGKILL)
                    else:
                        # Fallback: terminate the main process
                        self.proc.terminate()
                        try:
                            self.proc.wait(timeout=6)
                        except Exception:
                            self.proc.kill()
            except ProcessLookupError:
                pass

        # Terminate tracked children
        self._kill_tracked_pids()

        # Last resort: scan for leftover gazebo processes from this run and kill
        self._track_gazebo_pids()  # refresh
        self._kill_tracked_pids()

    def _track_child_pids(self) -> None:
        if not self.proc:
            return
        try:
            p = psutil.Process(self.proc.pid)
            for child in p.children(recursive=True):
                self._pids_to_kill.add(child.pid)
        except Exception:
            pass

    def _track_gazebo_pids(self) -> None:
        # Heuristics: processes named gz, gzserver, gzclient, gazebo; started after we launched
        names = {"gz", "gzserver", "gzclient", "gazebo", "gz-gui", "gz-sim"}
        patterns = {"sim", "gazebo", "gzserver", "gzclient", "gz-gui", "gz sim"}
        try:
            for proc in psutil.process_iter(["pid", "name", "create_time", "cmdline"]):
                name = (proc.info.get("name") or "").lower()
                if name in names:
                    ctime = float(proc.info.get("create_time") or 0.0)
                    if self._launch_time and ctime and ctime >= self._launch_time - 2:
                        self._pids_to_kill.add(proc.info["pid"])
                        continue
                # Check cmdline patterns
                try:
                    cmd = " ".join(proc.info.get("cmdline") or [])
                except Exception:
                    cmd = ""
                lcmd = cmd.lower()
                if any(p in lcmd for p in patterns):
                    ctime = float(proc.info.get("create_time") or 0.0)
                    if self._launch_time and ctime and ctime >= self._launch_time - 2:
                        self._pids_to_kill.add(proc.info["pid"])
        except Exception:
            pass

    def _kill_tracked_pids(self) -> None:
        for pid in list(self._pids_to_kill):
            try:
                proc = psutil.Process(pid)
                if not proc.is_running():
                    self._pids_to_kill.discard(pid)
                    continue
                # Try graceful then force
                try:
                    proc.terminate()
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            except psutil.NoSuchProcess:
                pass
            finally:
                self._pids_to_kill.discard(pid)

    # ---- orchestration ----
    async def run(self) -> int:
        simulator = self.config.get("simulator", "gazebo")
        if not check_ready(str(self.build_dir), simulator):
            return 2

        # missions/tests
        tests: List[Dict[str, Any]] = []
        if "wind_gust_tests" in self.config:
            tests = list(self.config.get("wind_gust_tests", []))
        elif "tests" in self.config:
            tests = list(self.config.get("tests", []))
        else:
            # single-run (no list)
            tests = [{
                "test_id": self.config.get("test_suite", "single"),
                "description": self.config.get("description", "single run"),
                "wind_config": self.config.get("wind_config", {}),
            }]

        mission_cfg = self.config.get("mission_config", self.config.get("mission", {}))
        mavlink_url = self.config.get("mavlink_url", "udp://:14540")

        results: List[Dict[str, Any]] = []

        for t in tests:
            test_id = t.get("test_id", "test")
            self.logger.info(f"Starting test: {test_id}")
            wind_env = wind_env_from_config(t.get("wind_config", {}))
            # Update Gazebo wind config file if available
            try:
                self._update_gz_wind_config(t.get("wind_config", {}))
            except Exception as e:
                self.logger.warning(f"Failed updating gz wind config: {e}")
            ok_launch = await self.launch_px4(wind_env)
            if not ok_launch:
                results.append({"test_id": test_id, "success": False, "error": "px4_start_failed"})
                self.stop_px4()
                continue

            try:
                connected = await self.connect_mavsdk(mavlink_url)
                if not connected:
                    results.append({"test_id": test_id, "success": False, "error": "mavsdk_connect_failed"})
                else:
                    # start CSV + wind logging
                    await self._start_recording(test_id)

                    # Optional user-provided params before arming
                    pre_params = self.config.get("prearm_params", {})
                    if isinstance(pre_params, dict) and pre_params:
                        self.logger.info("Applying prearm params from config…")
                        await self._set_params(pre_params)

                    ok = await self.run_simple_mission(mission_cfg)
                    results.append({"test_id": test_id, "success": bool(ok)})
            finally:
                await self._stop_recording()
                self.stop_px4()
                await asyncio.sleep(3)

        # Save summary
        out = {
            "suite": self.config.get("test_suite", "gust_suite"),
            "timestamp": datetime.now().isoformat(),
            "results": results,
        }
        with open(self.run_dir / "results.json", "w") as f:
            json.dump(out, f, indent=2)

        passed = sum(1 for r in results if r.get("success"))
        total = len(results)
        self.logger.info(f"Summary: {passed}/{total} passed")
        return 0 if passed == total else 1

    # ----------------------------
    # GZ wind config writer
    # ----------------------------
    def _update_gz_wind_config(self, wind_cfg: Dict[str, Any]) -> None:
        """Write wind parameters into gz server.config for the active model.
        Edits src/modules/simulation/gz_bridge/server.config in-place.
        Creates a backup copy under the run_dir for traceability.
        """
        if not wind_cfg:
            return

        cfg_path = (self.px4_root / "src/modules/simulation/gz_bridge/server.config").resolve()
        if not cfg_path.is_file():
            self.logger.debug(f"server.config not found at {cfg_path}, skipping file update")
            return

        try:
            tree = ET.parse(cfg_path)
            root = tree.getroot()
        except ET.ParseError as e:
            self.logger.warning(f"Failed to parse {cfg_path}: {e}")
            return

        # Find wind plugin block (use set to avoid duplicates)
        plugin_elems = set()
        for p in root.findall('.//plugin'):
            name = p.attrib.get('name', '')
            filename = p.attrib.get('filename', '')
            if 'WindGust' in name or 'WindGust' in filename or 'WindGust' in ''.join(p.itertext()):
                plugin_elems.add(p)
            elif 'libWindGustPlugin.so' in filename:
                plugin_elems.add(p)

        if not plugin_elems:
            self.logger.debug("No WindGust plugin block found in server.config; skipping")
            return

        # Build parent map for all elements (standard solution for ElementTree)
        parent_map = {c: p for p in root.iter() for c in p}

        # Rebuild each plugin element from scratch to avoid parameter conflicts
        for pe in plugin_elems:
            # Save plugin attributes
            attribs = pe.attrib.copy()

            # Find parent using pre-built map
            parent = parent_map.get(pe)

            if parent is None:
                self.logger.warning("Could not find parent for plugin element; skipping")
                continue

            # Get insertion index to maintain position (with error handling)
            try:
                insert_idx = list(parent).index(pe)
            except ValueError:
                self.logger.warning(f"Plugin element not found in parent's children; appending to end")
                insert_idx = len(list(parent))

            # Remove old plugin element
            parent.remove(pe)

            # Create new plugin element with same attributes
            new_plugin = ET.Element('plugin', attribs)
            parent.insert(insert_idx, new_plugin)

            # Add all parameters from wind_cfg dynamically
            for key, value in wind_cfg.items():
                if isinstance(value, list):
                    # Handle vector parameters (mean, direction, etc.)
                    if len(value) == 3 and all(isinstance(v, (int, float)) for v in value):
                        ET.SubElement(new_plugin, key).text = f"{value[0]} {value[1]} {value[2]}"
                    else:
                        # Non-3D vector list, convert to space-separated string
                        ET.SubElement(new_plugin, key).text = " ".join(str(v) for v in value)
                elif isinstance(value, (int, float, str)):
                    # Scalar parameters
                    ET.SubElement(new_plugin, key).text = str(value)
                # Skip dict and other complex types

            # Add indentation for better formatting
            new_plugin.text = "\n      "
            new_plugin.tail = "\n  "
            for i, child in enumerate(new_plugin):
                child.tail = "\n      " if i < len(new_plugin) - 1 else "\n    "

        # Backup then write
        try:
            backup = self.run_dir / f"server.config.backup"
            backup.parent.mkdir(parents=True, exist_ok=True)
            with open(cfg_path, 'rb') as fsrc, open(backup, 'wb') as fdst:
                fdst.write(fsrc.read())
        except Exception:
            pass

        tree.write(cfg_path, encoding='utf-8', xml_declaration=False)
        self.logger.info(f"Updated wind config in {cfg_path}")

    # ----------------------------
    # CSV recorder and wind subscriber
    # ----------------------------
    async def _start_recording(self, test_id: str) -> None:
        if not self.drone:
            return
        self._recording = True
        # reset wind state for new task
        self._latest_wind_ms = None
        self._latest_wind_xyz = None
        self._wind_last_update = 0.0
        csv_path = self.run_dir / f"{test_id}.csv"
        self._csv_file = open(csv_path, "w", newline="")
        self._csv_writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                "t_s",
                "lat_deg", "lon_deg", "rel_alt_m", "abs_alt_m",
                "roll_deg", "pitch_deg", "yaw_deg",
                "sp_lat_deg", "sp_lon_deg", "sp_abs_alt_m",
                "wind_x_m_s", "wind_y_m_s", "wind_z_m_s", "wind_m_s",
            ],
        )
        self._csv_writer.writeheader()

        # Start wind topic subscriber and CSV loop
        self._wind_task = asyncio.create_task(self._wind_subscriber_task())
        self._csv_task = asyncio.create_task(self._csv_loop())

    async def _stop_recording(self) -> None:
        self._recording = False
        # cancel tasks gracefully
        for task in (self._csv_task, self._wind_task):
            if task and not task.done():
                task.cancel()
        try:
            if self._csv_task:
                await asyncio.gather(self._csv_task, return_exceptions=True)
            if self._wind_task:
                await asyncio.gather(self._wind_task, return_exceptions=True)
        finally:
            self._csv_task = None
            self._wind_task = None
            if self._csv_file:
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                except Exception:
                    pass
                self._csv_file = None
                self._csv_writer = None
            # clear wind state so next task starts clean
            self._latest_wind_ms = None
            self._latest_wind_xyz = None
            self._wind_last_update = 0.0

    async def _csv_loop(self) -> None:
        assert self.drone is not None
        start = asyncio.get_event_loop().time()
        period = 0.1  # 10 Hz
        while self._recording:
            t = asyncio.get_event_loop().time() - start
            row: Dict[str, Any] = {
                "t_s": f"{t:.3f}",
                "lat_deg": "",
                "lon_deg": "",
                "rel_alt_m": "",
                "abs_alt_m": "",
                "roll_deg": "",
                "pitch_deg": "",
                "yaw_deg": "",
                "sp_lat_deg": "",
                "sp_lon_deg": "",
                "sp_abs_alt_m": "",
                "wind_m_s": "",
            }
            try:
                pos = await self.drone.telemetry.position().__anext__()
                row["lat_deg"] = f"{pos.latitude_deg:.8f}"
                row["lon_deg"] = f"{pos.longitude_deg:.8f}"
                row["rel_alt_m"] = f"{pos.relative_altitude_m:.3f}"
                abs_alt = getattr(pos, "absolute_altitude_m", None)
                if abs_alt is not None:
                    row["abs_alt_m"] = f"{abs_alt:.3f}"
            except Exception:
                pass
            try:
                att = await self.drone.telemetry.attitude_euler().__anext__()
                row["roll_deg"] = f"{att.roll_deg:.2f}"
                row["pitch_deg"] = f"{att.pitch_deg:.2f}"
                row["yaw_deg"] = f"{att.yaw_deg:.2f}"
            except Exception:
                pass

            sp = self.current_setpoint
            if sp.get("lat") is not None:
                row["sp_lat_deg"] = f"{sp['lat']:.8f}"
            if sp.get("lon") is not None:
                row["sp_lon_deg"] = f"{sp['lon']:.8f}"
            if sp.get("alt_amsl") is not None:
                row["sp_abs_alt_m"] = f"{sp['alt_amsl']:.3f}"

            now = asyncio.get_event_loop().time()
            fresh = (self._latest_wind_xyz is not None) and (now - self._wind_last_update <= self.wind_stale_sec)
            if fresh:
                x, y, z = self._latest_wind_xyz
                row["wind_x_m_s"] = f"{x:.3f}"
                row["wind_y_m_s"] = f"{y:.3f}"
                row["wind_z_m_s"] = f"{z:.3f}"
                if self._latest_wind_ms is not None:
                    row["wind_m_s"] = f"{self._latest_wind_ms:.3f}"
            elif self.wind_fill_zero:
                row["wind_x_m_s"] = "0.000"
                row["wind_y_m_s"] = "0.000"
                row["wind_z_m_s"] = "0.000"
                row["wind_m_s"] = "0.000"

            try:
                if self._csv_writer:
                    self._csv_writer.writerow(row)
                    if self._csv_file:
                        self._csv_file.flush()
            except Exception:
                pass

            await asyncio.sleep(period)

    async def _wind_subscriber_task(self) -> None:
        """Subscribe to /world/windy/wind_gust via gz topic and update latest wind speed.
        Plain-text mode only (no -j). Lines may contain x:, y:, z: separately.
        """
        topic = "/world/windy/wind_gust"
        try:
            proc = await asyncio.create_subprocess_exec(
                "gz", "topic", "-e", "-t", topic,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            if proc.pid:
                self._pids_to_kill.add(proc.pid)
        except FileNotFoundError:
            self.logger.warning("`gz` CLI not found; wind speed logging disabled")
            return
        except Exception as e:
            self.logger.warning(f"Failed to start gz topic subscriber: {e}")
            return

        assert proc.stdout is not None
        buffer = ""
        last_vals = {"x": None, "y": None, "z": None}
        try:
            while self._recording:
                chunk = await proc.stdout.read(1024)
                if not chunk:
                    await asyncio.sleep(0.05)
                    continue
                try:
                    text = chunk.decode("utf-8", errors="ignore")
                except Exception:
                    continue
                buffer += text
                lines = buffer.splitlines(keepends=False)
                if not buffer.endswith("\n") and lines:
                    buffer = lines[-1]
                    lines = lines[:-1]
                else:
                    buffer = ""
                for line in lines:
                    xs = re.search(r"\bx:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)", line)
                    ys = re.search(r"\by:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)", line)
                    zs = re.search(r"\bz:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)", line)
                    updated = False
                    if xs:
                        try:
                            last_vals["x"] = float(xs.group(1))
                            updated = True
                        except Exception:
                            pass
                    if ys:
                        try:
                            last_vals["y"] = float(ys.group(1))
                            updated = True
                        except Exception:
                            pass
                    if zs:
                        try:
                            last_vals["z"] = float(zs.group(1))
                            updated = True
                        except Exception:
                            pass
                    if updated:
                        x = last_vals["x"] if last_vals["x"] is not None else 0.0
                        y = last_vals["y"] if last_vals["y"] is not None else 0.0
                        z = last_vals["z"] if last_vals["z"] is not None else 0.0
                        self._latest_wind_xyz = (x, y, z)
                        self._latest_wind_ms = float(math.sqrt(x * x + y * y + z * z))
                        self._wind_last_update = time.time()
        finally:
            try:
                proc.terminate()
            except Exception:
                pass

# Helper: extract a vector3 (x,y,z) from a nested dict/list JSON object
def _extract_vec3(obj: Any) -> Optional[tuple]:
    # Preferred keys
    candidates = ["wind", "linear_velocity", "velocity", "vec3", "linear"]
    if isinstance(obj, dict):
        for key in candidates:
            v = obj.get(key)
            if isinstance(v, dict) and all(k in v for k in ("x", "y", "z")):
                try:
                    return (float(v["x"]), float(v["y"]), float(v["z"]))
                except Exception:
                    pass
        # generic try: any dict with x,y,z
        if all(k in obj for k in ("x", "y", "z")):
            try:
                return (float(obj["x"]), float(obj["y"]), float(obj["z"]))
            except Exception:
                pass
        # recurse
        for k, v in obj.items():
            res = _extract_vec3(v)
            if res is not None:
                return res
    elif isinstance(obj, list):
        # list of 3 numbers
        if len(obj) == 3 and all(isinstance(it, (int, float)) for it in obj):
            try:
                return (float(obj[0]), float(obj[1]), float(obj[2]))
            except Exception:
                pass
        for it in obj:
            res = _extract_vec3(it)
            if res is not None:
                return res
    return None



# ----------------------------
# CLI
# ----------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Simplified PX4 wind gust test runner")
    p.add_argument("config_file", help="Path to JSON config file")
    p.add_argument("--px4-root", default=str(Path(__file__).resolve().parents[2]), help="PX4 repo root")
    p.add_argument("--build-dir", default="build/px4_sitl_default", help="Relative build dir")
    p.add_argument("--verbose", "-v", action="store_true", help="Verbose logging")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    with open(args.config_file, "r") as f:
        config = json.load(f)

    # Graceful Ctrl+C
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, lambda s=sig: asyncio.ensure_future(asyncio.sleep(0)))
        except NotImplementedError:
            pass

    runner = SimpleGustRunner(
        config=config,
        px4_root=Path(args.px4_root).resolve(),
        build_dir=(Path(args.px4_root) / args.build_dir).resolve(),
        verbose=args.verbose,
    )

    exit_code = loop.run_until_complete(runner.run())
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
