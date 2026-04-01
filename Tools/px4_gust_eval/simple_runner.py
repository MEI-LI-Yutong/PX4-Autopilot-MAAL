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
import random
import signal
import subprocess
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import psutil  # type: ignore
from mavsdk import System  # type: ignore
from mavsdk.mission import MissionError  # type: ignore
from mavsdk.mission_raw import MissionRawError  # type: ignore
import shlex
import math
import xml.etree.ElementTree as ET
import csv
import asyncio.subprocess
import re

from task_status_server import TaskStatusServer, TaskStatusTracker


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
        print("PX4 SITL is not built. Build: `make px4_sitl_default` (or CI recipe).")
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
# W&B helpers
# ----------------------------

def _env_flag(name: str, default: bool = False) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return val not in ("0", "false", "False")

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
    def __init__(self, config: Dict[str, Any], px4_root: Path, build_dir: Path, config_path: Path, verbose: bool) -> None:
        self.config = config
        self.px4_root = px4_root
        self.build_dir = build_dir
        self.config_path = config_path
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
        self._proc_log_task: Optional[asyncio.Task] = None
        self._proc_log_tail = deque(maxlen=200)
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

        # Optional param overrides from env (JSON)
        self.env_param_overrides: Dict[str, Any] = {}
        for env_name in ("PREARM_PARAMS_JSON", "PX4_PARAM_OVERRIDES"):
            raw = os.getenv(env_name)
            if raw:
                try:
                    parsed = json.loads(raw)
                    if isinstance(parsed, dict):
                        self.env_param_overrides.update(parsed)
                except Exception:
                    self.logger.warning(f"Failed to parse {env_name} as JSON; ignore.")

        # NocoDB integration (env-driven)
        self.nocodb_enabled = _env_flag("NOCODB_ENABLE", False)
        nocodb_cfg = self.config.get("nocodb", {}) if isinstance(self.config.get("nocodb", {}), dict) else {}
        self.nocodb_table_id = nocodb_cfg.get("table_id") or os.getenv("NOCODB_TABLE_ID")
        self.nocodb_view_id = nocodb_cfg.get("view_id") or os.getenv("NOCODB_VIEW_ID")
        self.nocodb_token = nocodb_cfg.get("token") or os.getenv("NOCODB_TOKEN")
        self.nocodb_base = nocodb_cfg.get("base_url") or os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com")
        try:
            self.nocodb_max_records = int(nocodb_cfg.get("max_records") or os.getenv("NOCODB_MAX_RECORDS", 1000))
        except Exception:
            self.nocodb_max_records = 1000
        if self.nocodb_table_id and self.nocodb_token:
            self.nocodb_enabled = True
        self._nocodb_selected: Optional[Dict[str, Any]] = None

        # W&B
        wandb_cfg = self.config.get("wandb", {}) if isinstance(self.config.get("wandb", {}), dict) else {}
        env_enable = os.getenv("WANDB_ENABLE")
        default_enable = bool(os.getenv("WANDB_API_KEY")) if env_enable is None else env_enable not in ("0", "false", "False")
        self.wandb_enabled = bool(wandb_cfg.get("enable", default_enable))
        self.wandb_project = wandb_cfg.get("project", os.getenv("WANDB_PROJECT", "px4_gust_eval"))
        self.wandb_entity = wandb_cfg.get("entity", os.getenv("WANDB_ENTITY"))
        self.wandb_run_name = wandb_cfg.get("run_name") or os.getenv("WANDB_RUN_NAME")
        self.wandb_tags = wandb_cfg.get("tags") or []
        self.wandb_upload_each_csv = bool(wandb_cfg.get("upload_each_csv", True if self.wandb_enabled else False))
        self.wandb_table_each_csv = bool(wandb_cfg.get("table_each_csv", True if self.wandb_enabled else False))
        self.wandb_upload_config = bool(wandb_cfg.get("upload_config", True if self.wandb_enabled else False))
        self.wandb_param_names: Sequence[str] = wandb_cfg.get("param_snapshot", [])
        if isinstance(self.wandb_param_names, str):
            self.wandb_param_names = [self.wandb_param_names]
        env_param_list = os.getenv("WANDB_PARAM_NAMES")
        if env_param_list:
            self.wandb_param_names = [p.strip() for p in env_param_list.split(",") if p.strip()]
        self.wandb_param_all = bool(wandb_cfg.get("param_snapshot_all", False))
        if _env_flag("WANDB_PARAM_ALL", False):
            self.wandb_param_all = True
        self._wandb_run = None
        self._wandb_params_cached: Optional[Dict[str, Any]] = None
        self.run_plots = _env_flag("RUN_PLOTS", True)
        self.sdf_edit_cfg = self.config.get("sdf_edit", {})
        self._sdf_edit_path: Optional[Path] = None
        self._sdf_edit_backup: Optional[bytes] = None
        self._task_status_tracker = TaskStatusTracker()
        self._task_status_server = TaskStatusServer(self._task_status_tracker, logger=self.logger)

    # ---- wandb integration ----
    def _init_wandb(self, cfg: Dict[str, Any]) -> None:
        if not self.wandb_enabled or self._wandb_run:
            return

        try:
            import wandb  # type: ignore
        except ImportError:
            self.logger.warning("wandb not installed; disable WANDB_ENABLE or install wandb (`uv pip install wandb` or run with `uv run --with wandb`).")
            self.wandb_enabled = False
            return

        run_name = self.wandb_run_name or f"gust-run-{self.run_dir.name}"
        tags = list(self.wandb_tags) if isinstance(self.wandb_tags, (list, tuple, set)) else [self.wandb_tags]
        tags.append("gust-eval")
        base_cfg = {
            "suite": cfg.get("test_suite", "gust_suite"),
            "config_file": str(self.config_path),
            "run_dir": str(self.run_dir),
            "build_target": cfg.get("build_target"),
            "simulator": cfg.get("simulator", "gazebo"),
        }

        self._wandb_run = wandb.init(
            project=self.wandb_project,
            entity=self.wandb_entity,
            name=run_name,
            tags=tags,
            config=base_cfg,
        )
        if self.wandb_upload_config:
            self._upload_task_config()

        # Attempt to fetch NocoDB overrides after run init (so we have run_id)
        if self.nocodb_enabled:
            try:
                nocodb_params = self._maybe_fetch_nocodb_params()
                if nocodb_params:
                    # Merge into env overrides for later application
                    self.env_param_overrides.update(nocodb_params)
                if self._nocodb_selected and self._wandb_run:
                    self._wandb_run.config.update({
                        "nocodb_record": {
                            "Id": self._nocodb_selected.get("Id"),
                            "Title": self._nocodb_selected.get("Title"),
                            "exp_times": self._nocodb_selected.get("exp_times"),
                        }
                    }, allow_val_change=True)
                    self._update_nocodb_record(self._wandb_run.id)
            except Exception as e:
                self.logger.warning(f"NocoDB integration failed: {e}")

    async def _get_param_value(self, name: str) -> Optional[float]:
        if not self.drone:
            return None
        try:
            return await self.drone.param.get_param_float(name)
        except Exception:
            pass
        try:
            return await self.drone.param.get_param_int(name)
        except Exception:
            return None

    async def _get_all_params(self) -> Dict[str, Any]:
        """Fetch all PX4 params (int + float)."""
        if not self.drone:
            return {}
        try:
            all_params = await self.drone.param.get_all_params()
            snap: Dict[str, Any] = {}
            for p in getattr(all_params, "float_params", []):
                snap[p.name] = p.value
            for p in getattr(all_params, "int_params", []):
                snap[p.name] = p.value
            return snap
        except Exception as e:
            self.logger.warning(f"Failed to fetch all PX4 params: {e}")
            return {}

    def _maybe_fetch_nocodb_params(self) -> Dict[str, Any]:
        """Fetch parameter set from NocoDB (min exp_times, random among ties)."""
        if not self.nocodb_enabled:
            return {}
        if not (self.nocodb_table_id and self.nocodb_token):
            return {}

        import urllib.request

        url_base = f"{self.nocodb_base.rstrip('/')}/api/v2/tables/{self.nocodb_table_id}/records"
        page_size = min(self.nocodb_max_records, int(os.getenv("NOCODB_PAGE_SIZE", 100)))
        offset = 0
        records: list[Dict[str, Any]] = []
        while offset < self.nocodb_max_records:
            limit = min(page_size, self.nocodb_max_records - len(records))
            params = {"offset": offset, "limit": limit, "where": ""}
            if self.nocodb_view_id:
                params["viewId"] = self.nocodb_view_id
            query = "&".join(f"{k}={urllib.parse.quote(str(v))}" for k, v in params.items())
            full_url = f"{url_base}?{query}"
            req = urllib.request.Request(full_url, headers={"xc-token": self.nocodb_token})
            try:
                with urllib.request.urlopen(req, timeout=10) as resp:
                    data = json.loads(resp.read().decode("utf-8"))
            except Exception as e:
                self.logger.warning(f"NocoDB fetch failed at offset {offset}: {e}")
                break

            page = data.get("list") or []
            total = data.get("totalRows")
            self.logger.info(f"NocoDB page: fetched {len(page)} record(s) (offset={offset}, limit={limit}, total={total})")
            if not page:
                break
            records.extend(page)
            offset += limit
            if len(page) < limit:
                break

        if not records:
            self.logger.info("NocoDB: no records returned")
            return {}

        def _exp_times(rec: Dict[str, Any]) -> float:
            v = rec.get("exp_times", 0)
            try:
                return float(v)
            except Exception:
                return 0.0

        min_exp = min(_exp_times(r) for r in records)
        candidates = [r for r in records if _exp_times(r) == min_exp]
        chosen = random.choice(candidates)
        self._nocodb_selected = chosen
        # Optional: load sdf_edit from JSON column
        raw_sdf = chosen.get("sdf_edit_json")
        if raw_sdf:
            try:
                parsed = raw_sdf
                if isinstance(raw_sdf, str):
                    parsed = json.loads(raw_sdf)
                if isinstance(parsed, dict):
                    if isinstance(parsed.get("sdf_edit"), dict):
                        self.sdf_edit_cfg = parsed.get("sdf_edit", {})
                    else:
                        self.sdf_edit_cfg = parsed
                    self.logger.info("NocoDB: loaded sdf_edit_json")
            except Exception as e:
                self.logger.warning(f"NocoDB: failed to parse sdf_edit_json: {e}")
        # Extract params: numeric values only, skip None/empty, skip meta fields
        overrides: Dict[str, Any] = {}
        meta_keys = {"Id", "nc_order", "Title", "exp_times", "wandb_runid", "sdf_edit_json"}
        for k, v in chosen.items():
            if k in meta_keys:
                continue
            if v is None or v == "":
                continue
            if isinstance(v, (int, float)):
                overrides[k] = v
        self.logger.info(f"NocoDB: selected record Id={chosen.get('Id')} Title={chosen.get('Title')} exp_times={chosen.get('exp_times')}")
        return overrides

    def _update_nocodb_record(self, wandb_run_id: str) -> None:
        if not (self.nocodb_enabled and self._nocodb_selected and self.nocodb_token and self.nocodb_table_id):
            return

        import urllib.request

        record_id = self._nocodb_selected.get("Id")
        if record_id is None:
            return
        url = f"{self.nocodb_base.rstrip('/')}/api/v2/tables/{self.nocodb_table_id}/records"
        # Merge existing wandb_runid array (if any) and append this run
        new_entry = {
            "run_id": wandb_run_id,
            "run_at": datetime.utcnow().isoformat() + "Z",
        }
        existing_runs = []
        try:
            raw = self._nocodb_selected.get("wandb_runid")
            parsed: Any = raw
            if isinstance(raw, str) and raw.strip():
                parsed = json.loads(raw)
            if isinstance(parsed, dict):
                parsed = [parsed]
            if isinstance(parsed, list):
                for item in parsed:
                    if isinstance(item, dict) and item.get("run_id"):
                        existing_runs.append(item)
        except Exception:
            pass
        merged_runs = existing_runs + [new_entry]

        payload_obj = {
            "Id": record_id,
            "wandb_runid": json.dumps(merged_runs),
        }
        try:
            cur = self._nocodb_selected.get("exp_times", 0)
            payload_obj["exp_times"] = float(cur) + 1
        except Exception:
            pass

        data = json.dumps([payload_obj]).encode("utf-8")
        req = urllib.request.Request(url, data=data, method="PATCH", headers={
            "xc-token": self.nocodb_token,
            "Content-Type": "application/json",
        })
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                resp.read()
            self.logger.info(f"NocoDB: updated record {record_id} with run_id={wandb_run_id}")
        except Exception as e:
            self.logger.warning(f"NocoDB update failed: {e}")

    async def _maybe_capture_params_snapshot(self) -> None:
        """Fetch selected PX4 params once and store in W&B config."""
        if not self.wandb_enabled or not self._wandb_run:
            return
        if self._wandb_params_cached is not None:
            return
        if not self.drone:
            return

        snapshot: Dict[str, Any] = {}
        if self.wandb_param_all:
            snapshot = await self._get_all_params()
        elif self.wandb_param_names:
            for name in self.wandb_param_names:
                val = await self._get_param_value(name)
                if val is not None:
                    snapshot[name] = val

        self._wandb_params_cached = snapshot
        if snapshot:
            try:
                self._wandb_run.config.update({"px4_params": snapshot}, allow_val_change=True)
            except Exception:
                pass
            self.logger.info(f"W&B config updated with {len(snapshot)} PX4 params")

    async def _maybe_upload_single_csv(self, test_id: str) -> None:
        """Upload per-test CSV as an artifact for crash resilience."""
        if not (self._wandb_run and self.wandb_upload_each_csv):
            return

        csv_path = self.run_dir / f"{test_id}.csv"
        if not csv_path.is_file():
            return

        try:
            import wandb  # type: ignore
        except ImportError:
            return

        artifact_name = f"gust-csv-{self.run_dir.name}-{test_id}"
        artifact = wandb.Artifact(artifact_name, type="gust-test-csv")
        artifact.add_file(str(csv_path), name=csv_path.name)

        # Optionally attach run log to each artifact for context
        run_logs = list(self.run_dir.glob("gust_run_*.log"))
        if run_logs:
            artifact.add_file(str(run_logs[0]), name=run_logs[0].name)

        try:
            self._wandb_run.log_artifact(artifact)
            self.logger.info(f"W&B: uploaded artifact {artifact_name}")
        except Exception as e:
            self.logger.warning(f"W&B upload failed for {artifact_name}: {e}")

    def _upload_task_config(self) -> None:
        """Upload the task JSON file as a W&B artifact."""
        if not (self._wandb_run and self.config_path.is_file()):
            return
        try:
            import wandb  # type: ignore
        except ImportError:
            return
        artifact_name = f"gust-task-config-{self.run_dir.name}"
        artifact = wandb.Artifact(artifact_name, type="gust-task-config")
        artifact.add_file(str(self.config_path), name=self.config_path.name)
        try:
            self._wandb_run.log_artifact(artifact)
            self.logger.info(f"W&B: uploaded task config {self.config_path}")
        except Exception as e:
            self.logger.warning(f"W&B upload failed for task config: {e}")

    async def _maybe_log_table_for_csv(self, test_id: str) -> None:
        """Log the full CSV as a W&B table for easy browsing (no column drops)."""
        if not (self._wandb_run and self.wandb_table_each_csv):
            return

        csv_path = self.run_dir / f"{test_id}.csv"
        if not csv_path.is_file():
            return

        try:
            import wandb  # type: ignore
            import pandas as pd  # type: ignore
        except ImportError:
            return

        try:
            df = pd.read_csv(csv_path)
            table = wandb.Table(dataframe=df)
            self._wandb_run.log({f"csv_table/{test_id}": table})
            self.logger.info(f"W&B: logged table for {test_id} ({len(df)} rows)")
        except Exception as e:
            self.logger.warning(f"W&B table log failed for {test_id}: {e}")

    async def _run_post_plots(self) -> None:
        """Optionally run plot_gust_levels.py and log into the same W&B run."""
        if not self.run_plots:
            return
        try:
            from plot_gust_levels import plot_levels  # type: ignore
        except Exception as e:
            self.logger.warning(f"Plotter import failed: {e}")
            return

        self.logger.info("Running post-plot inline (no extra W&B run)…")
        try:
            await asyncio.to_thread(
                plot_levels,
                self.config_path,
                self.run_dir,
                300,
                self._wandb_run,
                self.wandb_project,
                self.wandb_entity,
                self.wandb_run_name,
                self.wandb_tags if isinstance(self.wandb_tags, list) else [self.wandb_tags] if self.wandb_tags else [],
                True,
                bool(self._wandb_run),
            )
        except Exception as e:
            self.logger.warning(f"Post-plot step failed: {e}")

    async def _postprocess_with_ulog(self, test_id: str) -> None:
        """Augment CSV with ULog-derived setpoints and plot tracking errors."""
        csv_path = self.run_dir / f"{test_id}.csv"
        if not csv_path.is_file():
            return
        log_root_cfg = self.config.get("ulog_root")
        log_root = Path(log_root_cfg).expanduser().resolve() if log_root_cfg else (self.build_dir / "rootfs/log")
        try:
            import postprocess_ulog  # type: ignore
        except Exception as e:
            self.logger.warning(f"Failed to import postprocess_ulog: {e}")
            return
        try:
            await asyncio.to_thread(
                postprocess_ulog.process_single_test,
                self.run_dir,
                test_id,
                log_root,
                None,
                False,
                self.logger,
            )
        except FileNotFoundError as e:
            self.logger.warning(f"ULog postprocess skipped ({e})")
        except Exception as e:
            self.logger.warning(f"ULog postprocess failed for {test_id}: {e}")

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

    async def _ensure_prearm_ready(self, mission_cfg: Dict[str, Any]) -> bool:
        """Block until the vehicle is armable or apply relaxed prearm params if requested."""
        require_gps = bool(mission_cfg.get("require_gps", True))
        ready_timeout = int(mission_cfg.get("ready_timeout_sec", 60))
        force_arm = bool(mission_cfg.get("force_arm_if_unready", True))
        ready = await self._wait_until_armable(timeout_sec=ready_timeout, require_gps=require_gps)
        if not ready:
            self.logger.warning("Vehicle not armable within timeout")
            if force_arm:
                self.logger.info("Applying minimal prearm parameters for SITL (COM_ARM_WO_GPS=1)…")
                await self._set_params({"COM_ARM_WO_GPS": 1})
        return ready

    def _resolve_path(self, raw_path: str) -> Path:
        """Resolve mission/asset paths relative to the config file."""
        candidate = Path(os.path.expandvars(os.path.expanduser(raw_path)))
        if not candidate.is_absolute():
            candidate = (self.config_path.parent / candidate).resolve()
        return candidate

    def _get_plan_key(self, mission_cfg: Dict[str, Any]) -> Optional[str]:
        plan_key = mission_cfg.get("plan_file") or mission_cfg.get("mission_plan") or mission_cfg.get("qgc_plan")
        return str(plan_key) if plan_key else None

    async def _set_params(self, params: Dict[str, Any]) -> None:
        assert self.drone is not None
        for name, value in params.items():
            ok = False
            try:
                await self.drone.param.set_param_float(name, float(value))
                ok = True
            except Exception:
                try:
                    await self.drone.param.set_param_int(name, int(value))
                    ok = True
                except Exception as e2:
                    self.logger.warning(f"Param set failed {name}={value}: {e2}")
            if ok:
                await asyncio.sleep(0.05)

    async def _drain_px4_output(self) -> None:
        """Continuously consume the PX4 stdout pipe to avoid blocking the simulator."""
        if not self.proc or not self.proc.stdout:
            return

        stream = self.proc.stdout
        try:
            while True:
                line = await asyncio.to_thread(stream.readline)
                if not line:
                    break

                text = line.decode("utf-8", errors="replace").rstrip()
                if not text:
                    continue

                self._proc_log_tail.append(text)
                self.logger.debug(f"[px4] {text}")
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.logger.debug(f"PX4 log drain stopped unexpectedly: {e}")

    async def _finish_px4_output_drain(self) -> None:
        task = self._proc_log_task
        self._proc_log_task = None

        if task:
            try:
                await asyncio.wait_for(task, timeout=2.0)
            except asyncio.TimeoutError:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            except Exception:
                pass

        if self.proc and self.proc.stdout:
            try:
                self.proc.stdout.close()
            except Exception:
                pass

    async def launch_px4(self, env_overrides: Dict[str, str]) -> bool:
        cmd = self.config.get("px4_sitl_command")
        if not cmd:
            self.logger.error("Missing `px4_sitl_command` in config")
            return False

        await self._finish_px4_output_drain()

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
            self._proc_log_tail.clear()
            self.proc = subprocess.Popen(
                cmd_parts,
                cwd=self.px4_root,
                env=env,
                stdout=subprocess.PIPE if self.verbose else subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,  # new process group for easier cleanup
            )
            if self.proc.stdout:
                self._proc_log_task = asyncio.create_task(self._drain_px4_output())
            self._launch_time = time.time()
            await asyncio.sleep(2.0)
            self._track_child_pids()
            self._track_gazebo_pids()
            await asyncio.sleep(8)  # basic startup delay
            if self.proc.poll() is not None:
                out = "\n".join(self._proc_log_tail)
                self.logger.error(f"PX4 failed to start. Output tail:\n{out[-4000:]}")
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

    async def run_qgc_mission(self, mission_cfg: Dict[str, Any]) -> bool:
        """Upload a QGC mission plan and let PX4 execute it."""
        assert self.drone is not None

        plan_key = mission_cfg.get("plan_file") or mission_cfg.get("mission_plan") or mission_cfg.get("qgc_plan")
        if not plan_key:
            self.logger.error("Mission config missing 'plan_file'/'mission_plan' key for QGC mission.")
            return False

        plan_path = self._resolve_path(str(plan_key))
        if not plan_path.is_file():
            self.logger.error(f"Mission plan not found: {plan_path}")
            return False

        timeout_sec = int(mission_cfg.get("timeout_sec", 600))
        post_wait = int(mission_cfg.get("post_mission_wait_sec", 20))
        rtl_after = bool(mission_cfg.get("rtl_after_mission", True))
        force_arm = bool(mission_cfg.get("force_arm_if_unready", True))

        self.logger.info(f"Importing QGC mission from {plan_path}")
        try:
            import_data = await self.drone.mission_raw.import_qgroundcontrol_mission(str(plan_path))
        except MissionRawError as e:
            self.logger.error(f"Failed to import mission from {plan_path}: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Unexpected mission import error for {plan_path}: {e}")
            return False

        mission_items = list(getattr(import_data, "mission_items", []))
        if not mission_items:
            self.logger.error(f"No mission items found in {plan_path}")
            return False

        try:
            await self.drone.mission_raw.clear_mission()
        except Exception:
            pass

        try:
            await self.drone.mission_raw.upload_mission(mission_items)
        except MissionRawError as e:
            self.logger.error(f"Mission upload failed: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Unexpected mission upload error: {e}")
            return False

        try:
            await self.drone.mission.set_return_to_launch_after_mission(rtl_after)
        except MissionError as e:
            self.logger.warning(f"Failed to set RTL after mission ({rtl_after}): {e}")
        except Exception as e:
            self.logger.warning(f"Unexpected RTL-after-mission error: {e}")

        await self._ensure_prearm_ready(mission_cfg)

        try:
            await self.drone.action.arm()
        except Exception as e:
            if force_arm:
                self.logger.warning(f"Arm failed: {e}; retrying with relaxed params for SITL…")
                await self._set_params({
                    "COM_ARM_WO_GPS": 1,
                    "NAV_RCL_ACT": 0,
                    "NAV_DLL_ACT": 0,
                })
                await asyncio.sleep(1.0)
                try:
                    await self.drone.action.arm()
                except Exception as e2:
                    self.logger.error(f"Arm failed after retry: {e2}")
                    return False
            else:
                self.logger.error(f"Arm failed: {e}")
                return False

        loop = asyncio.get_event_loop()
        max_attempts = int(mission_cfg.get("mission_start_attempts", mission_cfg.get("mission_start_retries", 2)))
        max_attempts = max(1, max_attempts)
        min_runtime = float(mission_cfg.get("min_mission_runtime_sec", 8.0))

        attempt = 0
        success = False

        while attempt < max_attempts and not success:
            attempt += 1
            if attempt > 1:
                self.logger.info(f"Retrying mission start attempt {attempt}/{max_attempts}")
                try:
                    await self.drone.mission.set_current_mission_item(0)
                except MissionError as e:
                    self.logger.warning(f"Failed to reset mission item before retry: {e}")
                except Exception as e:
                    self.logger.warning(f"Unexpected error resetting mission item: {e}")
                await asyncio.sleep(1.0)

            try:
                await self.drone.mission.start_mission()
            except MissionError as e:
                self.logger.error(f"Mission start failed: {e}")
                if attempt >= max_attempts:
                    return False
                await asyncio.sleep(2.0)
                continue
            except Exception as e:
                self.logger.error(f"Unexpected mission start failure: {e}")
                if attempt >= max_attempts:
                    return False
                await asyncio.sleep(2.0)
                continue

            attempt_start = loop.time()
            attempt_deadline = attempt_start + max(30, timeout_sec)
            progress_stream = None
            in_air_stream = None
            try:
                progress_stream = self.drone.mission.mission_progress()
            except Exception:
                progress_stream = None
            try:
                in_air_stream = self.drone.telemetry.in_air()
            except Exception:
                in_air_stream = None

            progress_started = False
            ever_in_air = False
            finished_flag = False

            self.logger.info(f"Mission started; monitoring progress (attempt {attempt}/{max_attempts})…")
            while loop.time() < attempt_deadline:
                if progress_stream is not None:
                    try:
                        progress = await asyncio.wait_for(progress_stream.__anext__(), timeout=0.5)
                        self.logger.info(f"Mission progress {progress.current}/{progress.total}")
                        if getattr(progress, "current", 0) > 0:
                            progress_started = True
                    except asyncio.TimeoutError:
                        pass
                    except StopAsyncIteration:
                        progress_stream = None
                    except Exception as e:
                        self.logger.debug(f"Mission progress stream error: {e}")
                        progress_stream = None

                if in_air_stream is not None:
                    try:
                        ia = await asyncio.wait_for(in_air_stream.__anext__(), timeout=0.1)
                        if ia:
                            ever_in_air = True
                    except asyncio.TimeoutError:
                        pass
                    except StopAsyncIteration:
                        in_air_stream = None
                    except Exception:
                        in_air_stream = None

                try:
                    finished = await self.drone.mission.is_mission_finished()
                except MissionError:
                    finished = False
                except Exception:
                    finished = False
                if finished:
                    finished_flag = True
                    break

                await asyncio.sleep(0.3)

            if progress_stream is not None:
                try:
                    await progress_stream.aclose()
                except Exception:
                    pass
            if in_air_stream is not None:
                try:
                    await in_air_stream.aclose()
                except Exception:
                    pass

            runtime = loop.time() - attempt_start
            if not finished_flag:
                self.logger.error(f"Mission timeout after {timeout_sec} seconds (attempt {attempt}/{max_attempts})")
                return False

            if progress_started or ever_in_air or runtime >= min_runtime or attempt >= max_attempts:
                success = True
            else:
                self.logger.warning("Mission finished without movement; retrying start to overcome PX4 initialization quirk…")
                await asyncio.sleep(2.0)

        if not success:
            return False

        if post_wait > 0:
            land_deadline = loop.time() + post_wait
            while loop.time() < land_deadline:
                try:
                    ia = await self.drone.telemetry.in_air().__anext__()
                except Exception:
                    ia = True
                if not ia:
                    break
                await asyncio.sleep(0.5)

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

    def _collect_tests(self) -> List[Dict[str, Any]]:
        if "wind_gust_tests" in self.config:
            return list(self.config.get("wind_gust_tests", []))
        if "tests" in self.config:
            return list(self.config.get("tests", []))
        return [{
            "test_id": self.config.get("test_suite", "single"),
            "description": self.config.get("description", "single run"),
            "wind_config": self.config.get("wind_config", {}),
        }]

    def _record_task_result(
        self,
        results: List[Dict[str, Any]],
        test_id: str,
        success: bool,
        error: Optional[str] = None,
    ) -> None:
        result: Dict[str, Any] = {"test_id": test_id, "success": bool(success)}
        if error:
            result["error"] = error
        results.append(result)
        self._task_status_tracker.mark_finished(test_id, success=bool(success), error=error)

    # ---- orchestration ----
    async def run(self) -> int:
        tests = self._collect_tests()
        self._task_status_tracker.set_tasks(
            self.config.get("test_suite", "gust_suite"),
            tests,
        )
        self._task_status_server.start()

        simulator = self.config.get("simulator", "gazebo")
        try:
            if not check_ready(str(self.build_dir), simulator):
                return 2

            self._init_wandb(self.config)
            self._apply_sdf_edit()

            mission_cfg_raw = self.config.get("mission_config", self.config.get("mission", {}))
            mission_cfg = mission_cfg_raw if isinstance(mission_cfg_raw, dict) else {}
            mavlink_url = self.config.get("mavlink_url", "udp://:14540")

            results: List[Dict[str, Any]] = []

            try:
                for t in tests:
                    test_id = t.get("test_id", "test")
                    self._task_status_tracker.mark_running(test_id)
                    self.logger.info(f"Starting test: {test_id}")
                    wind_env = wind_env_from_config(t.get("wind_config", {}))
                    # Update Gazebo wind config file if available
                    try:
                        self._update_gz_wind_config(t.get("wind_config", {}))
                    except Exception as e:
                        self.logger.warning(f"Failed updating gz wind config: {e}")
                    ok_launch = await self.launch_px4(wind_env)
                    if not ok_launch:
                        self._record_task_result(results, test_id, False, "px4_start_failed")
                        self.stop_px4()
                        continue

                    try:
                        connected = await self.connect_mavsdk(mavlink_url)
                        if not connected:
                            self._record_task_result(results, test_id, False, "mavsdk_connect_failed")
                        else:
                            try:
                                # start CSV + wind logging
                                await self._start_recording(test_id)

                                # Optional user-provided params before arming
                                pre_params = self.config.get("prearm_params", {})
                                if isinstance(pre_params, dict):
                                    pre_params = pre_params.copy()
                                else:
                                    pre_params = {}
                                if self.env_param_overrides:
                                    pre_params.update(self.env_param_overrides)
                                if pre_params:
                                    self.logger.info(f"Applying prearm params ({len(pre_params)}) …")
                                    await self._set_params(pre_params)
                                    await self._maybe_capture_params_snapshot()
                                else:
                                    await self._maybe_capture_params_snapshot()

                                if self._get_plan_key(mission_cfg):
                                    ok = await self.run_qgc_mission(mission_cfg)
                                else:
                                    ok = await self.run_simple_mission(mission_cfg)
                                error = None if ok else "mission_failed"
                                self._record_task_result(results, test_id, bool(ok), error)
                            except Exception:
                                self.logger.exception(f"Test {test_id} crashed")
                                exc_name = sys.exc_info()[0].__name__ if sys.exc_info()[0] else "Exception"
                                self._record_task_result(results, test_id, False, f"unexpected_error: {exc_name}")
                    finally:
                        await self._stop_recording()
                        await self._postprocess_with_ulog(test_id)
                        await self._maybe_log_table_for_csv(test_id)
                        await self._maybe_upload_single_csv(test_id)
                        self.stop_px4()
                        await self._finish_px4_output_drain()
                        await asyncio.sleep(3)
            finally:
                self._restore_sdf_edit()

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
            if self._wandb_run:
                try:
                    self._wandb_run.log({"summary/passed": passed, "summary/total": total})
                except Exception:
                    pass
                await self._run_post_plots()
                try:
                    self._wandb_run.finish()
                except Exception:
                    pass
            return 0 if passed == total else 1
        finally:
            self._task_status_tracker.mark_suite_finished()
            self._task_status_server.stop()

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
    # SDF inertial override (optional)
    # ----------------------------
    def _apply_sdf_edit(self) -> None:
        """Optionally patch a model SDF before running; restore in _restore_sdf_edit()."""
        if not isinstance(self.sdf_edit_cfg, dict) or not self.sdf_edit_cfg:
            return
        raw_path = self.sdf_edit_cfg.get("path")
        if not raw_path:
            return
        sdf_path = self._resolve_path(str(raw_path))
        if not sdf_path.is_file():
            self.logger.warning(f"SDF edit path not found: {sdf_path}")
            return

        try:
            data = sdf_path.read_bytes()
        except Exception as e:
            self.logger.warning(f"Failed to read SDF: {e}")
            return

        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
        except Exception as e:
            self.logger.warning(f"Failed to parse SDF XML: {e}")
            return

        patches = self.sdf_edit_cfg.get("patches", [])
        if not isinstance(patches, list) or not patches:
            self.logger.warning("SDF edit skipped: no patches defined")
            return

        def _find_link(name: str) -> Optional[ET.Element]:
            for link in root.findall(".//link"):
                if link.attrib.get("name") == name:
                    return link
            return None

        def _find_path(base: ET.Element, rel_path: str) -> Optional[ET.Element]:
            cur = base
            for part in rel_path.split("/"):
                if not part:
                    continue
                nxt = cur.find(part)
                if nxt is None:
                    return None
                cur = nxt
            return cur

        for patch in patches:
            if not isinstance(patch, dict):
                continue
            select = patch.get("select", {})
            if not isinstance(select, dict):
                continue
            link_name = select.get("link")
            rel_path = select.get("path")
            if not link_name or not rel_path:
                self.logger.warning("SDF edit skipped: select.link/select.path required")
                continue
            link_elem = _find_link(str(link_name))
            if link_elem is None:
                self.logger.warning(f"SDF edit skipped: <link name='{link_name}'> not found")
                continue
            target_elem = _find_path(link_elem, str(rel_path))
            if target_elem is None:
                self.logger.warning(f"SDF edit skipped: path not found {link_name}/{rel_path}")
                continue
            if "value" not in patch:
                self.logger.warning(f"SDF edit skipped: value missing for {link_name}/{rel_path}")
                continue
            target_elem.text = str(patch["value"])

        try:
            tree.write(sdf_path, encoding="utf-8", xml_declaration=False)
            self._sdf_edit_path = sdf_path
            self._sdf_edit_backup = data
            self.logger.info(f"Applied SDF edit to {sdf_path}")
        except Exception as e:
            self.logger.warning(f"Failed to write SDF edit: {e}")

    def _restore_sdf_edit(self) -> None:
        if not (self._sdf_edit_path and self._sdf_edit_backup is not None):
            return
        try:
            self._sdf_edit_path.write_bytes(self._sdf_edit_backup)
            self.logger.info(f"Restored SDF: {self._sdf_edit_path}")
        except Exception as e:
            self.logger.warning(f"Failed to restore SDF: {e}")

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
        config_path=Path(args.config_file).resolve(),
        verbose=args.verbose,
    )

    exit_code = loop.run_until_complete(runner.run())
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
