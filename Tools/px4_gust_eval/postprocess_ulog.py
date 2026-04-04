#!/usr/bin/env python3
"""Augment gust eval CSVs with ULog trajectory setpoints and generate plots."""

from __future__ import annotations

import argparse
import logging
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
from pyulog import ULog  # type: ignore

LOGGER = logging.getLogger("gust.ulog")

EARTH_RADIUS_M = 6378137.0


def _normalize_columns(data: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    normalized: Dict[str, np.ndarray] = {}
    for key, value in data.items():
        col = key.replace("[", "_").replace("]", "")
        normalized[col] = value
    return normalized


def _dataset_by_name(ulog: ULog, name: str):
    for d in ulog.data_list:
        if d.name == name:
            return d
    return None


def _dataset_to_df(dataset) -> pd.DataFrame:
    if dataset is None:
        return pd.DataFrame()
    data = _normalize_columns(dataset.data)
    if not data:
        return pd.DataFrame()
    df = pd.DataFrame(data)
    if "timestamp" in df.columns:
        ts0 = df["timestamp"].iloc[0]
        df["t_rel_s"] = (df["timestamp"] - ts0) / 1e6
    else:
        df["t_rel_s"] = np.arange(len(df), dtype=float) * 0.01
    return df


def _extract_home_reference(ulog: ULog) -> Tuple[float, float, float]:
    home = _dataset_to_df(_dataset_by_name(ulog, "home_position"))
    if not home.empty:
        lat = float(home.get("lat", home.get("latitude", pd.Series([np.nan]))).iloc[0])
        lon = float(home.get("lon", home.get("longitude", pd.Series([np.nan]))).iloc[0])
        alt = float(home.get("alt", pd.Series([0.0])).iloc[0])
        if math.isfinite(lat) and math.isfinite(lon):
            return lat, lon, alt
    global_pos = _dataset_to_df(_dataset_by_name(ulog, "vehicle_global_position"))
    if global_pos.empty:
        raise RuntimeError("ULog missing home/global position data")
    lat = float(global_pos.get("lat").iloc[0])
    lon = float(global_pos.get("lon").iloc[0])
    alt = float(global_pos.get("alt").iloc[0])
    return lat, lon, alt


def _extract_global_positions(ulog: ULog) -> pd.DataFrame:
    df = _dataset_to_df(_dataset_by_name(ulog, "vehicle_global_position"))
    return df[["timestamp", "t_rel_s", "lat", "lon", "alt"]] if not df.empty else df


def _extract_setpoints(ulog: ULog) -> pd.DataFrame:
    candidates = ["vehicle_local_position_setpoint", "trajectory_setpoint"]
    for name in candidates:
        df = _dataset_to_df(_dataset_by_name(ulog, name))
        if df.empty:
            continue
        if name == "trajectory_setpoint":
            if all(col in df.columns for col in ("position_0", "position_1", "position_2")):
                df = df.rename(columns={"position_0": "x", "position_1": "y", "position_2": "z"})
        if all(axis in df.columns for axis in ("x", "y", "z")):
            return df[["timestamp", "t_rel_s", "x", "y", "z"]]
    return pd.DataFrame()


def _extract_actuator_outputs(ulog: ULog) -> pd.DataFrame:
    """Extract actuator outputs (u1..u16) from ULog if available."""
    # Prefer instance 0 if present, otherwise fall back to first actuator_outputs dataset.
    candidates = [d for d in ulog.data_list if d.name.startswith("actuator_outputs")]
    if not candidates:
        return pd.DataFrame()
    chosen = None
    for d in candidates:
        if d.name in ("actuator_outputs", "actuator_outputs_0"):
            chosen = d
            break
    if chosen is None:
        chosen = candidates[0]
    df = _dataset_to_df(chosen)
    if df.empty:
        return pd.DataFrame()
    out = pd.DataFrame()
    out["timestamp"] = df["timestamp"]
    out["t_rel_s"] = df["t_rel_s"]
    found = False
    for idx in range(16):
        col = f"output_{idx}"
        if col in df.columns:
            out[f"u{idx + 1}"] = df[col]
            found = True
    return out if found else pd.DataFrame()


def _extract_actuator_servos(ulog: ULog) -> pd.DataFrame:
    """Extract actuator servo controls (s1..s16) from ULog if available."""
    dataset = _dataset_by_name(ulog, "actuator_servos")
    df = _dataset_to_df(dataset)
    if df.empty:
        return pd.DataFrame()
    out = pd.DataFrame()
    out["timestamp"] = df["timestamp"]
    out["t_rel_s"] = df["t_rel_s"]
    found = False
    for idx in range(16):
        col = f"control_{idx}"
        if col in df.columns:
            out[f"s{idx + 1}"] = df[col]
            found = True
    return out if found else pd.DataFrame()


def _local_to_global(df: pd.DataFrame, home_lat: float, home_lon: float, home_alt: float) -> pd.DataFrame:
    if df.empty:
        return df
    lat0 = math.radians(home_lat)
    df = df.copy()
    df["traj_sp_lat_deg"] = home_lat + (df["x"] / EARTH_RADIUS_M) * 180.0 / math.pi
    df["traj_sp_lon_deg"] = home_lon + (df["y"] / (EARTH_RADIUS_M * math.cos(lat0))) * 180.0 / math.pi
    df["traj_sp_abs_alt_m"] = home_alt - df["z"]
    return df


def _hemisphere_distance(lat1: np.ndarray, lon1: np.ndarray, lat2: np.ndarray, lon2: np.ndarray) -> np.ndarray:
    lat1_rad = np.radians(lat1)
    lat2_rad = np.radians(lat2)
    d_lat = lat2_rad - lat1_rad
    d_lon = np.radians(lon2 - lon1)
    a = np.sin(d_lat / 2.0) ** 2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(d_lon / 2.0) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def _estimate_time_offset(csv_df: pd.DataFrame, global_df: pd.DataFrame) -> float:
    if csv_df.empty or global_df.empty:
        return 0.0
    reference = csv_df.dropna(subset=["lat_deg", "lon_deg"])
    if reference.empty:
        return 0.0
    ref_rows = reference.head(100)
    offsets: List[float] = []
    lat_global = global_df["lat"].to_numpy()
    lon_global = global_df["lon"].to_numpy()
    t_global = global_df["t_rel_s"].to_numpy()
    for _, row in ref_rows.iterrows():
        lat = row["lat_deg"]
        lon = row["lon_deg"]
        diffs = np.sqrt((lat_global - lat) ** 2 + (lon_global - lon) ** 2)
        idx = int(np.argmin(diffs))
        if diffs[idx] > 1e-3:
            continue
        offsets.append(row["t_s"] - t_global[idx])
    if not offsets:
        return 0.0
    return float(np.median(offsets))


def _merge_setpoints(csv_df: pd.DataFrame, setpoints: pd.DataFrame, tolerance: float) -> pd.DataFrame:
    if csv_df.empty or setpoints.empty:
        return csv_df
    csv_df = csv_df.copy()
    csv_df["__order"] = np.arange(len(csv_df), dtype=int)
    csv_sorted = csv_df.sort_values("t_s")
    setpoints_sorted = setpoints.sort_values("t_aligned_s")
    merged = pd.merge_asof(
        csv_sorted,
        setpoints_sorted,
        left_on="t_s",
        right_on="t_aligned_s",
        direction="nearest",
        tolerance=tolerance,
    )
    merged = merged.sort_values("__order").drop(columns=["__order", "t_aligned_s"], errors="ignore")
    return merged


def _apply_tracking_errors(df: pd.DataFrame) -> pd.DataFrame:
    if {"lat_deg", "lon_deg", "traj_sp_lat_deg", "traj_sp_lon_deg"}.issubset(df.columns):
        lat = df[["lat_deg", "traj_sp_lat_deg"]].to_numpy(dtype=float)
        lon = df[["lon_deg", "traj_sp_lon_deg"]].to_numpy(dtype=float)
        mask = np.isfinite(lat[:, 0]) & np.isfinite(lat[:, 1]) & np.isfinite(lon[:, 0]) & np.isfinite(lon[:, 1])
        errs = np.full(len(df), np.nan)
        if mask.any():
            errs[mask] = _hemisphere_distance(lat[mask, 0], lon[mask, 0], lat[mask, 1], lon[mask, 1])
        df["track_err_h_m"] = errs
    if {"abs_alt_m", "traj_sp_abs_alt_m"}.issubset(df.columns):
        df["track_err_v_m"] = df["abs_alt_m"] - df["traj_sp_abs_alt_m"]
    return df


def _save_plots(df: pd.DataFrame, test_id: str, out_dir: Path) -> List[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    generated: List[Path] = []
    mask_cols = {"lat_deg", "lon_deg", "traj_sp_lat_deg", "traj_sp_lon_deg"}
    if mask_cols.issubset(df.columns):
        mask = df[list(mask_cols)].notna().all(axis=1)
    else:
        mask = pd.Series([False] * len(df))
    if mask.any():
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.plot(df.loc[mask, "lon_deg"], df.loc[mask, "lat_deg"], label="Actual", linewidth=2.0)
        ax.plot(df.loc[mask, "traj_sp_lon_deg"], df.loc[mask, "traj_sp_lat_deg"], label="Setpoint", linewidth=2.0, linestyle="--")
        ax.set_xlabel("Longitude (deg)")
        ax.set_ylabel("Latitude (deg)")
        ax.set_title(f"Flight Path vs Setpoint - {test_id}")
        ax.legend()
        ax.grid(True, alpha=0.3)
        track_path = out_dir / f"{test_id}_path.png"
        fig.tight_layout()
        fig.savefig(track_path, dpi=300)
        plt.close(fig)
        generated.append(track_path)
    if {"t_s", "track_err_h_m", "track_err_v_m"}.issubset(df.columns):
        fig, ax = plt.subplots(figsize=(8, 4))
        if df["track_err_h_m"].notna().any():
            ax.plot(df["t_s"], df["track_err_h_m"], label="Horizontal Error (m)", linewidth=2.0)
        if df["track_err_v_m"].notna().any():
            ax.plot(df["t_s"], df["track_err_v_m"], label="Vertical Error (m)", linewidth=2.0, linestyle="--")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.set_title(f"Tracking Error vs Time - {test_id}")
        ax.grid(True, alpha=0.3)
        ax.legend()
        error_path = out_dir / f"{test_id}_tracking_error.png"
        fig.tight_layout()
        fig.savefig(error_path, dpi=300)
        plt.close(fig)
        generated.append(error_path)
    return generated


def augment_csv_with_ulog(
    csv_path: Path,
    ulog_path: Path,
    tolerance: float = 0.5,
    time_offset: Optional[float] = None,
    logger: Optional[logging.Logger] = None,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    if logger:
        global LOGGER
        LOGGER = logger
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    if not ulog_path or not ulog_path.is_file():
        raise FileNotFoundError(f"ULog not found: {ulog_path}")

    csv_df = pd.read_csv(csv_path)
    if "t_s" not in csv_df.columns:
        raise ValueError("CSV missing 't_s' column for alignment")

    ulog = ULog(str(ulog_path))
    global_df = _extract_global_positions(ulog)
    setpoints = _extract_setpoints(ulog)
    outputs = _extract_actuator_outputs(ulog)
    try:
        servos = _extract_actuator_servos(ulog)
    except Exception:  # Some logs do not contain actuator_servos
        servos = pd.DataFrame()
    if setpoints.empty:
        raise RuntimeError("ULog does not contain trajectory/local position setpoints")
    home_lat, home_lon, home_alt = _extract_home_reference(ulog)
    setpoints = _local_to_global(setpoints, home_lat, home_lon, home_alt)

    if time_offset is None:
        time_offset = _estimate_time_offset(csv_df, global_df)
    setpoints["t_aligned_s"] = setpoints["t_rel_s"] + float(time_offset)

    merged = _merge_setpoints(csv_df, setpoints, tolerance)
    if not outputs.empty:
        outputs["t_aligned_s"] = outputs["t_rel_s"] + float(time_offset)
        merged = _merge_setpoints(merged, outputs, tolerance)
    if not servos.empty:
        servos["t_aligned_s"] = servos["t_rel_s"] + float(time_offset)
        merged = _merge_setpoints(merged, servos, tolerance)
    merged = _apply_tracking_errors(merged)
    merged.to_csv(csv_path, index=False)
    return merged, setpoints


def find_latest_ulog(log_root: Path) -> Optional[Path]:
    if not log_root.exists():
        return None
    for date_entry in sorted(log_root.iterdir(), reverse=True):
        if date_entry.is_file() and date_entry.suffix == ".ulg":
            return date_entry
        if not date_entry.is_dir():
            continue
        # Some setups store ulogs directly inside the date dir
        direct_logs = sorted(date_entry.glob("*.ulg"), key=lambda p: p.stat().st_mtime, reverse=True)
        if direct_logs:
            return direct_logs[0]
        for run_dir in sorted(date_entry.iterdir(), reverse=True):
            if not run_dir.is_dir():
                continue
            ulogs = sorted(run_dir.glob("*.ulg"), key=lambda p: p.stat().st_mtime, reverse=True)
            if ulogs:
                return ulogs[0]
    return None


def process_single_test(
    run_dir: Path,
    test_id: str,
    log_root: Path,
    ulog_path: Optional[Path] = None,
    make_plots: bool = True,
    logger: Optional[logging.Logger] = None,
) -> Optional[List[Path]]:
    csv_path = run_dir / f"{test_id}.csv"
    if not csv_path.is_file():
        LOGGER.warning("CSV not found for test %s in %s", test_id, run_dir)
        return None
    resolved_ulog = ulog_path or find_latest_ulog(log_root)
    if resolved_ulog is None:
        LOGGER.warning("No ULog file found under %s", log_root)
        return None
    LOGGER.info("Augmenting %s using %s", csv_path.name, resolved_ulog)
    merged_df, _ = augment_csv_with_ulog(csv_path, resolved_ulog, logger=logger)
    generated: List[Path] = []
    if make_plots:
        generated = _save_plots(merged_df, test_id, run_dir)
    return generated


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Augment gust CSV with ULog trajectory setpoints and plot tracking errors"
    )
    parser.add_argument("--run-dir", type=Path, help="Directory containing run_* CSV files")
    parser.add_argument("--test-id", type=str, help="Test ID whose CSV should be updated")
    parser.add_argument("--csv", type=Path, help="Explicit CSV path to update")
    parser.add_argument("--ulog", type=Path, help="Path to a specific ULog; defaults to latest under --log-root")
    parser.add_argument(
        "--log-root",
        type=Path,
        default=Path("build/px4_sitl_default/rootfs/log"),
        help="PX4 log root (contains date folders)",
    )
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    parser.add_argument("--tolerance", type=float, default=0.5, help="Max time difference (s) for alignment")
    parser.add_argument("--time-offset", type=float, default=None, help="Manual time offset for setpoints (seconds)")
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()
    if args.csv:
        csv_path = args.csv
        run_dir = csv_path.parent
        test_id = csv_path.stem
    else:
        if not (args.run_dir and args.test_id):
            raise SystemExit("Provide either --csv or (--run-dir and --test-id)")
        run_dir = args.run_dir
        test_id = args.test_id
        csv_path = run_dir / f"{test_id}.csv"
    ulog_path = args.ulog or find_latest_ulog(args.log_root)
    if ulog_path is None:
        raise SystemExit(f"No ULog found under {args.log_root}")
    if args.time_offset is not None:
        merged_df, _ = augment_csv_with_ulog(
            csv_path,
            ulog_path,
            tolerance=args.tolerance,
            time_offset=args.time_offset,
        )
        if not args.no_plots:
            _save_plots(merged_df, test_id, run_dir)
        return
    process_single_test(run_dir, test_id, args.log_root, ulog_path=ulog_path, make_plots=not args.no_plots)


if __name__ == "__main__":
    main()
