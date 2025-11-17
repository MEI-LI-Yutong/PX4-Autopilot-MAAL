#!/usr/bin/env python3
"""
Convert PX4 ULog files into gust evaluation CSVs.

The output format is aligned with the CSV files used by the
PX4 gust evaluation tools, e.g.:

  Tools/px4_gust_eval/logs/levels/CA_switchingStrategy_plus/gust_lvl_00.csv

By default this script:
- scans ULog files under:  build/px4_sitl_neural/rootfs/log/2025-11-16
- sorts them by filename
- converts each to a CSV with columns:
    t_s, lat_deg, lon_deg, rel_alt_m, abs_alt_m,
    roll_deg, pitch_deg, yaw_deg,
    sp_lat_deg, sp_lon_deg, sp_abs_alt_m,
    wind_x_m_s, wind_y_m_s, wind_z_m_s, wind_m_s
- writes CSVs to: Tools/px4_gust_eval/logs/levels/from_ulog/2025-11-16
- names them sequentially: gust_lvl_00.csv, gust_lvl_01.csv, ...

Usage examples (from repo root):

  # Use defaults (2025-11-16 logs -> from_ulog/2025-11-16/gust_lvl_*.csv)
  uv run Tools/px4_gust_eval/ulog_to_gust_csv.py

  # Custom input/output directories
  uv run Tools/px4_gust_eval/ulog_to_gust_csv.py \
    --ulog-dir build/px4_sitl_neural/rootfs/log/2025-11-16 \
    --output-dir Tools/px4_gust_eval/logs/levels/CA_switchingStrategy_plus
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
from pyulog import ULog  # type: ignore


def _repo_root() -> Path:
    """Return repository root (two levels above this file)."""
    return Path(__file__).resolve().parents[2]


def _get_dataset(ulog: ULog, name: str) -> Optional[dict]:
    """Try to get a dataset by name, returning its .data dict or None."""
    try:
        ds = ulog.get_dataset(name)
    except (KeyError, IndexError):
        return None
    return ds.data


def _quaternion_to_euler_deg(
    q0: np.ndarray, q1: np.ndarray, q2: np.ndarray, q3: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Convert PX4 attitude quaternion (w, x, y, z) to roll/pitch/yaw in degrees.
    Quaternion is rotation from FRD body frame to NED earth frame.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp_clipped = np.clip(sinp, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp_clipped))

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


def extract_gust_dataframe(ulog: ULog) -> pd.DataFrame:
    """
    Extract a gust-eval compatible DataFrame from a ULog.

    Columns:
      t_s, lat_deg, lon_deg, rel_alt_m, abs_alt_m,
      roll_deg, pitch_deg, yaw_deg,
      sp_lat_deg, sp_lon_deg, sp_abs_alt_m,
      wind_x_m_s, wind_y_m_s, wind_z_m_s, wind_m_s
    """
    # ---------------------
    # Base position (time axis)
    # ---------------------
    gpos = _get_dataset(ulog, "vehicle_global_position")
    if gpos is not None:
        ts_pos = gpos["timestamp"]
        lat = gpos["lat"]
        lon = gpos["lon"]
        alt = gpos["alt"]
        df = pd.DataFrame(
            {
                "timestamp": ts_pos,
                "lat_deg": lat,
                "lon_deg": lon,
                "abs_alt_m": alt,
            }
        )
        df["rel_alt_m"] = df["abs_alt_m"] - float(df["abs_alt_m"].iloc[0])
    else:
        # Fallback to raw GPS if fused global position is unavailable
        gps = _get_dataset(ulog, "vehicle_gps_position")
        if gps is not None:
            ts_pos = gps["timestamp"]
            lat = gps["lat"] * 1.0e-7  # [1e-7 deg] -> [deg]
            lon = gps["lon"] * 1.0e-7
            alt = gps["alt"] * 1.0e-3  # [mm] -> [m]
            df = pd.DataFrame(
                {
                    "timestamp": ts_pos,
                    "lat_deg": lat,
                    "lon_deg": lon,
                    "abs_alt_m": alt,
                }
            )
            df["rel_alt_m"] = df["abs_alt_m"] - float(df["abs_alt_m"].iloc[0])
        else:
            # Last resort: derive only time from first dataset
            base = ulog.data_list[0].data
            ts_pos = base["timestamp"]
            df = pd.DataFrame(
                {
                    "timestamp": ts_pos,
                    "lat_deg": np.nan,
                    "lon_deg": np.nan,
                    "abs_alt_m": np.nan,
                    "rel_alt_m": np.nan,
                }
            )

    df.sort_values("timestamp", inplace=True)
    df.reset_index(drop=True, inplace=True)

    # ---------------------
    # Attitude
    # ---------------------
    att = _get_dataset(ulog, "vehicle_attitude") or _get_dataset(ulog, "vehicle_attitude_groundtruth")
    if att is not None:
        ts_att = att["timestamp"]
        q0 = np.asarray(att["q[0]"], dtype=float)
        q1 = np.asarray(att["q[1]"], dtype=float)
        q2 = np.asarray(att["q[2]"], dtype=float)
        q3 = np.asarray(att["q[3]"], dtype=float)
        roll, pitch, yaw = _quaternion_to_euler_deg(q0, q1, q2, q3)

        df_att = pd.DataFrame(
            {
                "timestamp": ts_att,
                "roll_deg": roll,
                "pitch_deg": pitch,
                "yaw_deg": yaw,
            }
        ).sort_values("timestamp")

        df = pd.merge_asof(
            df.sort_values("timestamp"),
            df_att,
            on="timestamp",
            direction="nearest",
        )
    else:
        df["roll_deg"] = np.nan
        df["pitch_deg"] = np.nan
        df["yaw_deg"] = np.nan

    # ---------------------
    # Wind (horizontal only; vertical set to 0)
    # ---------------------
    wind = _get_dataset(ulog, "wind") or _get_dataset(ulog, "estimator_wind")
    if wind is not None and "windspeed_north" in wind and "windspeed_east" in wind:
        ts_w = wind["timestamp"]
        wind_x = np.asarray(wind["windspeed_north"], dtype=float)
        wind_y = np.asarray(wind["windspeed_east"], dtype=float)
        wind_z = np.zeros_like(wind_x)
        wind_m = np.sqrt(wind_x * wind_x + wind_y * wind_y + wind_z * wind_z)

        df_wind = pd.DataFrame(
            {
                "timestamp": ts_w,
                "wind_x_m_s": wind_x,
                "wind_y_m_s": wind_y,
                "wind_z_m_s": wind_z,
                "wind_m_s": wind_m,
            }
        ).sort_values("timestamp")

        df = pd.merge_asof(
            df.sort_values("timestamp"),
            df_wind,
            on="timestamp",
            direction="nearest",
        )
    else:
        df["wind_x_m_s"] = np.nan
        df["wind_y_m_s"] = np.nan
        df["wind_z_m_s"] = np.nan
        df["wind_m_s"] = np.nan

    # ---------------------
    # Setpoints (not easily recoverable from ULog here) -> keep empty
    # ---------------------
    df["sp_lat_deg"] = np.nan
    df["sp_lon_deg"] = np.nan
    df["sp_abs_alt_m"] = np.nan

    # ---------------------
    # Time in seconds from start
    # ---------------------
    t0 = float(df["timestamp"].iloc[0])
    df["t_s"] = (df["timestamp"] - t0) * 1.0e-6

    # Final column ordering
    cols = [
        "t_s",
        "lat_deg",
        "lon_deg",
        "rel_alt_m",
        "abs_alt_m",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "sp_lat_deg",
        "sp_lon_deg",
        "sp_abs_alt_m",
        "wind_x_m_s",
        "wind_y_m_s",
        "wind_z_m_s",
        "wind_m_s",
    ]

    for c in cols:
        if c not in df.columns:
            df[c] = np.nan

    return df[cols]


def convert_directory(ulog_dir: Path, output_dir: Path, prefix: str = "gust_lvl_", start_index: int = 0) -> None:
    """Convert all .ulg files in a directory to sequential gust_lvl_XX.csv files."""
    if not ulog_dir.is_dir():
        raise SystemExit(f"ULog directory not found: {ulog_dir}")

    ulog_files = sorted(ulog_dir.glob("*.ulg"))
    if not ulog_files:
        raise SystemExit(f"No .ulg files found in: {ulog_dir}")

    output_dir.mkdir(parents=True, exist_ok=True)

    for idx, ulog_path in enumerate(ulog_files, start=start_index):
        test_id = f"{prefix}{idx:02d}"
        out_path = output_dir / f"{test_id}.csv"

        print(f"[ulog_to_gust_csv] {ulog_path.name} -> {out_path}")

        ulog = ULog(str(ulog_path))
        df = extract_gust_dataframe(ulog)

        # Save with reasonable precision; NaNs become empty/NaN cells
        df.to_csv(out_path, index=False, float_format="%.3f")


def parse_args() -> argparse.Namespace:
    repo = _repo_root()
    default_ulog_dir = repo / "build" / "px4_sitl_neural" / "rootfs" / "log" / "2025-11-16"
    default_output_dir = repo / "Tools" / "px4_gust_eval" / "logs" / "levels" / "from_ulog" / "2025-11-16"

    parser = argparse.ArgumentParser(
        description="Convert PX4 ULog files into gust evaluation CSVs named gust_lvl_XX.csv"
    )
    parser.add_argument(
        "--ulog-dir",
        type=Path,
        default=default_ulog_dir,
        help=f"Directory containing .ulg files (default: {default_ulog_dir})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=default_output_dir,
        help=f"Output directory for gust_lvl_XX.csv files (default: {default_output_dir})",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="gust_lvl_",
        help="Filename prefix for CSVs (default: gust_lvl_)",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=0,
        help="Starting index for gust_lvl_XX numbering (default: 0)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    convert_directory(args.ulog_dir, args.output_dir, prefix=args.prefix, start_index=args.start_index)


if __name__ == "__main__":
    main()
