from __future__ import annotations

import math
import re
from typing import Dict, Optional, Tuple

import numpy as np
import pandas as pd

# Stability thresholds
H_MAX = 1.5      # m - horizontal max deviation
H_STD = 0.75     # m - horizontal std deviation
V_MAX = 3.0      # m - vertical max deviation
V_STD = 1.5      # m - vertical std deviation
ANG_MAX = 45.0   # deg - max roll/pitch

# Recovery window for Wind-Recoverable
RECOVER_T = 10.0  # seconds

# Fixed analysis window (seconds)
ANALYSIS_WINDOW = (30.0, 60.0)

# Earth radius for lat->meters conversion
EARTH_RADIUS_M = 6378137.0


def latlon_to_xy(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
    """Convert lat/lon to local XY in meters."""
    lat = df["lat_deg"].to_numpy(dtype=float)
    lon = df["lon_deg"].to_numpy(dtype=float)
    mask = np.isfinite(lat) & np.isfinite(lon)
    if not mask.any():
        return np.array([]), np.array([])
    i0 = int(np.argmax(mask))
    lat0 = math.radians(lat[i0])
    lon0 = math.radians(lon[i0])
    x = (np.radians(lon) - lon0) * math.cos(lat0) * EARTH_RADIUS_M
    y = (np.radians(lat) - lat0) * EARTH_RADIUS_M
    return x, y


def _apply_time_window(df: pd.DataFrame) -> pd.DataFrame:
    """Restrict dataframe to the fixed analysis window if enough samples exist."""
    if "t_s" not in df.columns:
        return df
    mask = (df["t_s"] >= ANALYSIS_WINDOW[0]) & (df["t_s"] <= ANALYSIS_WINDOW[1])
    if mask.sum() >= 5:
        return df[mask].reset_index(drop=True)
    return df


def _segment_mask_from_x(x: np.ndarray) -> np.ndarray:
    """Return boolean mask for mid-flight segment (5m <= x <= 95m)."""
    if x.size == 0 or not np.isfinite(x).any():
        return np.zeros_like(x, dtype=bool)
    mask = (x >= 5.0) & (x <= 95.0)
    if mask.sum() >= max(10, int(0.2 * x.size)):
        return mask
    # Fallback percentiles
    x_valid = x[np.isfinite(x)]
    lo, hi = np.percentile(x_valid, [10.0, 90.0])
    mask2 = (x >= lo) & (x <= hi)
    return mask2


def compute_track_errors_from_raw(df: pd.DataFrame) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Compute tracking errors from raw columns (prefer lat-based horizontal error)."""
    h_err = None
    v_err = None

    if {"lat_deg", "traj_sp_lat_deg"}.issubset(df.columns):
        lat = pd.to_numeric(df["lat_deg"], errors="coerce").to_numpy(dtype=float)
        lat_sp = pd.to_numeric(df["traj_sp_lat_deg"], errors="coerce").to_numpy(dtype=float)
        if lat.size and lat_sp.size:
            h_err = (lat - lat_sp) * (EARTH_RADIUS_M * (np.pi / 180.0))
    elif "track_err_h_m" in df.columns:
        h_err = pd.to_numeric(df["track_err_h_m"], errors="coerce").to_numpy(dtype=float)

    if "track_err_v_m" in df.columns:
        v_err = pd.to_numeric(df["track_err_v_m"], errors="coerce").to_numpy(dtype=float)

    if v_err is None and "traj_sp_abs_alt_m" in df.columns:
        if "abs_alt_m" in df.columns:
            alt = pd.to_numeric(df["abs_alt_m"], errors="coerce").to_numpy(dtype=float)
        elif "rel_alt_m" in df.columns:
            alt = pd.to_numeric(df["rel_alt_m"], errors="coerce").to_numpy(dtype=float)
        else:
            alt = None
        if alt is not None:
            alt_sp = pd.to_numeric(df["traj_sp_abs_alt_m"], errors="coerce").to_numpy(dtype=float)
            if alt.size and alt_sp.size:
                v_err = alt - alt_sp
    return h_err, v_err


def _compute_track_errors_from_raw(df: pd.DataFrame) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Backward-compatible wrapper."""
    return compute_track_errors_from_raw(df)


def _select_analysis_window_by_wind(df: pd.DataFrame) -> Optional[Tuple[float, float]]:
    """Select analysis time window based on wind segment and rules."""
    if "t_s" not in df.columns or "wind_m_s" not in df.columns:
        return None
    t = pd.to_numeric(df["t_s"], errors="coerce").to_numpy(dtype=float)
    wind = pd.to_numeric(df["wind_m_s"], errors="coerce").to_numpy(dtype=float)
    if t.size == 0 or wind.size == 0:
        return None
    finite = np.isfinite(t)
    if not finite.any():
        return None
    t_min = float(np.nanmin(t))
    t_max = float(np.nanmax(t))
    total = t_max - t_min
    if total <= 0.0:
        return None

    wind_mask = wind > 0.01
    if wind_mask.sum() < 5:
        return None

    idx = np.where(wind_mask)[0]
    # Split into contiguous segments by index gaps
    splits = np.where(np.diff(idx) > 1)[0]
    segments = []
    start = 0
    for s in splits:
        segments.append(idx[start:s + 1])
        start = s + 1
    segments.append(idx[start:])
    # Pick segment with longest duration
    best = max(segments, key=lambda seg: t[seg[-1]] - t[seg[0]])
    seg_start = float(t[best[0]])
    seg_end = float(t[best[-1]])
    seg_duration = seg_end - seg_start

    if seg_duration / total > 0.5:
        # Use mid-route time ±10s
        x, _ = latlon_to_xy(df)
        if x.size:
            mid_mask = _segment_mask_from_x(x)
            if mid_mask.sum() >= 5:
                mid_t = float(np.nanmedian(t[mid_mask]))
            else:
                mid_t = float(np.nanmedian(t))
        else:
            mid_t = float(np.nanmedian(t))
        return mid_t - 10.0, mid_t + 10.0

    return seg_start, seg_end + 10.0


def select_analysis_df(df: pd.DataFrame) -> pd.DataFrame:
    """Select analysis window based on wind rules, fall back to fixed time window."""
    wind_window = _select_analysis_window_by_wind(df)
    if wind_window is not None and "t_s" in df.columns:
        t0, t1 = wind_window
        time_mask = (df["t_s"] >= t0) & (df["t_s"] <= t1)
        if time_mask.sum() >= 5:
            return df[time_mask].reset_index(drop=True)

    df_time = _apply_time_window(df)
    return df_time


def compute_analysis_window(df: pd.DataFrame) -> Optional[Tuple[float, float]]:
    """Return analysis window (t0, t1) consistent with select_analysis_df."""
    if "t_s" not in df.columns:
        return None

    wind_window = _select_analysis_window_by_wind(df)
    if wind_window is not None:
        t0, t1 = wind_window
        time_mask = (df["t_s"] >= t0) & (df["t_s"] <= t1)
        if time_mask.sum() >= 5:
            return t0, t1

    mask = (df["t_s"] >= ANALYSIS_WINDOW[0]) & (df["t_s"] <= ANALYSIS_WINDOW[1])
    if mask.sum() >= 5:
        return ANALYSIS_WINDOW

    return None


def compute_metrics_for_test(df: pd.DataFrame) -> Optional[Dict[str, float]]:
    """Compute stability metrics on selected window."""
    if df.empty or not {"lat_deg", "lon_deg"}.issubset(df.columns):
        return None
    df = select_analysis_df(df)
    if df.empty:
        return None

    x, y = latlon_to_xy(df)
    mask = np.ones_like(x, dtype=bool) if x.size else np.ones(len(df), dtype=bool)
    if mask.sum() < 5:
        return None

    h_err, v_err = _compute_track_errors_from_raw(df)

    h_max = float("nan")
    h_std = float("nan")
    if h_err is not None:
        finite = np.isfinite(h_err)
        if finite.sum() >= 5:
            h_max = float(np.nanmax(np.abs(h_err[finite])))
            h_std = float(np.nanstd(h_err[finite]))

    v_max = float("nan")
    v_std = float("nan")
    if v_err is not None:
        finite_v = np.isfinite(v_err)
        if finite_v.sum() >= 5:
            v_max = float(np.nanmax(np.abs(v_err[finite_v])))
            v_std = float(np.nanstd(v_err[finite_v]))

    # Fallback to legacy lat/lon baseline when tracking error columns are missing
    if math.isnan(h_max) or math.isnan(h_std):
        y_seg = y[mask]
        y_ref = np.nanmedian(y_seg)
        h_dev = y_seg - y_ref
        h_max = float(np.nanmax(np.abs(h_dev))) if y_seg.size else float("nan")
        h_std = float(np.nanstd(h_dev)) if y_seg.size else float("nan")

    if math.isnan(v_max) or math.isnan(v_std):
        if "rel_alt_m" in df.columns:
            z = df["rel_alt_m"].to_numpy(dtype=float)
            z_seg = z[mask]
            z_ref = 10.0
            v_dev = z_seg - z_ref
            v_max = float(np.nanmax(np.abs(v_dev))) if z_seg.size else float("nan")
            v_std = float(np.nanstd(v_dev)) if z_seg.size else float("nan")

    # Attitude (within segment)
    def _max_abs(col: str) -> float:
        if col in df.columns:
            arr = df[col].to_numpy(dtype=float)[mask]
            if arr.size:
                return float(np.nanmax(np.abs(arr)))
        return float("nan")

    max_abs_roll = _max_abs("roll_deg")
    max_abs_pitch = _max_abs("pitch_deg")

    return {
        "h_max_dev": h_max,
        "h_std": h_std,
        "v_max_dev": v_max,
        "v_std": v_std,
        "max_abs_roll": max_abs_roll,
        "max_abs_pitch": max_abs_pitch,
    }


def compute_grade_dimensional(df: pd.DataFrame, dim: str) -> str:
    """Compute stability grade for a single test, per dimension."""
    if df.empty or not {"lat_deg", "lon_deg"}.issubset(df.columns):
        return "Not launched"
    df = select_analysis_df(df)
    if df.empty:
        return "Not launched"

    if dim == 'v' and "traj_sp_abs_alt_m" not in df.columns:
        return "Not launched"

    m = compute_metrics_for_test(df)
    if m is None:
        return "Not launched"

    if dim == 'h':
        base_ok = (
            (m.get("h_max_dev", float("inf")) <= H_MAX) and
            (m.get("h_std", float("inf")) <= H_STD)
        )
    else:  # 'v'
        base_ok = (
            (m.get("v_max_dev", float("inf")) <= V_MAX) and
            (m.get("v_std", float("inf")) <= V_STD)
        )

    if base_ok:
        return "Wind-Resilient"

    # Check recovery for Wind-Recoverable (dimension-specific)
    h_err, v_err = _compute_track_errors_from_raw(df)
    err = h_err if dim == 'h' else v_err
    has_track = err is not None and np.isfinite(err).sum() >= 5

    if "t_s" in df.columns and ({"lat_deg", "lon_deg"}.issubset(df.columns) or has_track):
        x, y = latlon_to_xy(df)
        t = df["t_s"].to_numpy(dtype=float)
        mask = _segment_mask_from_x(x) if x.size else np.zeros(len(df), dtype=bool)

        if mask.any():
            # Build exceedance flag for the selected dimension
            exceed_dim = np.zeros(len(df), dtype=bool)
            if has_track:
                finite = np.isfinite(err)
                threshold = H_MAX if dim == 'h' else V_MAX
                exceed_dim[mask & finite] = np.abs(err[mask & finite]) > threshold
            else:
                try:
                    if dim == 'h':
                        x_seg = x[mask]
                        y_seg = y[mask]
                        finite_xy = np.isfinite(x_seg) & np.isfinite(y_seg)
                        if finite_xy.sum() >= 2:
                            k, b = np.polyfit(x_seg[finite_xy], y_seg[finite_xy], 1)
                            y_pred = k * x + b
                        else:
                            y_med = float(np.nanmedian(y_seg)) if y_seg.size else 0.0
                            y_pred = np.full_like(y, y_med)
                        y_dev_full = y - y_pred
                        exceed_dim = np.abs(y_dev_full) > H_MAX
                    else:  # 'v'
                        z = df["rel_alt_m"].to_numpy(dtype=float)
                        z_ref = 10.0
                        v_dev_full = z - z_ref
                        exceed_dim = np.abs(v_dev_full) > V_MAX
                except Exception:
                    pass

            exceed_any = exceed_dim & mask

            if exceed_any.any():
                i0 = int(np.argmax(exceed_any))
                t0 = float(t[i0])
                mask_recover = mask & (t >= t0) & (t <= (t0 + RECOVER_T))

                if mask_recover.sum() >= 5:
                    df_after = df[mask_recover]
                    m_after = compute_metrics_for_test(df_after)
                    if m_after is not None:
                        if dim == 'h':
                            ok_after = (
                                (m_after.get("h_max_dev", float("inf")) <= H_MAX) and
                                (m_after.get("h_std", float("inf")) <= H_STD)
                            )
                        else:
                            ok_after = (
                                (m_after.get("v_max_dev", float("inf")) <= V_MAX) and
                                (m_after.get("v_std", float("inf")) <= V_STD)
                            )
                        if ok_after:
                            return "Wind-Recoverable"

    return "Unstable"


def extract_gust_level(test_id: str, description: str) -> Optional[int]:
    """Extract gust level number from test ID or description."""
    # Try test_id first: e.g., "gust_lvl_05" -> 5
    match = re.search(r'lvl_(\d+)', test_id)
    if match:
        return int(match.group(1))

    # Try description: e.g., "Gust L5: ..." -> 5
    match = re.search(r'L(\d+)', description)
    if match:
        return int(match.group(1))

    return None
