from __future__ import annotations

from typing import Dict, Optional, Tuple

import math
import numpy as np
import pandas as pd
import re

from .gust_metrics import (
    ANG_MAX,
    H_MAX,
    H_STD,
    V_MAX,
    V_STD,
    RECOVER_T,
    select_analysis_df,
    compute_track_errors_from_raw,
)

# Limits for scoring
ACTUATOR_MAX = 1000.0
SERVO_MAX = 1.0
ACTUATOR_SAT_RATIO = 0.98
SERVO_SAT_RATIO = 0.98

DIM_ORDER = [
    "h_pos_track_acc",
    "v_pos_track_acc",
    "h_pos_robustness",
    "v_pos_robustness",
    "pitch_att_track_acc",
    "roll_att_track_acc",
    "yaw_att_track_acc",
    "act_margin",
    "recovery",
    "exceed_area",
]

DIM_LABELS = [
    "H Pos Track Acc",
    "V Pos Track Acc",
    "H Pos Robustness",
    "V Pos Robustness",
    "Pitch Track Acc",
    "Roll Track Acc",
    "Yaw Track Acc",
    "Act-Margin",
    "Recovery",
    "Exceed-Area",
]

RADAR_DIM_ORDER = [
    "h_pos_track_acc",
    "v_pos_track_acc",
    "h_pos_robustness",
    "v_pos_robustness",
    "pitch_att_track_acc",
    "roll_att_track_acc",
    "yaw_att_track_acc",
    "act_margin",
    "recovery",
    "exceed_area",
]

RADAR_DIM_LABELS = [
    "H Pos\nTrack Acc",
    "V Pos\nTrack Acc",
    "H Pos\nRobustness",
    "V Pos\nRobustness",
    "Pitch \nTrack Acc",
    "Roll \nTrack Acc",
    "Yaw \nTrack Acc",
    "Actuator\nMargin",
    "Recovery",
    "Exceed\nArea",
]

def _integral_over_time(t: np.ndarray, y: np.ndarray) -> float:
    mask = np.isfinite(t) & np.isfinite(y)
    if mask.sum() < 2:
        return float("nan")
    t_valid = t[mask]
    y_valid = y[mask]
    order = np.argsort(t_valid)
    t_sorted = t_valid[order]
    y_sorted = y_valid[order]
    if t_sorted[-1] <= t_sorted[0]:
        return float("nan")
    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(y_sorted, t_sorted))
    return float(np.trapz(y_sorted, t_sorted))


def _exceed_area(err: Optional[np.ndarray], threshold: float, t: np.ndarray) -> float:
    if err is None:
        return float("nan")
    exceed = np.maximum(0.0, np.abs(err) - threshold)
    return _integral_over_time(t, exceed)


def _attitude_hold_peak(df: pd.DataFrame, col: str, circular: bool = False, baseline_samples: int = 25) -> float:
    if col not in df.columns:
        return float("nan")
    arr = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
    finite = np.isfinite(arr)
    if finite.sum() < 3:
        return float("nan")

    arr_valid = arr[finite]
    baseline = float(np.nanmedian(arr_valid[: min(baseline_samples, arr_valid.size)]))
    if circular:
        dev = ((arr_valid - baseline + 180.0) % 360.0) - 180.0
    else:
        dev = arr_valid - baseline
    return float(np.nanmax(np.abs(dev)))

def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _score_sigmoid(value: float, limit: float, k: float = 6.0) -> float:
    if not np.isfinite(value) or limit <= 0:
        return float("nan")
    x = float(value) / float(limit)
    score = 1.0 / (1.0 + math.exp(k * (x - 1.0)))
    return _clamp01(score)


def _mean_ignore_nan(values: list[float]) -> float:
    vals = [v for v in values if np.isfinite(v)]
    if not vals:
        return float("nan")
    return float(np.mean(vals))


def _actuator_columns(df: pd.DataFrame, prefix: str) -> list[str]:
    pattern = re.compile(rf"^{re.escape(prefix)}\d+$")
    return [c for c in df.columns if pattern.match(c)]


def _extract_actuator_baseline(
    df: pd.DataFrame,
    prefix: str,
    samples: int = 25,
    eps: float = 0.0,
) -> Optional[float]:
    cols = _actuator_columns(df, prefix)
    if not cols:
        return None
    arr = df[cols].to_numpy(dtype=float)
    if arr.size == 0:
        return None
    if eps > 0.0:
        active_mask = np.nanmax(np.abs(arr), axis=0) >= eps
        if not np.any(active_mask):
            return None
        arr = arr[:, active_mask]
    head = arr[:samples, :]
    if head.size == 0:
        return None
    return float(np.nanmean(np.abs(head)))


def _extract_actuator_max(df: pd.DataFrame, prefix: str) -> Optional[float]:
    cols = _actuator_columns(df, prefix)
    if not cols:
        return None
    arr = df[cols].to_numpy(dtype=float)
    if arr.size == 0:
        return None
    return float(np.nanmax(np.abs(arr)))


def _actuator_margin_score(base: Optional[float], peak: Optional[float], limit: float) -> Tuple[float, float]:
    if base is None or peak is None:
        return float("nan"), float("nan")
    delta = float(peak - base)
    if limit <= base:
        return float("nan"), delta
    score = _score_sigmoid(delta, limit - base)
    return score, delta


def _recovery_time(df: pd.DataFrame, h_err: Optional[np.ndarray], v_err: Optional[np.ndarray]) -> Optional[float]:
    if "t_s" not in df.columns:
        return None
    t = df["t_s"].to_numpy(dtype=float)
    if t.size == 0:
        return None

    exceed = np.zeros_like(t, dtype=bool)
    if h_err is not None and np.isfinite(h_err).any():
        exceed |= np.abs(h_err) > H_MAX
    if v_err is not None and np.isfinite(v_err).any():
        exceed |= np.abs(v_err) > V_MAX

    if not exceed.any():
        return 0.0

    idx0 = int(np.argmax(exceed))
    t0 = t[idx0]
    within = (~exceed) & (t >= t0)
    if not within.any():
        return None

    t1 = t[np.argmax(within)]
    return float(max(0.0, t1 - t0))



def compute_dimension_breakdown(df: pd.DataFrame) -> Dict[str, Dict[str, float]]:
    """Compute capability scores (0-1) with raw metric breakdown."""
    df = select_analysis_df(df)
    if df.empty:
        return {"scores": {}, "raw": {}, "overall": float("nan")}

    h_err, v_err = compute_track_errors_from_raw(df)

    h_max = float(np.nanmax(np.abs(h_err))) if h_err is not None and np.isfinite(h_err).any() else float("nan")
    h_std = float(np.nanstd(h_err)) if h_err is not None and np.isfinite(h_err).any() else float("nan")
    v_max = float(np.nanmax(np.abs(v_err))) if v_err is not None and np.isfinite(v_err).any() else float("nan")
    v_std = float(np.nanstd(v_err)) if v_err is not None and np.isfinite(v_err).any() else float("nan")

    pitch_att_max = _attitude_hold_peak(df, "pitch_deg", circular=False)
    roll_att_max = _attitude_hold_peak(df, "roll_deg", circular=False)
    yaw_att_max = _attitude_hold_peak(df, "yaw_deg", circular=True)

    max_u = _extract_actuator_max(df, "u")
    base_u = _extract_actuator_baseline(df, "u", samples=25, eps=0.01)
    score_u, delta_u = _actuator_margin_score(base_u, max_u, ACTUATOR_MAX)

    max_s = _extract_actuator_max(df, "s")
    base_s = _extract_actuator_baseline(df, "s", samples=25, eps=0.01)
    score_s, delta_s = _actuator_margin_score(base_s, max_s, SERVO_MAX)

    act_scores = [s for s in (score_u, score_s) if np.isfinite(s)]
    act_margin_score = float(min(act_scores)) if act_scores else float("nan")
    act_delta_effective = float("nan")
    if np.isfinite(score_u) and np.isfinite(score_s):
        act_delta_effective = float(delta_u if score_u <= score_s else delta_s)
    elif np.isfinite(score_u):
        act_delta_effective = float(delta_u)
    elif np.isfinite(score_s):
        act_delta_effective = float(delta_s)

    t_rec = _recovery_time(df, h_err, v_err)

    t = pd.to_numeric(df["t_s"], errors="coerce").to_numpy(dtype=float) if "t_s" in df.columns else np.array([])
    duration = float(np.nanmax(t) - np.nanmin(t)) if t.size and np.isfinite(t).any() else float("nan")

    area_h = _exceed_area(h_err, H_MAX, t) if t.size else float("nan")
    area_v = _exceed_area(v_err, V_MAX, t) if t.size else float("nan")
    area_h_ratio = float(area_h / (H_MAX * duration)) if np.isfinite(area_h) and np.isfinite(duration) and duration > 0.0 else float("nan")
    area_v_ratio = float(area_v / (V_MAX * duration)) if np.isfinite(area_v) and np.isfinite(duration) and duration > 0.0 else float("nan")
    area_candidates = [r for r in (area_h_ratio, area_v_ratio) if np.isfinite(r)]
    exceed_area_ratio = float(max(area_candidates)) if area_candidates else float("nan")

    scores = {
        "h_pos_track_acc": _score_sigmoid(h_max, H_MAX),
        "v_pos_track_acc": _score_sigmoid(v_max, V_MAX),
        "h_pos_robustness": _score_sigmoid(h_std, H_STD),
        "v_pos_robustness": _score_sigmoid(v_std, V_STD),
        "pitch_att_track_acc": _score_sigmoid(pitch_att_max, ANG_MAX),
        "roll_att_track_acc": _score_sigmoid(roll_att_max, ANG_MAX),
        "yaw_att_track_acc": _score_sigmoid(yaw_att_max, ANG_MAX),
        "act_margin": act_margin_score,
        "recovery": _score_sigmoid(t_rec, RECOVER_T) if t_rec is not None else float("nan"),
        "exceed_area": _score_sigmoid(exceed_area_ratio, 1.0),
    }

    overall = _mean_ignore_nan([scores.get(k, float("nan")) for k in DIM_ORDER])

    raw = {
        "h_max_dev_m": h_max,
        "h_std_m": h_std,
        "v_max_dev_m": v_max,
        "v_std_m": v_std,
        "pitch_att_hold_peak_deg": pitch_att_max,
        "roll_att_hold_peak_deg": roll_att_max,
        "yaw_att_hold_peak_deg": yaw_att_max,
        "actuator_peak": float(max_u) if max_u is not None else float("nan"),
        "actuator_baseline": float(base_u) if base_u is not None else float("nan"),
        "actuator_delta": float(delta_u) if np.isfinite(delta_u) else float("nan"),
        "servo_peak": float(max_s) if max_s is not None else float("nan"),
        "servo_baseline": float(base_s) if base_s is not None else float("nan"),
        "servo_delta": float(delta_s) if np.isfinite(delta_s) else float("nan"),
        "actuator_delta_effective": act_delta_effective,
        "recovery_time_s": float(t_rec) if t_rec is not None else float("nan"),
        "h_exceed_area_m_s": area_h,
        "v_exceed_area_m_s": area_v,
        "exceed_area_ratio": exceed_area_ratio,
    }

    return {"scores": scores, "raw": raw, "overall": float(overall)}


def compute_dimension_scores(df: pd.DataFrame) -> Dict[str, float]:
    """Backward-compatible wrapper returning only scores."""
    return compute_dimension_breakdown(df).get("scores", {})


def compute_actuator_saturation_stats(df: pd.DataFrame) -> Dict[str, Dict[str, Dict[str, float]]]:
    """Compute per-actuator saturation stats for outputs (u*) and servos (s*)."""
    df = select_analysis_df(df)
    if df.empty or "t_s" not in df.columns:
        return {}
    t = pd.to_numeric(df["t_s"], errors="coerce").to_numpy(dtype=float)
    if t.size < 2 or not np.isfinite(t).any():
        return {}
    dt = np.diff(t, prepend=t[0])
    total_time = float(max(0.0, t[-1] - t[0]))

    def _stats_for(prefix: str, limit: float, ratio: float) -> Dict[str, Dict[str, float]]:
        cols = _actuator_columns(df, prefix)
        if not cols:
            return {}
        out: Dict[str, Dict[str, float]] = {}
        threshold = limit * ratio
        for col in cols:
            values = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
            if values.size == 0:
                continue
            mask = np.isfinite(values)
            sat = np.zeros_like(values, dtype=bool)
            sat[mask] = np.abs(values[mask]) >= threshold
            if sat.any():
                first_idx = int(np.argmax(sat))
                first_t = float(t[first_idx])
                duration = float(np.sum(dt[sat]))
            else:
                first_t = float("nan")
                duration = 0.0
            ratio_time = float(duration / total_time) if total_time > 0 else float("nan")
            out[col] = {
                "sat_any": float(bool(sat.any())),
                "sat_first_s": first_t,
                "sat_duration_s": duration,
                "sat_ratio": ratio_time,
            }
        return out

    return {
        "u": _stats_for("u", ACTUATOR_MAX, ACTUATOR_SAT_RATIO),
        "s": _stats_for("s", SERVO_MAX, SERVO_SAT_RATIO),
        "meta": {"total_time_s": float(total_time)},
    }
