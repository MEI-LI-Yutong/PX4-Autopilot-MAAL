from __future__ import annotations

from typing import Dict, Optional, Tuple

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
    "h_max",
    "h_std",
    "v_max",
    "v_std",
    "att_max",
    "act_margin",
    "recovery",
]

DIM_LABELS = [
    "H-Max",
    "H-Std",
    "V-Max",
    "V-Std",
    "Att-Max",
    "Act-Margin",
    "Recovery",
]


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _score_inverse(value: float, limit: float) -> float:
    if not np.isfinite(value) or limit <= 0:
        return float("nan")
    return _clamp01(1.0 - (value / limit))


def _mean_ignore_nan(values: list[float]) -> float:
    vals = [v for v in values if np.isfinite(v)]
    if not vals:
        return float("nan")
    return float(np.mean(vals))


def _actuator_columns(df: pd.DataFrame, prefix: str) -> list[str]:
    pattern = re.compile(rf"^{re.escape(prefix)}\d+$")
    return [c for c in df.columns if pattern.match(c)]


def _extract_actuator_baseline(df: pd.DataFrame, prefix: str, samples: int = 25) -> Optional[float]:
    cols = _actuator_columns(df, prefix)
    if not cols:
        return None
    arr = df[cols].to_numpy(dtype=float)
    if arr.size == 0:
        return None
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
    score = _score_inverse(delta, limit - base)
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
    """Compute 8D capability scores (0-1) with raw metric breakdown."""
    df = select_analysis_df(df)
    if df.empty:
        return {"scores": {}, "raw": {}, "overall": float("nan")}

    h_err, v_err = compute_track_errors_from_raw(df)

    h_max = float(np.nanmax(np.abs(h_err))) if h_err is not None and np.isfinite(h_err).any() else float("nan")
    h_std = float(np.nanstd(h_err)) if h_err is not None and np.isfinite(h_err).any() else float("nan")
    v_max = float(np.nanmax(np.abs(v_err))) if v_err is not None and np.isfinite(v_err).any() else float("nan")
    v_std = float(np.nanstd(v_err)) if v_err is not None and np.isfinite(v_err).any() else float("nan")

    roll = pd.to_numeric(df.get("roll_deg"), errors="coerce") if "roll_deg" in df.columns else None
    pitch = pd.to_numeric(df.get("pitch_deg"), errors="coerce") if "pitch_deg" in df.columns else None
    att_max = float("nan")
    if roll is not None and pitch is not None:
        att_max = float(np.nanmax([np.nanmax(np.abs(roll)), np.nanmax(np.abs(pitch))]))
    elif roll is not None:
        att_max = float(np.nanmax(np.abs(roll)))
    elif pitch is not None:
        att_max = float(np.nanmax(np.abs(pitch)))

    max_u = _extract_actuator_max(df, "u")
    base_u = _extract_actuator_baseline(df, "u", samples=25)
    score_u, delta_u = _actuator_margin_score(base_u, max_u, ACTUATOR_MAX)

    max_s = _extract_actuator_max(df, "s")
    base_s = _extract_actuator_baseline(df, "s", samples=25)
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

    scores = {
        "h_max": _score_inverse(h_max, H_MAX),
        "h_std": _score_inverse(h_std, H_STD),
        "v_max": _score_inverse(v_max, V_MAX),
        "v_std": _score_inverse(v_std, V_STD),
        "att_max": _score_inverse(att_max, ANG_MAX),
        "act_margin": act_margin_score,
        "recovery": _score_inverse(t_rec, RECOVER_T) if t_rec is not None else float("nan"),
    }

    overall = _mean_ignore_nan([scores.get(k, float("nan")) for k in DIM_ORDER])

    raw = {
        "h_max_dev_m": h_max,
        "h_std_m": h_std,
        "v_max_dev_m": v_max,
        "v_std_m": v_std,
        "att_max_deg": att_max,
        "actuator_peak": float(max_u) if max_u is not None else float("nan"),
        "actuator_baseline": float(base_u) if base_u is not None else float("nan"),
        "actuator_delta": float(delta_u) if np.isfinite(delta_u) else float("nan"),
        "servo_peak": float(max_s) if max_s is not None else float("nan"),
        "servo_baseline": float(base_s) if base_s is not None else float("nan"),
        "servo_delta": float(delta_s) if np.isfinite(delta_s) else float("nan"),
        "actuator_delta_effective": act_delta_effective,
        "recovery_time_s": float(t_rec) if t_rec is not None else float("nan"),
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
