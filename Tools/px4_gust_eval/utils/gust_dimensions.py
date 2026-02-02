from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import pandas as pd

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
WIND_CORR_MAX = 0.8

DIM_ORDER = [
    "h_max",
    "h_std",
    "v_max",
    "v_std",
    "att_max",
    "act_margin",
    "recovery",
    "wind_sense",
]

DIM_LABELS = [
    "H-Max",
    "H-Std",
    "V-Max",
    "V-Std",
    "Att-Max",
    "Act-Margin",
    "Recovery",
    "Wind-Corr",
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


def _extract_actuator_baseline(df: pd.DataFrame, samples: int = 25) -> Optional[float]:
    cols = [c for c in df.columns if c.startswith("u")]
    if not cols:
        return None
    arr = df[cols].to_numpy(dtype=float)
    if arr.size == 0:
        return None
    head = arr[:samples, :]
    if head.size == 0:
        return None
    return float(np.nanmean(np.abs(head)))


def _extract_actuator_max(df: pd.DataFrame) -> Optional[float]:
    cols = [c for c in df.columns if c.startswith("u")]
    if not cols:
        return None
    arr = df[cols].to_numpy(dtype=float)
    if arr.size == 0:
        return None
    return float(np.nanmax(np.abs(arr)))


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


def _wind_sensitivity(df: pd.DataFrame, h_err: Optional[np.ndarray], v_err: Optional[np.ndarray]) -> Optional[float]:
    if "wind_m_s" not in df.columns:
        return None
    wind = pd.to_numeric(df["wind_m_s"], errors="coerce").to_numpy(dtype=float)
    if wind.size == 0 or not np.isfinite(wind).any():
        return None

    corrs = []
    for err in (h_err, v_err):
        if err is None:
            continue
        mask = np.isfinite(wind) & np.isfinite(err)
        if mask.sum() < 5:
            continue
        c = np.corrcoef(wind[mask], err[mask])[0, 1]
        if np.isfinite(c):
            corrs.append(abs(c))
    if not corrs:
        return None
    return float(max(corrs))


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

    max_u = _extract_actuator_max(df)
    base_u = _extract_actuator_baseline(df, samples=25)
    act_delta = float("nan")
    if max_u is not None and base_u is not None:
        act_delta = float(max_u - base_u)

    t_rec = _recovery_time(df, h_err, v_err)
    corr = _wind_sensitivity(df, h_err, v_err)

    scores = {
        "h_max": _score_inverse(h_max, H_MAX),
        "h_std": _score_inverse(h_std, H_STD),
        "v_max": _score_inverse(v_max, V_MAX),
        "v_std": _score_inverse(v_std, V_STD),
        "att_max": _score_inverse(att_max, ANG_MAX),
        "act_margin": _score_inverse(act_delta, ACTUATOR_MAX - base_u) if base_u is not None and ACTUATOR_MAX > base_u else float("nan"),
        "recovery": _score_inverse(t_rec, RECOVER_T) if t_rec is not None else float("nan"),
        "wind_sense": _score_inverse(corr, WIND_CORR_MAX) if corr is not None else float("nan"),
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
        "actuator_delta": act_delta,
        "recovery_time_s": float(t_rec) if t_rec is not None else float("nan"),
        "wind_err_corr": float(corr) if corr is not None else float("nan"),
    }

    return {"scores": scores, "raw": raw, "overall": float(overall)}


def compute_dimension_scores(df: pd.DataFrame) -> Dict[str, float]:
    """Backward-compatible wrapper returning only scores."""
    return compute_dimension_breakdown(df).get("scores", {})
