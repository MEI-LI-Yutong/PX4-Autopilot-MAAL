#!/usr/bin/env python3
"""
Compare simulation CSV vs. source CSV with time alignment.

- Mode "offset": sim_t aligns to src_t = sim_t + offset_s.
  (offset_s = src_time_at_sim_zero)
- Mode "match": search offset that minimizes wind-speed MSE.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Optional, Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from cycler import cycler


@dataclass
class SeriesData:
    t: np.ndarray
    wind: np.ndarray
    wind_n: np.ndarray
    wind_e: np.ndarray
    wind_z: np.ndarray
    pos_n: np.ndarray
    pos_e: np.ndarray
    pos_z: np.ndarray
    roll: np.ndarray
    pitch: np.ndarray
    yaw: np.ndarray


def _to_numeric(series: pd.Series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce")


def load_sim_csv(path: Path) -> SeriesData:
    df = pd.read_csv(path)
    df["t_s"] = _to_numeric(df.get("t_s"))
    df = df.dropna(subset=["t_s"])

    wind_x = _to_numeric(df.get("wind_x_m_s", pd.Series(dtype=float)))
    wind_y = _to_numeric(df.get("wind_y_m_s", pd.Series(dtype=float)))
    wind_z = _to_numeric(df.get("wind_z_m_s", pd.Series(dtype=float)))
    wind_m = _to_numeric(df.get("wind_m_s", pd.Series(dtype=float)))
    wind = wind_m.to_numpy()
    if np.all(np.isnan(wind)) and wind_x.notna().any() and wind_y.notna().any():
        wind = np.sqrt(wind_x.fillna(0.0) ** 2 + wind_y.fillna(0.0) ** 2 + wind_z.fillna(0.0) ** 2).to_numpy()

    lat = _to_numeric(df.get("lat_deg", pd.Series(dtype=float))).to_numpy()
    lon = _to_numeric(df.get("lon_deg", pd.Series(dtype=float))).to_numpy()
    rel_alt = _to_numeric(df.get("rel_alt_m", pd.Series(dtype=float))).to_numpy()

    lat0 = lat[0] if lat.size else 0.0
    lon0 = lon[0] if lon.size else 0.0
    r = 6378137.0
    pos_n = np.radians(lat - lat0) * r
    pos_e = np.radians(lon - lon0) * r * np.cos(np.radians(lat0))
    pos_z = rel_alt

    roll = _to_numeric(df.get("roll_deg", pd.Series(dtype=float))).to_numpy()
    pitch = _to_numeric(df.get("pitch_deg", pd.Series(dtype=float))).to_numpy()
    yaw = _to_numeric(df.get("yaw_deg", pd.Series(dtype=float))).to_numpy()

    return SeriesData(
        t=df["t_s"].to_numpy(),
        wind=wind,
        wind_n=wind_x.fillna(0.0).to_numpy(),
        wind_e=wind_y.fillna(0.0).to_numpy(),
        wind_z=wind_z.fillna(0.0).to_numpy(),
        pos_n=pos_n,
        pos_e=pos_e,
        pos_z=pos_z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def load_src_csv(path: Path) -> SeriesData:
    df = pd.read_csv(path)
    df["time_s"] = _to_numeric(df.get("time_s"))
    df = df.dropna(subset=["time_s"])

    wind_n = _to_numeric(df.get("windN", pd.Series(dtype=float))).fillna(0.0).to_numpy()
    wind_e = _to_numeric(df.get("windE", pd.Series(dtype=float))).fillna(0.0).to_numpy()
    wind = np.sqrt(wind_n ** 2 + wind_e ** 2)
    wind_z = np.zeros_like(wind_n)

    pos_n = _to_numeric(df.get("x", pd.Series(dtype=float))).to_numpy()
    pos_e = _to_numeric(df.get("y", pd.Series(dtype=float))).to_numpy()
    pos_z = _to_numeric(df.get("z", pd.Series(dtype=float))).to_numpy()

    roll = _to_numeric(df.get("roll", pd.Series(dtype=float))).to_numpy()
    pitch = _to_numeric(df.get("pitch", pd.Series(dtype=float))).to_numpy()
    yaw = _to_numeric(df.get("yaw", pd.Series(dtype=float))).to_numpy()

    return SeriesData(
        t=df["time_s"].to_numpy(),
        wind=wind,
        wind_n=wind_n,
        wind_e=wind_e,
        wind_z=wind_z,
        pos_n=pos_n,
        pos_e=pos_e,
        pos_z=pos_z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def interp_series(src: SeriesData, t_query: np.ndarray) -> SeriesData:
    def interp(arr: np.ndarray) -> np.ndarray:
        return np.interp(t_query, src.t, arr, left=np.nan, right=np.nan)

    return SeriesData(
        t=t_query,
        wind=interp(src.wind),
        wind_n=interp(src.wind_n),
        wind_e=interp(src.wind_e),
        wind_z=interp(src.wind_z),
        pos_n=interp(src.pos_n),
        pos_e=interp(src.pos_e),
        pos_z=interp(src.pos_z),
        roll=interp(src.roll),
        pitch=interp(src.pitch),
        yaw=interp(src.yaw),
    )

def apply_plot_style(font_size: int) -> None:
    try:
        import scienceplots  # noqa: F401
        plt.style.use(["science", "no-latex"])
    except Exception:
        plt.style.use("default")

    try:
        import seaborn as sns  # noqa: F401
        palette = sns.color_palette("colorblind", 8)
    except Exception:
        cmap = plt.get_cmap("tab10")
        palette = [cmap(i) for i in range(8)]
    plt.rcParams["axes.prop_cycle"] = cycler(color=palette)
    plt.rcParams.update({
        "font.size": font_size,
        "axes.labelsize": font_size,
        "axes.titlesize": font_size + 2,
        "xtick.labelsize": font_size - 1,
        "ytick.labelsize": font_size - 1,
        "legend.fontsize": font_size - 1,
        "figure.titlesize": font_size + 2,
    })


def compute_mse(sim_t: np.ndarray, sim_wind: np.ndarray, src: SeriesData, offset_s: float,
                t_start: float, t_end: float) -> Optional[float]:
    t_query = sim_t + offset_s
    mask = (sim_t >= t_start) & (sim_t <= t_end)
    if not np.any(mask):
        return None
    t_masked = t_query[mask]
    wind_src = np.interp(t_masked, src.t, src.wind, left=np.nan, right=np.nan)
    wind_sim = sim_wind[mask]
    valid = ~np.isnan(wind_src) & ~np.isnan(wind_sim)
    if np.sum(valid) < 10:
        return None
    diff = wind_sim[valid] - wind_src[valid]
    return float(np.mean(diff ** 2))


def match_offset(sim: SeriesData, src: SeriesData, window_s: Optional[float],
                 step_s: float, min_offset: Optional[float], max_offset: Optional[float]) -> Tuple[float, float]:
    t0 = float(sim.t[0])
    t1 = float(sim.t[-1])
    if window_s is None:
        window_s = t1 - t0
    window_s = max(1e-3, window_s)

    src_start = float(src.t[0])
    src_end = float(src.t[-1])

    if min_offset is None:
        min_offset = src_start - t0
    if max_offset is None:
        max_offset = src_end - (t0 + window_s)

    best_offset = min_offset
    best_mse = np.inf
    offset = min_offset
    while offset <= max_offset:
        mse = compute_mse(sim.t, sim.wind, src, offset, t0, t0 + window_s)
        if mse is not None and mse < best_mse:
            best_mse = mse
            best_offset = offset
        offset += step_s

    return best_offset, best_mse


def plot_compare(sim: SeriesData, src_aligned: SeriesData, out_path: Path, t_rel: np.ndarray,
                 mask: np.ndarray, include_yaw: bool, flip_sim_pitch: bool) -> None:
    sim_pitch = -sim.pitch if flip_sim_pitch else sim.pitch
    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    axes[0].plot(t_rel[mask], sim.wind[mask], label="sim_wind")
    axes[0].plot(t_rel[mask], src_aligned.wind[mask], label="src_wind", alpha=0.8)
    axes[0].set_ylabel("Wind speed (m/s)")
    axes[0].legend()

    axes[1].plot(t_rel[mask], sim.pos_n[mask], label="sim_N")
    axes[1].plot(t_rel[mask], sim.pos_e[mask], label="sim_E")
    axes[1].plot(t_rel[mask], sim.pos_z[mask], label="sim_Z")
    axes[1].plot(t_rel[mask], src_aligned.pos_n[mask], "--", label="src_X(N)")
    axes[1].plot(t_rel[mask], src_aligned.pos_e[mask], "--", label="src_Y(E)")
    axes[1].plot(t_rel[mask], src_aligned.pos_z[mask], "--", label="src_Z")
    axes[1].set_ylabel("Position (m)")
    axes[1].legend(ncol=3)

    axes[2].plot(t_rel[mask], sim.roll[mask], label="sim_roll")
    axes[2].plot(t_rel[mask], sim_pitch[mask], label="sim_pitch")
    axes[2].plot(t_rel[mask], src_aligned.roll[mask], "--", label="src_roll")
    axes[2].plot(t_rel[mask], src_aligned.pitch[mask], "--", label="src_pitch")
    if include_yaw:
        axes[2].plot(t_rel[mask], sim.yaw[mask], label="sim_yaw")
        axes[2].plot(t_rel[mask], src_aligned.yaw[mask], "--", label="src_yaw")
    axes[2].set_ylabel("Attitude (deg)")
    axes[2].set_xlabel("Aligned time (s)")
    axes[2].legend(ncol=3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)

def plot_errors(t_rel: np.ndarray, sim: SeriesData, src_aligned: SeriesData,
                out_path: Path, mask: np.ndarray) -> None:
    sim_err_pos_x = sim.pos_n - np.nanmean(sim.pos_n[mask])
    sim_err_pos_y = sim.pos_e - np.nanmean(sim.pos_e[mask])
    sim_err_pos_z = sim.pos_z - np.nanmean(sim.pos_z[mask])
    src_err_pos_x = src_aligned.pos_n - np.nanmean(src_aligned.pos_n[mask])
    src_err_pos_y = src_aligned.pos_e - np.nanmean(src_aligned.pos_e[mask])
    src_err_pos_z = src_aligned.pos_z - np.nanmean(src_aligned.pos_z[mask])

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(t_rel[mask], sim_err_pos_x[mask], label="sim_pos_err_x")
    axes[0].plot(t_rel[mask], src_err_pos_x[mask], "--", label="src_pos_err_x")
    axes[0].set_ylabel("X error (m)")
    axes[0].legend(ncol=2)

    axes[1].plot(t_rel[mask], sim_err_pos_y[mask], label="sim_pos_err_y")
    axes[1].plot(t_rel[mask], src_err_pos_y[mask], "--", label="src_pos_err_y")
    axes[1].set_ylabel("Y error (m)")
    axes[1].legend(ncol=2)

    axes[2].plot(t_rel[mask], sim_err_pos_z[mask], label="sim_pos_err_z")
    axes[2].plot(t_rel[mask], src_err_pos_z[mask], "--", label="src_pos_err_z")
    axes[2].set_ylabel("Z error (m)")
    axes[2].set_xlabel("Aligned time (s)")
    axes[2].legend(ncol=2)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def save_aligned(sim: SeriesData, src_aligned: SeriesData, out_path: Path, t_rel: np.ndarray,
                 src_time: np.ndarray) -> None:
    df = pd.DataFrame({
        "t_s": sim.t,
        "t_rel_s": t_rel,
        "src_time_s": src_time,
        "sim_wind_m_s": sim.wind,
        "src_wind_m_s": src_aligned.wind,
        "sim_n_m": sim.pos_n,
        "sim_e_m": sim.pos_e,
        "sim_z_m": sim.pos_z,
        "src_x_m": src_aligned.pos_n,
        "src_y_m": src_aligned.pos_e,
        "src_z_m": src_aligned.pos_z,
        "sim_roll_deg": sim.roll,
        "sim_pitch_deg": sim.pitch,
        "sim_yaw_deg": sim.yaw,
        "src_roll_deg": src_aligned.roll,
        "src_pitch_deg": src_aligned.pitch,
        "src_yaw_deg": src_aligned.yaw,
    })
    df.to_csv(out_path, index=False)

def compute_error_metrics(t_rel: np.ndarray, sim: SeriesData, src_aligned: SeriesData,
                          crop_start: float, crop_end: float) -> Tuple[Dict[str, np.ndarray], pd.DataFrame]:
    mask = (t_rel >= crop_start) & (t_rel <= crop_end)
    if not np.any(mask):
        raise SystemExit("No samples left after cropping")

    def center(arr: np.ndarray) -> np.ndarray:
        m = np.nanmean(arr[mask])
        return arr - m

    sim_x = center(sim.pos_n)
    sim_y = center(sim.pos_e)
    sim_z = center(sim.pos_z)
    src_x = center(src_aligned.pos_n)
    src_y = center(src_aligned.pos_e)
    src_z = center(src_aligned.pos_z)

    err_pos_x = sim_x - src_x
    err_pos_y = sim_y - src_y
    err_pos_z = sim_z - src_z

    err_roll = sim.roll - src_aligned.roll
    err_pitch = sim.pitch - src_aligned.pitch

    def rmse(v: np.ndarray) -> float:
        v = v[mask]
        return float(np.sqrt(np.nanmean(v ** 2)))

    def max_err(v: np.ndarray) -> float:
        v = v[mask]
        return float(np.nanmax(np.abs(v)))

    sim_err_pos_x = sim_x
    sim_err_pos_y = sim_y
    sim_err_pos_z = sim_z
    src_err_pos_x = src_x
    src_err_pos_y = src_y
    src_err_pos_z = src_z

    sim_err_roll = sim.roll - np.nanmean(sim.roll[mask])
    sim_err_pitch = sim.pitch - np.nanmean(sim.pitch[mask])
    src_err_roll = src_aligned.roll - np.nanmean(src_aligned.roll[mask])
    src_err_pitch = src_aligned.pitch - np.nanmean(src_aligned.pitch[mask])

    metrics = pd.DataFrame([
        {"metric": "pos_x", "rmse_err": rmse(err_pos_x), "max_err": max_err(err_pos_x),
         "rmse_sim": rmse(sim_err_pos_x), "rmse_src": rmse(src_err_pos_x)},
        {"metric": "pos_y", "rmse_err": rmse(err_pos_y), "max_err": max_err(err_pos_y),
         "rmse_sim": rmse(sim_err_pos_y), "rmse_src": rmse(src_err_pos_y)},
        {"metric": "pos_z", "rmse_err": rmse(err_pos_z), "max_err": max_err(err_pos_z),
         "rmse_sim": rmse(sim_err_pos_z), "rmse_src": rmse(src_err_pos_z)},
        {"metric": "roll", "rmse_err": rmse(err_roll), "max_err": max_err(err_roll),
         "rmse_sim": rmse(sim_err_roll), "rmse_src": rmse(src_err_roll)},
        {"metric": "pitch", "rmse_err": rmse(err_pitch), "max_err": max_err(err_pitch),
         "rmse_sim": rmse(sim_err_pitch), "rmse_src": rmse(src_err_pitch)},
    ])

    errors = {
        "pos_x": err_pos_x,
        "pos_y": err_pos_y,
        "pos_z": err_pos_z,
        "roll": err_roll,
        "pitch": err_pitch,
    }

    return errors, metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Align and compare sim CSV vs source CSV")
    parser.add_argument("--sim-csv", required=True, help="Simulation log CSV (from gust eval)")
    parser.add_argument("--src-csv", required=True, help="Source wind CSV (with time_s, windN, windE, x,y,z, roll,pitch,yaw)")
    parser.add_argument("--mode", choices=["offset", "match"], default="offset")
    parser.add_argument("--offset-s", type=float, default=0.0, help="Offset for offset mode (src_time = sim_time + offset)")
    parser.add_argument("--match-window-s", type=float, default=None, help="Window length for matching (seconds)")
    parser.add_argument("--match-step-s", type=float, default=0.5, help="Offset search step (seconds)")
    parser.add_argument("--match-min-offset", type=float, default=None, help="Minimum offset (seconds)")
    parser.add_argument("--match-max-offset", type=float, default=None, help="Maximum offset (seconds)")
    parser.add_argument("--out-dir", default=None, help="Output directory (default: sim_csv folder)")
    parser.add_argument("--zero-sim-time", type=float, default=None, help="Set this sim time as t=0 for plotting")
    parser.add_argument("--crop-start-s", type=float, default=30.0, help="Crop start time (aligned time, seconds)")
    parser.add_argument("--crop-end-s", type=float, default=200.0, help="Crop end time (aligned time, seconds)")
    parser.add_argument("--include-yaw", action="store_true", help="Include yaw in attitude plot")
    parser.add_argument("--flip-sim-pitch", action="store_true", help="Flip sim pitch around 0 for plotting")
    parser.add_argument("--font-size", type=int, default=14, help="Base font size for plots")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sim_path = Path(args.sim_csv)
    out_dir = Path(args.out_dir) if args.out_dir else sim_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    sim = load_sim_csv(Path(args.sim_csv))
    src = load_src_csv(Path(args.src_csv))
    apply_plot_style(args.font_size)

    if sim.t.size == 0 or src.t.size == 0:
        raise SystemExit("Empty time series in input CSVs")

    if args.mode == "match":
        best_offset, best_mse = match_offset(
            sim, src, args.match_window_s, args.match_step_s, args.match_min_offset, args.match_max_offset
        )
        offset = best_offset
        print(f"[align] best_offset_s={best_offset:.3f}, mse={best_mse:.6f}")
    else:
        offset = args.offset_s
        print(f"[align] offset_s={offset:.3f}")

    src_time = sim.t + offset
    src_aligned = interp_series(src, src_time)
    t0 = args.zero_sim_time if args.zero_sim_time is not None else float(sim.t[0])
    t_rel = sim.t - t0
    mask = (t_rel >= args.crop_start_s) & (t_rel <= args.crop_end_s)
    save_aligned(sim, src_aligned, out_dir / "aligned.csv", t_rel, src_time)
    plot_compare(sim, src_aligned, out_dir / "compare.png", t_rel, mask, args.include_yaw, args.flip_sim_pitch)
    errors, metrics = compute_error_metrics(t_rel, sim, src_aligned, args.crop_start_s, args.crop_end_s)
    plot_errors(t_rel, sim, src_aligned, out_dir / "errors.png", mask)
    metrics.to_csv(out_dir / "error_metrics.csv", index=False)
    print(f"[output] {out_dir / 'aligned.csv'}")
    print(f"[output] {out_dir / 'compare.png'}")
    print(f"[output] {out_dir / 'errors.png'}")
    print(f"[output] {out_dir / 'error_metrics.csv'}")


if __name__ == "__main__":
    main()
