#!/usr/bin/env python3
"""
Wind Stability Experiment Analyzer

Inputs:
- tasks JSON file (defines test order and IDs)
- results directory containing per-test CSVs named <test_id>.csv

Output:
- A multi-page PDF with:
  1) Summary: overlaid XY trajectories and altitudes for all tests
  2) Per-test pages (4 plots per test):
     - Attitude (roll/pitch/yaw) vs time
     - Horizontal trajectory (X/Y from lat/lon)
     - Relative altitude vs time
     - Wind components (x/y/z) and magnitude vs time

Usage:
  uv run Tools/px4_gust_eval/analyze_results.py \
    Tools/px4_gust_eval/tasks/stability_tests.json \
    Tools/px4_gust_eval/logs/stability/run_20250929_131736 \
    --output Tools/px4_gust_eval/logs/stability/analysis_report.pdf
"""

from __future__ import annotations

import argparse
from pathlib import Path
import json
import math
from typing import Dict, List, Tuple, Optional

import pandas as pd
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import seaborn as sns

try:
    import scienceplots  # noqa: F401
    plt.style.use(["science", "no-latex"])  # ensure no LaTeX dependency
except Exception:
    # Fallback gracefully if SciencePlots is unavailable
    sns.set_theme(context="paper", style="whitegrid")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Analyze wind stability experiment CSVs and produce a PDF report")
    p.add_argument("tasks_json", type=Path, help="Path to tasks JSON used to run the experiments")
    p.add_argument("results_dir", type=Path, help="Directory containing per-test CSV files (<test_id>.csv)")
    p.add_argument("--output", type=Path, default=None, help="Output PDF path (default: <results_dir>/analysis_report.pdf)")
    p.add_argument("--dpi", type=int, default=200, help="Figure DPI for rasterized elements")
    return p.parse_args()


def read_tasks(tasks_path: Path) -> Tuple[str, List[Dict]]:
    data = json.loads(tasks_path.read_text(encoding="utf-8"))
    suite = data.get("test_suite", tasks_path.stem)
    tests = data.get("wind_gust_tests", data.get("tests", []))
    return suite, tests


def load_csv(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    # Coerce numerics
    for col in [
        "t_s", "lat_deg", "lon_deg", "rel_alt_m", "abs_alt_m",
        "roll_deg", "pitch_deg", "yaw_deg",
        "sp_lat_deg", "sp_lon_deg", "sp_abs_alt_m",
        "wind_x_m_s", "wind_y_m_s", "wind_z_m_s", "wind_m_s",
    ]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def latlon_to_xy(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
    """Convert lat/lon to local XY in meters using first valid sample as origin.
    X: Easting, Y: Northing
    """
    lat = df["lat_deg"].to_numpy(dtype=float)
    lon = df["lon_deg"].to_numpy(dtype=float)
    # Find first finite sample
    mask = np.isfinite(lat) & np.isfinite(lon)
    if not mask.any():
        return np.array([]), np.array([])
    i0 = int(np.argmax(mask))
    lat0 = math.radians(lat[i0])
    lon0 = math.radians(lon[i0])
    R = 6378137.0
    x = (np.radians(lon) - lon0) * math.cos(lat0) * R
    y = (np.radians(lat) - lat0) * R
    return x, y


def _segment_mask_from_x(x: np.ndarray) -> np.ndarray:
    """Return boolean mask for mid-flight segment.
    Primary rule: 5 m <= x <= 95 m.
    Fallback: 10th..90th percentile of x if primary yields too few samples.
    """
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


def compute_metrics_for_test(df: pd.DataFrame) -> Optional[Dict[str, float]]:
    """Compute stability metrics on mid-flight segment.
    Returns dict with keys: h_max_dev, h_std, v_max_dev, v_std, max_abs_roll, max_abs_pitch.
    """
    if df.empty or not {"lat_deg", "lon_deg"}.issubset(df.columns):
        return None

    x, y = latlon_to_xy(df)
    t = df.get("t_s", pd.Series(dtype=float)).to_numpy(dtype=float) if "t_s" in df.columns else np.arange(len(x), dtype=float)
    mask = _segment_mask_from_x(x)
    if mask.sum() < 5:
        return None

    # Horizontal deviation (Y deviation from reference)
    y_seg = y[mask]
    y_ref = np.nanmedian(y_seg)
    h_dev = y_seg - y_ref
    h_max = float(np.nanmax(np.abs(h_dev))) if y_seg.size else float("nan")
    h_std = float(np.nanstd(h_dev)) if y_seg.size else float("nan")

    # Vertical deviation (relative altitude) vs 10m reference
    if "rel_alt_m" in df.columns:
        z = df["rel_alt_m"].to_numpy(dtype=float)
        z_seg = z[mask]
        z_ref = 10.0  # Use 10m as reference height
        v_dev = z_seg - z_ref
        v_max = float(np.nanmax(np.abs(v_dev))) if z_seg.size else float("nan")
        v_std = float(np.nanstd(v_dev)) if z_seg.size else float("nan")
    else:
        v_max = float("nan"); v_std = float("nan")

    # Attitude (within segment)
    def _max_abs(col: str) -> float:
        if col in df.columns:
            arr = df[col].to_numpy(dtype=float)[mask]
            if arr.size:
                return float(np.nanmax(np.abs(arr)))
        return float("nan")

    max_abs_roll = _max_abs("roll_deg")
    max_abs_pitch = _max_abs("pitch_deg")

    # Segment timing
    t_seg = t[mask]
    t_len = float(t_seg[-1] - t_seg[0]) if t_seg.size > 1 else 0.0

    return {
        "h_max_dev": h_max,
        "h_std": h_std,
        "v_max_dev": v_max,
        "v_std": v_std,
        "max_abs_roll": max_abs_roll,
        "max_abs_pitch": max_abs_pitch,
        "t_seg_len": t_len,
    }


def compute_grades(dfs: Dict[str, pd.DataFrame], id_to_cfg: Dict[str, Dict]) -> Tuple[Dict[str, str], Dict[str, Dict[str, float]]]:
    """Compute stability grade for each test.
    Grades: "Level 1", "Level 2", "Unstable", "Not launched".

    Level 2 rule: for tests that fail base thresholds on the mid-flight segment,
    detect the first time any instantaneous deviation (|Y cross-track|, |Z deviation|,
    |roll|, |pitch|) exceeds its threshold. If within the following RECOVER_T seconds
    a window of data (the window [t_exceed, t_exceed + RECOVER_T]) produces metrics
    that are all within thresholds, grade as "Level 2"; otherwise "Unstable".
    """
    grades: Dict[str, str] = {}
    metrics: Dict[str, Dict[str, float]] = {}

    # Thresholds
    H_MAX = 1.5
    H_STD = 0.75
    V_MAX = 3.0
    V_STD = 1.5
    ANG_MAX = 45.0  # deg for roll/pitch
    RECOVER_T = 10.0  # seconds

    for tid, df in dfs.items():
        m = compute_metrics_for_test(df)
        if m is None:
            grades[tid] = "Not launched"
            continue
        metrics[tid] = m
        base_ok = (
            (m.get("h_max_dev", float("inf")) <= H_MAX) and
            (m.get("h_std", float("inf")) <= H_STD) and
            (m.get("v_max_dev", float("inf")) <= V_MAX) and
            (m.get("v_std", float("inf")) <= V_STD) and
            (m.get("max_abs_roll", float("inf")) <= ANG_MAX) and
            (m.get("max_abs_pitch", float("inf")) <= ANG_MAX)
        )

        gust_cfg = id_to_cfg.get(tid, {})
        gust_len = float(gust_cfg.get("gust_length", 0.0) or 0.0)
        is_gust = gust_len > 0.0

        if base_ok:
            grades[tid] = "Level 1"
            print(f"[DEBUG] {tid}: Level 1 - all metrics within thresholds")
            continue

        print(f"[DEBUG] {tid}: Failed Level 1 - gust_len={gust_len}, is_gust={is_gust}")
        print(f"[DEBUG] {tid}: Metrics - h_max={m.get('h_max_dev', 'N/A'):.2f} (≤{H_MAX}), h_std={m.get('h_std', 'N/A'):.2f} (≤{H_STD}), v_max={m.get('v_max_dev', 'N/A'):.2f} (≤{V_MAX}), v_std={m.get('v_std', 'N/A'):.2f} (≤{V_STD}), roll={m.get('max_abs_roll', 'N/A'):.1f} (≤{ANG_MAX}), pitch={m.get('max_abs_pitch', 'N/A'):.1f} (≤{ANG_MAX})")
        print(f"[DEBUG] {tid}: Level 1 checks - h_max_ok:{m.get('h_max_dev', float('inf')) <= H_MAX}, h_std_ok:{m.get('h_std', float('inf')) <= H_STD}, v_max_ok:{m.get('v_max_dev', float('inf')) <= V_MAX}, v_std_ok:{m.get('v_std', float('inf')) <= V_STD}, roll_ok:{m.get('max_abs_roll', float('inf')) <= ANG_MAX}, pitch_ok:{m.get('max_abs_pitch', float('inf')) <= ANG_MAX}")

        # Check recovery within RECOVER_T after first exceedance for all tests
        if "t_s" in df.columns and {"lat_deg", "lon_deg"}.issubset(df.columns):
            print(f"[DEBUG] {tid}: Checking Level 2 recovery logic...")
            x, y = latlon_to_xy(df)
            t = df["t_s"].to_numpy(dtype=float)
            mask = _segment_mask_from_x(x)
            if mask.any():
                # Build instantaneous exceedance flags on the mid-flight segment
                # Horizontal cross-track deviation against best-fit line
                exceed_h = np.zeros_like(mask, dtype=bool)
                try:
                    x_seg = x[mask]
                    y_seg = y[mask]
                    finite_xy = np.isfinite(x_seg) & np.isfinite(y_seg)
                    if finite_xy.sum() >= 2:
                        k, b = np.polyfit(x_seg[finite_xy], y_seg[finite_xy], 1)
                        y_pred = k * x + b
                    else:
                        # Fallback: use median as straight line (zero slope)
                        y_med = float(np.nanmedian(y_seg)) if y_seg.size else 0.0
                        y_pred = np.full_like(y, y_med)
                    y_dev_full = y - y_pred
                    exceed_h = np.abs(y_dev_full) > H_MAX
                except Exception:
                    pass

                # Vertical deviation against 10m reference
                exceed_v = np.zeros_like(mask, dtype=bool)
                if "rel_alt_m" in df.columns:
                    try:
                        z = df["rel_alt_m"].to_numpy(dtype=float)
                        z_ref = 10.0  # Use 10m as reference height
                        v_dev_full = z - z_ref
                        exceed_v = np.abs(v_dev_full) > V_MAX
                    except Exception:
                        pass

                # Attitude exceedance
                exceed_roll = np.zeros_like(mask, dtype=bool)
                exceed_pitch = np.zeros_like(mask, dtype=bool)
                if "roll_deg" in df.columns:
                    try:
                        exceed_roll = np.abs(df["roll_deg"].to_numpy(dtype=float)) > ANG_MAX
                    except Exception:
                        pass
                if "pitch_deg" in df.columns:
                    try:
                        exceed_pitch = np.abs(df["pitch_deg"].to_numpy(dtype=float)) > ANG_MAX
                    except Exception:
                        pass

                exceed_any = (exceed_h | exceed_v | exceed_roll | exceed_pitch) & mask
                exceed_count = exceed_any.sum()
                print(f"[DEBUG] {tid}: Exceedances - h:{exceed_h.sum()}, v:{exceed_v.sum()}, roll:{exceed_roll.sum()}, pitch:{exceed_pitch.sum()}, total:{exceed_count}")

                if exceed_any.any():
                    # First exceedance time
                    i0 = int(np.argmax(exceed_any))
                    t0 = float(t[i0])
                    print(f"[DEBUG] {tid}: First exceedance at t={t0:.1f}s, checking recovery window [t0, t0+{RECOVER_T}]")
                    mask_recover = mask & (t >= t0) & (t <= (t0 + RECOVER_T))
                    recover_count = mask_recover.sum()
                    print(f"[DEBUG] {tid}: Recovery window has {recover_count} samples (need ≥5)")

                    if recover_count >= 5:
                        df_after = df[mask_recover]
                        m_after = compute_metrics_for_test(df_after)
                        if m_after is not None:
                            print(f"[DEBUG] {tid}: Recovery metrics - h_max={m_after.get('h_max_dev', 'N/A'):.2f}, h_std={m_after.get('h_std', 'N/A'):.2f}, v_max={m_after.get('v_max_dev', 'N/A'):.2f}, v_std={m_after.get('v_std', 'N/A'):.2f}, roll={m_after.get('max_abs_roll', 'N/A'):.1f}, pitch={m_after.get('max_abs_pitch', 'N/A'):.1f}")
                            ok_after = (
                                (m_after.get("h_max_dev", float("inf")) <= H_MAX) and
                                (m_after.get("h_std", float("inf")) <= H_STD) and
                                (m_after.get("v_max_dev", float("inf")) <= V_MAX) and
                                (m_after.get("v_std", float("inf")) <= V_STD) and
                                (m_after.get("max_abs_roll", float("inf")) <= ANG_MAX) and
                                (m_after.get("max_abs_pitch", float("inf")) <= ANG_MAX)
                            )
                            print(f"[DEBUG] {tid}: Recovery successful: {ok_after}")
                            if ok_after:
                                grades[tid] = "Level 2"
                                print(f"[DEBUG] {tid}: Assigned Level 2")
                                continue
                        else:
                            print(f"[DEBUG] {tid}: Failed to compute recovery metrics")
                    else:
                        print(f"[DEBUG] {tid}: Insufficient recovery samples")
                else:
                    print(f"[DEBUG] {tid}: No exceedances found in segment")
            else:
                print(f"[DEBUG] {tid}: No valid segment mask found")
        else:
            print(f"[DEBUG] {tid}: Skipping Level 2 check - missing required columns (t_s, lat_deg, lon_deg)")

        grades[tid] = "Unstable"
        print(f"[DEBUG] {tid}: Final grade: Unstable")

    # Add missing ones as Not launched
    for tid in id_to_cfg.keys():
        if tid not in grades:
            grades[tid] = "Not launched"

    return grades, metrics


def plot_per_test(fig: mpl.figure.Figure, df: pd.DataFrame, display_name: str, palette: List[str]) -> None:
    fig.suptitle(f"Test: {display_name}")
    axs = fig.subplots(2, 2, squeeze=True)
    ax1, ax2 = axs[0, 0], axs[0, 1]
    ax3, ax4 = axs[1, 0], axs[1, 1]

    # 1) Attitude vs time
    t = df.get("t_s", pd.Series(dtype=float))
    for name, color in zip(["roll_deg", "pitch_deg", "yaw_deg"], palette[:3]):
        if name in df.columns:
            sns.lineplot(ax=ax1, x=t, y=df[name], label=name.replace("_deg", " (deg)"), linewidth=1.2, color=color)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Angle (deg)")
    ax1.legend(loc="best", frameon=True)
    ax1.grid(True, alpha=0.3)

    # 2) XY trajectory (meters)
    if "lat_deg" in df.columns and "lon_deg" in df.columns:
        x, y = latlon_to_xy(df)
        if x.size > 0:
            sns.lineplot(ax=ax2, x=x, y=y, linewidth=1.2, color=palette[0])
            ax2.scatter([x[0]], [y[0]], s=12, color="green", label="start")
            ax2.scatter([x[-1]], [y[-1]], s=12, color="red", label="end")
            ax2.set_aspect("equal", adjustable="datalim")
    ax2.set_xlabel("X East (m)")
    ax2.set_ylabel("Y North (m)")
    ax2.legend(loc="best", frameon=True)
    ax2.grid(True, alpha=0.3)

    # 3) Altitude vs time
    if "rel_alt_m" in df.columns:
        sns.lineplot(ax=ax3, x=t, y=df["rel_alt_m"], linewidth=1.2, color=palette[1])
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Relative Altitude (m)")
    ax3.grid(True, alpha=0.3)

    # 4) Wind components and magnitude vs time
    wind_cols = ["wind_x_m_s", "wind_y_m_s", "wind_z_m_s", "wind_m_s"]
    labels = ["Wind X (m/s)", "Wind Y (m/s)", "Wind Z (m/s)", "|Wind| (m/s)"]
    for col, label, color in zip(wind_cols, labels, palette[:4]):
        if col in df.columns:
            sns.lineplot(ax=ax4, x=t, y=df[col].fillna(0.0), label=label, linewidth=1.2, color=color)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Wind (m/s)")
    ax4.legend(loc="best", frameon=True)
    ax4.grid(True, alpha=0.3)


def plot_summary(fig: mpl.figure.Figure, dfs: Dict[str, pd.DataFrame], labels: Dict[str, str], palette: List[str]) -> None:
    fig.suptitle("Summary: Trajectories and Altitudes")
    ax_xy, ax_alt = fig.subplots(1, 2, squeeze=True)

    # Overlaid XY for each test
    for i, (test_id, df) in enumerate(dfs.items()):
        if {"lat_deg", "lon_deg"}.issubset(df.columns):
            x, y = latlon_to_xy(df)
            if x.size > 0:
                label = labels.get(test_id, test_id)
                sns.lineplot(ax=ax_xy, x=x, y=y, label=label, linewidth=1.2, color=palette[i % len(palette)])
    ax_xy.set_title("Horizontal Trajectories")
    ax_xy.set_xlabel("X East (m)")
    ax_xy.set_ylabel("Y North (m)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(loc="best", frameon=True, fontsize=8)
    ax_xy.grid(True, alpha=0.3)

    # Overlaid altitude vs time
    for i, (test_id, df) in enumerate(dfs.items()):
        if "rel_alt_m" in df.columns:
            label = labels.get(test_id, test_id)
            sns.lineplot(ax=ax_alt, x=df.get("t_s", pd.Series(dtype=float)), y=df["rel_alt_m"],
                         label=label, linewidth=1.2, color=palette[i % len(palette)])
    ax_alt.set_title("Relative Altitude vs Time")
    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Relative Altitude (m)")
    ax_alt.legend(loc="best", frameon=True, fontsize=8)
    ax_alt.grid(True, alpha=0.3)


def main() -> None:
    args = parse_args()
    suite, tests = read_tasks(args.tasks_json)
    results_dir = args.results_dir
    out_pdf = args.output if args.output else (results_dir / "analysis_report.pdf")

    sns.set_theme(context="paper", style="whitegrid", palette="colorblind")
    palette = sns.color_palette("colorblind", 8)

    # Map test_id -> metadata
    id_to_desc: Dict[str, str] = {}
    id_to_cfg: Dict[str, Dict] = {}
    for t in tests:
        tid = t.get("test_id", "")
        if not tid:
            continue
        id_to_desc[tid] = t.get("description", tid)
        id_to_cfg[tid] = t.get("wind_config", {})

    # Load CSVs in task order
    dfs: Dict[str, pd.DataFrame] = {}
    for tid in id_to_desc.keys():
        csv_path = results_dir / f"{tid}.csv"
        if not csv_path.is_file():
            print(f"[warn] CSV not found for {tid}: {csv_path}")
            continue
        df = load_csv(csv_path)
        if "t_s" in df.columns:
            df = df[df["t_s"].notna()]
        dfs[tid] = df

    if not dfs:
        print("No CSVs found. Nothing to do.")
        return

    # Compute stability metrics and grades
    grades, metrics = compute_grades(dfs, id_to_cfg)

    with PdfPages(out_pdf) as pdf:
        # Summary first
        fig = plt.figure(figsize=(11.5, 6.5), dpi=args.dpi)
        plot_summary(fig, dfs, id_to_desc, palette)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # Summary table page: stability grades (highlight out-of-spec values)
        fig = plt.figure(figsize=(11.5, 6.5), dpi=args.dpi)
        ax = fig.add_subplot(111)
        ax.axis('off')
        # Build table data
        header = ["Test", "Grade", "H max dev (m)", "H std (m)", "V max dev (m)", "V std (m)", "Max |roll| (deg)", "Max |pitch| (deg)"]
        rows = []
        for tid in id_to_desc.keys():
            desc = id_to_desc.get(tid, tid)
            g = grades.get(tid, "Not launched")
            m = metrics.get(tid, {})
            rows.append([
                desc,
                g,
                f"{m.get('h_max_dev', float('nan')):.2f}" if 'h_max_dev' in m else "",
                f"{m.get('h_std', float('nan')):.2f}" if 'h_std' in m else "",
                f"{m.get('v_max_dev', float('nan')):.2f}" if 'v_max_dev' in m else "",
                f"{m.get('v_std', float('nan')):.2f}" if 'v_std' in m else "",
                f"{m.get('max_abs_roll', float('nan')):.1f}" if 'max_abs_roll' in m else "",
                f"{m.get('max_abs_pitch', float('nan')):.1f}" if 'max_abs_pitch' in m else "",
            ])
        # Wider first column (Test) to avoid overflow; relative widths
        col_widths = [0.26, 0.10, 0.10, 0.10, 0.10, 0.10, 0.12, 0.12]
        table = ax.table(cellText=rows, colLabels=header, loc='center', colWidths=col_widths)
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.4)

        # Highlight metric cells exceeding thresholds in red
        H_MAX, H_STD, V_MAX, V_STD, ANG_MAX = 1.5, 0.75, 3.0, 1.5, 45.0
        # Column indices for metrics in the table
        COL_HMAX, COL_HSTD, COL_VMAX, COL_VSTD, COL_RMAX, COL_PMAX = 2, 3, 4, 5, 6, 7
        red = (0.85, 0.20, 0.20)
        for r_idx, tid in enumerate(id_to_desc.keys(), start=1):  # +1 for header row
            m = metrics.get(tid, {})
            # Helper to parse cell value back to float safely
            def _gt(v, thr):
                try:
                    return float(v) > thr
                except Exception:
                    return False
            cells = table.get_celld()
            # Map each metric col to (value, threshold)
            vals = [
                (m.get('h_max_dev', None), H_MAX, COL_HMAX),
                (m.get('h_std', None),     H_STD, COL_HSTD),
                (m.get('v_max_dev', None), V_MAX, COL_VMAX),
                (m.get('v_std', None),     V_STD, COL_VSTD),
                (m.get('max_abs_roll', None),  ANG_MAX, COL_RMAX),
                (m.get('max_abs_pitch', None), ANG_MAX, COL_PMAX),
            ]
            for val, thr, c in vals:
                if val is None or (isinstance(val, float) and (math.isnan(val) or math.isinf(val))):
                    continue
                if float(val) > thr:
                    cell = cells[(r_idx, c)]
                    cell.get_text().set_color(red)
                    cell.set_edgecolor(red)
                    # Light red background to highlight
                    cell.set_facecolor((1.0, 0.92, 0.92))
        pdf.savefig(fig)
        plt.close(fig)

        # Per-test pages
        for idx, (test_id, df) in enumerate(dfs.items()):
            fig = plt.figure(figsize=(11.5, 7.5), dpi=args.dpi)
            display = id_to_desc.get(test_id, test_id)
            plot_per_test(fig, df, display, palette)
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)

    print(f"Wrote analysis PDF: {out_pdf}")


if __name__ == "__main__":
    main()
