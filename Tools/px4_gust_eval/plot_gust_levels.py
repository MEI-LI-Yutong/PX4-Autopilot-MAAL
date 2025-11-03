#!/usr/bin/env python3
"""
Wind Gust Level Performance Plotter

Generates bar charts showing stability metrics across different gust levels.
Automatically detects task types (e.g., different wind directions) and creates
separate plots for each type.

Usage:
  uv run Tools/px4_gust_eval/plot_gust_levels.py \
    Tools/px4_gust_eval/tasks/beaufort_levels_tests.json \
    Tools/px4_gust_eval/logs/levels/run_20250929_131736
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from collections import defaultdict

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

try:
    import scienceplots  # noqa: F401
    plt.style.use(["science", "no-latex"])
except Exception:
    pass


# Stability thresholds
H_MAX = 1.5      # m - horizontal max deviation
H_STD = 0.75     # m - horizontal std deviation
V_MAX = 3.0      # m - vertical max deviation
V_STD = 1.5      # m - vertical std deviation
ANG_MAX = 45.0   # deg - max roll/pitch

# Recovery window for Level 2
RECOVER_T = 10.0  # seconds

# Grade colors
COLORS = {
    "Level 1": "#2ecc71",      # Green
    "Level 2": "#f39c12",      # Orange
    "Unstable": "#e74c3c",     # Red
    "Not launched": "#95a5a6"  # Gray
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot wind gust level performance metrics")
    p.add_argument("tasks_json", type=Path, help="Path to tasks JSON")
    p.add_argument("results_dir", type=Path, help="Directory containing CSV results")
    p.add_argument("--dpi", type=int, default=300, help="Figure DPI")
    return p.parse_args()


def read_tasks(tasks_path: Path) -> Tuple[str, List[Dict]]:
    """Load tasks from JSON file."""
    data = json.loads(tasks_path.read_text(encoding="utf-8"))
    suite = data.get("test_suite", tasks_path.stem)
    tests = data.get("wind_gust_tests", data.get("tests", []))
    return suite, tests


def load_csv(csv_path: Path) -> pd.DataFrame:
    """Load and preprocess CSV data."""
    df = pd.read_csv(csv_path)
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
    """Convert lat/lon to local XY in meters."""
    import math
    lat = df["lat_deg"].to_numpy(dtype=float)
    lon = df["lon_deg"].to_numpy(dtype=float)
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


def compute_metrics_for_test(df: pd.DataFrame) -> Optional[Dict[str, float]]:
    """Compute stability metrics on mid-flight segment."""
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
        z_ref = 10.0
        v_dev = z_seg - z_ref
        v_max = float(np.nanmax(np.abs(v_dev))) if z_seg.size else float("nan")
        v_std = float(np.nanstd(v_dev)) if z_seg.size else float("nan")
    else:
        v_max = float("nan")
        v_std = float("nan")

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


def compute_grade(df: pd.DataFrame) -> str:
    """Compute stability grade for a single test."""
    m = compute_metrics_for_test(df)
    if m is None:
        return "Not launched"

    base_ok = (
        (m.get("h_max_dev", float("inf")) <= H_MAX) and
        (m.get("h_std", float("inf")) <= H_STD) and
        (m.get("v_max_dev", float("inf")) <= V_MAX) and
        (m.get("v_std", float("inf")) <= V_STD) and
        (m.get("max_abs_roll", float("inf")) <= ANG_MAX) and
        (m.get("max_abs_pitch", float("inf")) <= ANG_MAX)
    )

    if base_ok:
        return "Level 1"

    # Check recovery for Level 2
    if "t_s" in df.columns and {"lat_deg", "lon_deg"}.issubset(df.columns):
        x, y = latlon_to_xy(df)
        t = df["t_s"].to_numpy(dtype=float)
        mask = _segment_mask_from_x(x)

        if mask.any():
            # Build exceedance flags
            exceed_h = np.zeros_like(mask, dtype=bool)
            try:
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
                exceed_h = np.abs(y_dev_full) > H_MAX
            except Exception:
                pass

            exceed_v = np.zeros_like(mask, dtype=bool)
            if "rel_alt_m" in df.columns:
                try:
                    z = df["rel_alt_m"].to_numpy(dtype=float)
                    z_ref = 10.0
                    v_dev_full = z - z_ref
                    exceed_v = np.abs(v_dev_full) > V_MAX
                except Exception:
                    pass

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

            if exceed_any.any():
                i0 = int(np.argmax(exceed_any))
                t0 = float(t[i0])
                mask_recover = mask & (t >= t0) & (t <= (t0 + RECOVER_T))

                if mask_recover.sum() >= 5:
                    df_after = df[mask_recover]
                    m_after = compute_metrics_for_test(df_after)
                    if m_after is not None:
                        ok_after = (
                            (m_after.get("h_max_dev", float("inf")) <= H_MAX) and
                            (m_after.get("h_std", float("inf")) <= H_STD) and
                            (m_after.get("v_max_dev", float("inf")) <= V_MAX) and
                            (m_after.get("v_std", float("inf")) <= V_STD) and
                            (m_after.get("max_abs_roll", float("inf")) <= ANG_MAX) and
                            (m_after.get("max_abs_pitch", float("inf")) <= ANG_MAX)
                        )
                        if ok_after:
                            return "Level 2"

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


def detect_task_type(test_config: Dict) -> str:
    """Detect task type from test_id pattern."""
    test_id = test_config.get("test_id", "")

    # Use test_id pattern to detect type
    if "_z_" in test_id:
        return "Vertical (Z)"
    elif "_y_" in test_id:
        return "Horizontal (Y)"
    elif "gust_lvl_" in test_id:
        return "Horizontal (X)"

    return "Unknown"


def group_tests_by_type(tests: List[Dict]) -> Dict[str, List[Dict]]:
    """Group tests by their type (wind direction)."""
    groups = defaultdict(list)
    for test in tests:
        task_type = detect_task_type(test)
        groups[task_type].append(test)
    return dict(groups)


def plot_metric_bar_chart(
    levels: List[int],
    values: List[float],
    grades: List[str],
    threshold: float,
    metric_name: str,
    task_type: str,
    output_path: Path,
    dpi: int = 300
) -> None:
    """Create bar chart for a specific metric across gust levels."""
    fig, ax = plt.subplots(figsize=(8, 5))

    # Sort by level
    sorted_data = sorted(zip(levels, values, grades), key=lambda x: x[0])
    levels_sorted, values_sorted, grades_sorted = zip(*sorted_data) if sorted_data else ([], [], [])

    # Create bars with colors based on grade
    colors = [COLORS.get(g, COLORS["Not launched"]) for g in grades_sorted]
    bars = ax.bar(levels_sorted, values_sorted, color=colors, edgecolor='black', linewidth=0.8, alpha=0.85)

    # Add threshold lines
    ax.axhline(y=threshold, color='#e74c3c', linestyle='--', linewidth=1.5,
               label=f'Level 2 Threshold ({threshold} m)', zorder=10)

    # Labels and title
    ax.set_xlabel('Gust Level', fontsize=24, fontweight='bold')
    ax.set_ylabel(f'{metric_name} (m)', fontsize=24, fontweight='bold')
    # ax.set_title(f'{metric_name} vs Gust Level\n{task_type}', fontsize=12, fontweight='bold', pad=15)

    # Grid
    ax.grid(True, axis='y', alpha=0.3, linestyle=':', linewidth=0.8)
    ax.set_axisbelow(True)

    # Legend for grades
    legend_elements = [
        mpatches.Patch(facecolor=COLORS["Level 1"], edgecolor='black', label='Level 1'),
        mpatches.Patch(facecolor=COLORS["Level 2"], edgecolor='black', label='Level 2'),
        mpatches.Patch(facecolor=COLORS["Unstable"], edgecolor='black', label='Unstable'),
        plt.Line2D([0], [0], color='#e74c3c', linestyle='--', linewidth=1.5, label=f'Threshold ({threshold} m)')
    ]
    ax.legend(handles=legend_elements, loc='upper left', frameon=True, fontsize=24)

    # Set x-axis to show all levels
    if levels_sorted:
        ax.set_xticks(list(levels_sorted))
        ax.set_xlim(min(levels_sorted) - 0.5, max(levels_sorted) + 0.5)

    # Tight layout
    fig.tight_layout()

    # Save
    fig.savefig(output_path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")


def main() -> None:
    args = parse_args()
    suite, tests = read_tasks(args.tasks_json)
    results_dir = args.results_dir

    # Group tests by type
    test_groups = group_tests_by_type(tests)

    print(f"Found {len(test_groups)} task type(s):")
    for task_type, group in test_groups.items():
        print(f"  - {task_type}: {len(group)} tests")

    # Process each group
    for task_type, group_tests in test_groups.items():
        print(f"\nProcessing {task_type}...")

        # Collect data
        levels = []
        v_max_devs = []
        h_max_devs = []
        grades_v = []
        grades_h = []

        for test in group_tests:
            test_id = test.get("test_id", "")
            description = test.get("description", "")

            # Extract level
            level = extract_gust_level(test_id, description)
            if level is None:
                print(f"  Warning: Could not extract level from {test_id}")
                continue

            # Load CSV
            csv_path = results_dir / f"{test_id}.csv"
            if not csv_path.is_file():
                print(f"  Warning: CSV not found for {test_id}")
                continue

            df = load_csv(csv_path)
            if "t_s" in df.columns:
                df = df[df["t_s"].notna()]

            # Compute metrics and grade
            metrics = compute_metrics_for_test(df)
            if metrics is None:
                print(f"  Warning: Could not compute metrics for {test_id}")
                continue

            grade = compute_grade(df)

            levels.append(level)
            v_max_devs.append(metrics.get("v_max_dev", float("nan")))
            h_max_devs.append(metrics.get("h_max_dev", float("nan")))
            grades_v.append(grade)
            grades_h.append(grade)

        if not levels:
            print(f"  No valid data for {task_type}, skipping...")
            continue

        # Generate safe filename
        safe_type = task_type.replace(" ", "_").replace("(", "").replace(")", "").lower()

        # Plot V max dev
        v_output = results_dir / f"v_max_dev_{safe_type}.png"
        plot_metric_bar_chart(
            levels, v_max_devs, grades_v,
            V_MAX, "Vertical Max Deviation", task_type,
            v_output, args.dpi
        )

        # Plot H max dev
        h_output = results_dir / f"h_max_dev_{safe_type}.png"
        plot_metric_bar_chart(
            levels, h_max_devs, grades_h,
            H_MAX, "Horizontal Max Deviation", task_type,
            h_output, args.dpi
        )

    print("\nAll plots generated successfully!")


if __name__ == "__main__":
    main()
