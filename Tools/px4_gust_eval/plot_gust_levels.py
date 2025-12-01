#!/usr/bin/env python3
"""
Wind Gust Level Performance Plotter

Generates bar charts showing stability metrics across different gust levels.
Automatically detects task types (e.g., different wind directions) and creates
separate plots for each type.

Usage:
  uv run Tools/px4_gust_eval/plot_gust_levels.py \
    Tools/px4_gust_eval/tasks/beaufort_levels_tests.json

  # With explicit results dir + online logging
  uv run --with wandb Tools/px4_gust_eval/plot_gust_levels.py \
    Tools/px4_gust_eval/tasks/beaufort_levels_tests.json \
    --results-dir Tools/px4_gust_eval/logs/levels/run_20250929_131736 \
    --wandb --wandb-entity MAALab --wandb-project px4_gust_eval --upload-data
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Tuple, Optional, Any

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

# Recovery window for Wind-Recoverable
RECOVER_T = 10.0  # seconds

# Grade colors
COLORS = {
    "Wind-Resilient": "#2ecc71",      # Green
    "Wind-Recoverable": "#f39c12",      # Orange
    "Unstable": "#e74c3c",     # Red
    "Not launched": "#95a5a6"  # Gray
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot wind gust level performance metrics")
    p.add_argument("tasks_json", type=Path, help="Path to tasks JSON")
    p.add_argument("--results-dir", type=Path, default=None,
                   help="Directory containing CSV results; defaults to latest run_* under the configured logs directory")
    p.add_argument("--dpi", type=int, default=300, help="Figure DPI")
    p.add_argument("--wandb", action="store_true", help="Upload plots (and optionally data) to Weights & Biases")
    p.add_argument("--wandb-project", type=str, default=os.getenv("WANDB_PROJECT", "px4_gust_eval"),
                   help="Weights & Biases project name")
    p.add_argument("--wandb-entity", type=str, default=os.getenv("WANDB_ENTITY"),
                   help="Weights & Biases entity / team")
    p.add_argument("--wandb-run-id", type=str, default=os.getenv("WANDB_RUN_ID"),
                   help="Reuse an existing W&B run id (resume=allow)")
    p.add_argument("--wandb-run-name", type=str, default=None, help="Custom run name")
    p.add_argument("--wandb-tags", nargs="*", default=None, help="Optional W&B tags")
    p.add_argument("--upload-data", action="store_true",
                   help="Also upload CSV/log files from the results directory as a W&B artifact")
    return p.parse_args()


def load_tasks(tasks_path: Path) -> Dict:
    """Load tasks JSON with basic validation."""
    try:
        return json.loads(tasks_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SystemExit(f"Failed to read tasks file {tasks_path}: {exc}") from exc


def read_tasks(tasks_path: Path, data: Dict) -> Tuple[str, List[Dict]]:
    """Extract suite metadata and tests from loaded JSON data."""
    suite = data.get("test_suite", tasks_path.stem)
    tests = data.get("wind_gust_tests", data.get("tests", []))
    return suite, tests


def _resolve_logs_base(tasks_path: Path, data: Dict) -> Path:
    """Return the base logs directory for the given tasks file."""
    log_dir = data.get("output_config", {}).get("log_directory")
    default_dir = Path("Tools/px4_gust_eval/logs")
    base = Path(log_dir) if log_dir else default_dir

    if base.is_absolute():
        return base

    # Prefer current working directory (matches runner behavior), then tasks dir
    cwd_candidate = (Path.cwd() / base).resolve()
    tasks_candidate = (tasks_path.parent / base).resolve()

    if cwd_candidate.exists():
        return cwd_candidate
    if tasks_candidate.exists():
        return tasks_candidate

    # Fall back to CWD-based path even if it doesn't exist yet
    return cwd_candidate


def _latest_run_dir(logs_base: Path) -> Optional[Path]:
    """Return the most recent run directory inside the given logs base."""
    if not logs_base.exists():
        return None

    if logs_base.is_dir() and any(logs_base.glob("*.csv")):
        return logs_base

    run_dirs = [p for p in logs_base.iterdir() if p.is_dir()]
    if not run_dirs:
        return None

    run_dirs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    for candidate in run_dirs:
        if any(candidate.glob("*.csv")):
            return candidate

    return run_dirs[0] if run_dirs else None


def resolve_results_dir(args: argparse.Namespace, tasks_path: Path, data: Dict) -> Path:
    """Determine which results directory to use."""
    if args.results_dir:
        res_input = args.results_dir
        candidates = []
        if res_input.is_absolute():
            candidates.append(res_input)
        else:
            candidates.append((Path.cwd() / res_input).resolve())
            candidates.append((tasks_path.parent / res_input).resolve())

        for res in candidates:
            if res.is_dir():
                return res

        raise SystemExit(
            f"Provided results directory does not exist: {args.results_dir} "
            f"(checked: {', '.join(str(c) for c in candidates)})"
        )

    logs_base = _resolve_logs_base(tasks_path, data)
    latest = _latest_run_dir(logs_base)
    if latest and latest.is_dir():
        print(f"[auto] Using latest results directory: {latest}")
        return latest

    raise SystemExit(
        f"Could not find a results directory. Looked under {logs_base} "
        "(expecting run_* subdirectories containing CSV files). "
        "Specify one with --results-dir."
    )


def describe_results_dir(results_dir: Path) -> None:
    """Print a short summary of files in the results directory."""
    print(f"[info] Results directory: {results_dir}")
    if not results_dir.exists():
        print("  (does not exist)")
        return

    files = sorted(results_dir.iterdir())
    if not files:
        print("  (empty)")
        return

    for f in files:
        if f.is_file():
            print(f"  file: {f.name}")
        elif f.is_dir():
            print(f"  dir : {f.name}/")


def start_wandb_if_requested(args: argparse.Namespace, suite: str, results_dir: Path, existing_run=None):
    """Initialize Weights & Biases run when --wandb is set, or reuse an existing run."""
    if existing_run is not None:
        return existing_run
    if not args.wandb:
        return None

    try:
        import wandb  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "wandb is required for --wandb. Install it with `uv pip install wandb` "
            "or run via `uv run --with wandb ...`."
        ) from exc

    run_name = args.wandb_run_name or f"gust-levels-{results_dir.name}"
    tags = args.wandb_tags or []
    run = wandb.init(
        project=args.wandb_project,
        entity=args.wandb_entity,
        name=run_name,
        tags=tags,
        config={
            "suite": suite,
            "results_dir": str(results_dir),
            "tasks_file": str(args.tasks_json),
        },
        id=args.wandb_run_id,
        resume="allow" if args.wandb_run_id else None,
        settings=wandb.Settings(init_timeout=180),
    )
    return run


def log_plots_to_wandb(run, images: List[Tuple[str, Path]]) -> None:
    """Upload plot images to a W&B run."""
    if not run or not images:
        return

    try:
        import wandb  # type: ignore
    except ImportError:
        return

    for key, path in images:
        run.log({key: wandb.Image(str(path))})


def upload_data_artifact(run, results_dir: Path, suite: str) -> None:
    """Upload CSV/log files from the results dir as an artifact."""
    if not run:
        return

    try:
        import wandb  # type: ignore
    except ImportError:
        return

    data_files = sorted(
        p for p in results_dir.iterdir()
        if p.is_file() and p.suffix.lower() in {".csv", ".log", ".json"}
    )
    if not data_files:
        print("No CSV/log files found to upload as an artifact.")
        return

    safe_suite = re.sub(r"[^A-Za-z0-9_.-]+", "-", suite).strip("-") or "suite"
    artifact_name = f"gust-levels-data-{safe_suite}-{results_dir.name}"
    artifact = wandb.Artifact(artifact_name, type="gust-level-logs")
    for f in data_files:
        artifact.add_file(str(f), name=f.name)

    run.log_artifact(artifact)
    print(f"Uploaded {len(data_files)} data file(s) to W&B artifact '{artifact_name}':")
    for f in data_files:
        print(f"  - {f.name}")


def _clean_value(val: Any) -> Any:
    """Convert NaN/inf to None for JSON friendliness."""
    try:
        if isinstance(val, float) and not math.isfinite(val):
            return None
    except Exception:
        pass
    return val



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


def compute_grade_dimensional(df: pd.DataFrame, dim: str) -> str:
    """Compute stability grade for a single test, per dimension.

    dim:
      - 'h' for horizontal metrics (uses H_MAX/H_STD)
      - 'v' for vertical metrics (uses V_MAX/V_STD)

    Notes:
      - Angle constraints are not considered for per-dimension grading.
      - Recovery check is evaluated only on the same dimension's exceedance.
    """
    if df.empty or not {"lat_deg", "lon_deg"}.issubset(df.columns):
        return "Not launched"

    if dim == 'v' and "rel_alt_m" not in df.columns:
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
    if "t_s" in df.columns and {"lat_deg", "lon_deg"}.issubset(df.columns):
        x, y = latlon_to_xy(df)
        t = df["t_s"].to_numpy(dtype=float)
        mask = _segment_mask_from_x(x)

        if mask.any():
            # Build exceedance flag for the selected dimension
            exceed_dim = np.zeros_like(mask, dtype=bool)
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
               label=f'Wind-Recoverable Threshold ({threshold} m)', zorder=10)

    # Labels and title
    ax.set_xlabel('Gust Level', fontsize=24, fontweight='bold')
    ax.set_ylabel(f'{metric_name} (m)', fontsize=24, fontweight='bold')
    # ax.set_title(f'{metric_name} vs Gust Level\n{task_type}', fontsize=12, fontweight='bold', pad=15)

    # Grid
    ax.grid(True, axis='y', alpha=0.3, linestyle=':', linewidth=0.8)
    ax.set_axisbelow(True)

    # Legend for grades
    legend_elements = [
        mpatches.Patch(facecolor=COLORS["Wind-Resilient"], edgecolor='black', label='Wind-Resilient'),
        mpatches.Patch(facecolor=COLORS["Wind-Recoverable"], edgecolor='black', label='Wind-Recoverable'),
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


def plot_levels(
    tasks_path: Path,
    results_dir: Optional[Path] = None,
    dpi: int = 300,
    wandb_run=None,
    wandb_project: str = "px4_gust_eval",
    wandb_entity: Optional[str] = None,
    wandb_run_name: Optional[str] = None,
    wandb_tags: Optional[List[str]] = None,
    upload_data: bool = False,
    wandb_enabled: bool = False,
) -> None:
    """Generate plots (and optionally log to an existing W&B run)."""
    args = SimpleNamespace(
        tasks_json=tasks_path,
        results_dir=results_dir,
        dpi=dpi,
        wandb=wandb_enabled,
        wandb_project=wandb_project,
        wandb_entity=wandb_entity,
        wandb_run_id=getattr(wandb_run, "id", None),
        wandb_run_name=wandb_run_name,
        wandb_tags=wandb_tags,
        upload_data=upload_data,
    )

    tasks_data = load_tasks(tasks_path)
    suite, tests = read_tasks(tasks_path, tasks_data)
    results_dir_resolved = resolve_results_dir(args, tasks_path, tasks_data)
    describe_results_dir(results_dir_resolved)

    # Group tests by type
    test_groups = group_tests_by_type(tests)

    print(f"Found {len(test_groups)} task type(s):")
    for task_type, group in test_groups.items():
        print(f"  - {task_type}: {len(group)} tests")

    wandb_run_local = start_wandb_if_requested(args, suite, results_dir_resolved, existing_run=wandb_run)
    images_to_upload: List[Tuple[str, Path]] = []
    summary_rows: List[Dict[str, Any]] = []

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
            csv_path = results_dir_resolved / f"{test_id}.csv"
            if not csv_path.is_file():
                print(f"  Warning: CSV not found for {test_id}")
                continue

            df = load_csv(csv_path)
            if "t_s" in df.columns:
                df = df[df["t_s"].notna()]

            # Compute metrics and per-dimension grades
            metrics = compute_metrics_for_test(df)
            if metrics is None:
                print(f"  Warning: Could not compute metrics for {test_id}")
                continue

            grade_h = compute_grade_dimensional(df, 'h')
            grade_v = compute_grade_dimensional(df, 'v')

            levels.append(level)
            v_max_devs.append(metrics.get("v_max_dev", float("nan")))
            h_max_devs.append(metrics.get("h_max_dev", float("nan")))
            grades_v.append(grade_v)
            grades_h.append(grade_h)
            summary_rows.append({
                "task_type": task_type,
                "test_id": test_id,
                "level": level,
                "v_max_dev": _clean_value(metrics.get("v_max_dev", float("nan"))),
                "h_max_dev": _clean_value(metrics.get("h_max_dev", float("nan"))),
                "grade_v": grade_v,
                "grade_h": grade_h,
            })

        if not levels:
            print(f"  No valid data for {task_type}, skipping...")
            continue

        # Generate safe filename
        safe_type = task_type.replace(" ", "_").replace("(", "").replace(")", "").lower()

        # Plot V max dev
        v_output = results_dir_resolved / f"v_max_dev_{safe_type}.png"
        plot_metric_bar_chart(
            levels, v_max_devs, grades_v,
            V_MAX, "Vertical Max Deviation", task_type,
            v_output, dpi
        )
        images_to_upload.append((f"plots/{v_output.name}", v_output))

        # Plot H max dev
        h_output = results_dir_resolved / f"h_max_dev_{safe_type}.png"
        plot_metric_bar_chart(
            levels, h_max_devs, grades_h,
            H_MAX, "Horizontal Max Deviation", task_type,
            h_output, dpi
        )
        images_to_upload.append((f"plots/{h_output.name}", h_output))

    if summary_rows:
        summary_path = results_dir_resolved / "gust_levels_summary.json"
        try:
            with open(summary_path, "w", encoding="utf-8") as f:
                json.dump(summary_rows, f, indent=2)
            print(f"Saved summary JSON: {summary_path}")
        except Exception as e:
            print(f"Warning: failed to write summary JSON: {e}")

        if wandb_run_local:
            try:
                import wandb  # type: ignore
                import pandas as pd  # type: ignore
                df_summary = pd.DataFrame(summary_rows)
                wandb_run_local.log({"gust_summary/table": wandb.Table(dataframe=df_summary)})
            except Exception as e:
                print(f"Warning: failed to log summary table to W&B: {e}")

    if wandb_run_local:
        log_plots_to_wandb(wandb_run_local, images_to_upload)
        if upload_data:
            upload_data_artifact(wandb_run_local, results_dir_resolved, suite)
        # Do not finish run here; caller may own lifecycle

    print("\nAll plots generated successfully!")


def main() -> None:
    args = parse_args()
    plot_levels(
        tasks_path=args.tasks_json,
        results_dir=args.results_dir,
        dpi=args.dpi,
        wandb_run=None,
        wandb_project=args.wandb_project,
        wandb_entity=args.wandb_entity,
        wandb_run_name=args.wandb_run_name,
        wandb_tags=args.wandb_tags,
        upload_data=args.upload_data,
        wandb_enabled=args.wandb,
    )


if __name__ == "__main__":
    main()
