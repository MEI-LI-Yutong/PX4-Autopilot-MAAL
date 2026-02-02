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
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Tuple, Optional, Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from utils.gust_metrics import (
    H_MAX,
    V_MAX,
    compute_analysis_window,
    compute_metrics_for_test,
    compute_grade_dimensional,
    extract_gust_level,
    latlon_to_xy,
)
from utils.gust_grouping import (
    group_tests_by_type,
    wind_axis_from_task_type,
)
from utils.gust_plotting import (
    load_csv,
    log_plots_to_wandb,
    plot_metric_bar_chart,
    plot_radar_chart,
    upload_data_artifact,
)
from utils.gust_dimensions import (
    DIM_LABELS,
    DIM_ORDER,
    compute_dimension_breakdown,
)

try:
    import scienceplots  # noqa: F401
    plt.style.use(["science", "no-latex"])
except Exception:
    pass


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


def _clean_value(val: Any) -> Any:
    """Convert NaN/inf to None for JSON friendliness."""
    try:
        if isinstance(val, float) and not math.isfinite(val):
            return None
    except Exception:
        pass
    return val



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
        dim_scores_list = []
        dim_scores_by_level: Dict[int, List[Dict[str, float]]] = {}
        dim_raw_by_level: Dict[int, List[Dict[str, float]]] = {}
        debug_series = []

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
            breakdown = compute_dimension_breakdown(df)
            dim_scores = breakdown.get("scores", {})
            dim_raw = breakdown.get("raw", {})
            overall_score = breakdown.get("overall", float("nan"))

            levels.append(level)
            v_max_devs.append(metrics.get("v_max_dev", float("nan")))
            h_max_devs.append(metrics.get("h_max_dev", float("nan")))
            grades_v.append(grade_v)
            grades_h.append(grade_h)
            dim_scores_list.append(dim_scores)
            dim_scores_by_level.setdefault(level, []).append(dim_scores)
            dim_raw_by_level.setdefault(level, []).append(dim_raw)
            summary_rows.append({
                "task_type": task_type,
                "test_id": test_id,
                "level": level,
                "v_max_dev": _clean_value(metrics.get("v_max_dev", float("nan"))),
                "h_max_dev": _clean_value(metrics.get("h_max_dev", float("nan"))),
                "grade_v": grade_v,
                "grade_h": grade_h,
                "dimension_scores": dim_scores,
                "dimension_raw": dim_raw,
                "dimension_overall": _clean_value(overall_score),
            })

            if level != 0 and "t_s" in df.columns and "rel_alt_m" in df.columns:
                t_s = df["t_s"].to_numpy(dtype=float)
                rel_alt = df["rel_alt_m"].to_numpy(dtype=float)
                x_pos, y_pos = latlon_to_xy(df)
                horiz = y_pos if y_pos.size == t_s.size else None
                window = compute_analysis_window(df)
                if horiz is not None:
                    debug_series.append({
                        "level": level,
                        "t_s": t_s,
                        "rel_alt": rel_alt,
                        "horiz": horiz,
                        "window": window,
                    })

        if not levels:
            print(f"  No valid data for {task_type}, skipping...")
            continue

        axis = wind_axis_from_task_type(task_type)

        # Plot V max dev
        v_output = results_dir_resolved / f"vertical_max_{axis}.png"
        plot_metric_bar_chart(
            levels, v_max_devs, grades_v,
            V_MAX, "Vertical Max Deviation", task_type,
            v_output, dpi
        )
        images_to_upload.append((f"plots/{v_output.name}", v_output))

        # Plot H max dev
        h_output = results_dir_resolved / f"horizontal_max_{axis}.png"
        plot_metric_bar_chart(
            levels, h_max_devs, grades_h,
            H_MAX, "Horizontal Max Deviation", task_type,
            h_output, dpi
        )
        images_to_upload.append((f"plots/{h_output.name}", h_output))

        # Radar summary (min + mean)
        if dim_scores_list:
            min_scores = {}
            mean_scores = {}
            for k in DIM_ORDER:
                vals = [s.get(k, float("nan")) for s in dim_scores_list]
                vals = [v for v in vals if v == v]
                min_scores[k] = min(vals) if vals else float("nan")
                mean_scores[k] = float(sum(vals) / len(vals)) if vals else float("nan")
            radar_output = results_dir_resolved / f"radar_{axis}_summary.png"
            plot_radar_chart(
                [min_scores, mean_scores],
                ["worst", "mean"],
                radar_output,
                dpi,
                dims=DIM_ORDER,
                dim_labels=DIM_LABELS,
            )
            images_to_upload.append((f"plots/{radar_output.name}", radar_output))

        if dim_scores_by_level:
            target_levels = [0, 2, 4, 6, 8, 10, 12]
            levels_sorted = [lvl for lvl in target_levels if lvl in dim_scores_by_level]
            if not levels_sorted:
                levels_sorted = sorted(dim_scores_by_level.keys())

            def _mean_scores(bucket: List[Dict[str, float]]) -> Dict[str, float]:
                scores_mean = {}
                for k in DIM_ORDER:
                    vals = [s.get(k, float("nan")) for s in bucket]
                    vals = [v for v in vals if v == v]
                    scores_mean[k] = float(sum(vals) / len(vals)) if vals else float("nan")
                return scores_mean

            def _mean_raw(bucket: List[Dict[str, float]]) -> Dict[str, float]:
                if not bucket:
                    return {}
                keys = set()
                for b in bucket:
                    keys.update(b.keys())
                raw_mean = {}
                for k in keys:
                    vals = [b.get(k, float("nan")) for b in bucket]
                    vals = [v for v in vals if v == v]
                    raw_mean[k] = float(sum(vals) / len(vals)) if vals else float("nan")
                return raw_mean

            level_means: List[Dict[str, float]] = []
            level_labels: List[str] = []
            for lvl in levels_sorted:
                level_means.append(_mean_scores(dim_scores_by_level.get(lvl, [])))
                level_labels.append(f"L{lvl:02d}")

            baseline_scores = None
            baseline_raw = None
            if 0 in dim_scores_by_level:
                baseline_scores = _mean_scores(dim_scores_by_level.get(0, []))
                baseline_raw = _mean_raw(dim_raw_by_level.get(0, []))

            raw_map = {
                "h_max": "h_max_dev_m",
                "h_std": "h_std_m",
                "v_max": "v_max_dev_m",
                "v_std": "v_std_m",
                "att_max": "att_max_deg",
                "act_margin": "actuator_delta",
                "recovery": "recovery_time_s",
                "wind_sense": "wind_err_corr",
            }

            def _normalize(scores: Dict[str, float], raw: Dict[str, float], baseline: Dict[str, float] | None, baseline_raw: Dict[str, float] | None, is_baseline: bool) -> Dict[str, float]:
                if not baseline and not baseline_raw:
                    return scores
                out = {}
                for k in DIM_ORDER:
                    if is_baseline:
                        out[k] = 1.0
                    else:
                        raw_key = raw_map.get(k)
                        b_raw = baseline_raw.get(raw_key, float("nan")) if baseline_raw else float("nan")
                        v_raw = raw.get(raw_key, float("nan"))
                        if b_raw == b_raw and v_raw == v_raw and v_raw > 0:
                            out[k] = max(0.0, min(1.0, b_raw / v_raw))
                        else:
                            b = baseline.get(k, float("nan")) if baseline else float("nan")
                            v = scores.get(k, float("nan"))
                            if b == b and b > 0 and v == v:
                                out[k] = max(0.0, min(1.0, v / b))
                            else:
                                out[k] = v
                return out

            raw_means = [
                _mean_raw(dim_raw_by_level.get(lvl, []))
                for lvl in levels_sorted
            ]
            normalized_levels = [
                _normalize(s, r, baseline_scores, baseline_raw, lvl == 0)
                for s, r, lvl in zip(level_means, raw_means, levels_sorted)
            ]

            cmap = plt.cm.get_cmap("viridis", max(2, len(levels_sorted)))
            colors = [cmap(i) for i in range(len(levels_sorted))]

            radar_levels_output = results_dir_resolved / f"radar_{axis}_levels_baseline.png"
            plot_labels = [f"L{lvl:02d}" for lvl in levels_sorted if lvl != 0]
            plot_scores = [
                s for s, lvl in zip(normalized_levels, levels_sorted)
                if lvl != 0
            ]
            plot_colors = [
                c for c, lvl in zip(colors, levels_sorted)
                if lvl != 0
            ]
            plot_radar_chart(
                plot_scores,
                plot_labels,
                radar_levels_output,
                dpi,
                dims=DIM_ORDER,
                dim_labels=DIM_LABELS,
                colors=plot_colors,
                rmax=1.0,
            )
            images_to_upload.append((f"plots/{radar_levels_output.name}", radar_levels_output))

        if debug_series:
            debug_series.sort(key=lambda s: s["level"])
            cmap = plt.cm.get_cmap("viridis", max(2, len(debug_series)))
            colors = [cmap(i) for i in range(len(debug_series))]
            fig, (ax_alt, ax_h) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
            for idx, series in enumerate(debug_series):
                level = series["level"]
                t_s = series["t_s"]
                rel_alt = series["rel_alt"]
                horiz = series["horiz"]
                color = colors[idx % len(colors)]
                ax_alt.plot(t_s, rel_alt, color=color, linewidth=1.5, label=f"L{level:02d}")
                ax_h.plot(t_s, horiz, color=color, linewidth=1.5, label=f"L{level:02d}")
                if series["window"] is not None:
                    t0, t1 = series["window"]
                    ax_alt.axvspan(t0, t1, color=color, alpha=0.08)
                    ax_h.axvspan(t0, t1, color=color, alpha=0.08)

            ax_alt.set_ylabel("rel_alt_m", fontsize=12, fontweight="bold")
            ax_h.set_ylabel("horizontal_pos_m", fontsize=12, fontweight="bold")
            ax_h.set_xlabel("time (s)", fontsize=12, fontweight="bold")
            ax_alt.grid(True, axis="y", alpha=0.3, linestyle=":", linewidth=0.8)
            ax_h.grid(True, axis="y", alpha=0.3, linestyle=":", linewidth=0.8)
            ax_alt.legend(loc="upper right", frameon=True, fontsize=9, ncol=3)
            fig.tight_layout()
            debug_output = results_dir_resolved / f"debug_{axis}_alt_horiz.png"
            fig.savefig(debug_output, dpi=dpi, bbox_inches="tight")
            plt.close(fig)
            images_to_upload.append((f"plots/{debug_output.name}", debug_output))

            trend_output = results_dir_resolved / f"trend_{axis}_levels_baseline.png"
            fig, ax = plt.subplots(figsize=(9, 5))
            for k, label in zip(DIM_ORDER, DIM_LABELS):
                series = []
                for scores, lvl in zip(normalized_levels, levels_sorted):
                    if lvl == 0:
                        continue
                    series.append(scores.get(k, float("nan")))
                ax.plot(
                    [lvl for lvl in levels_sorted if lvl != 0],
                    series,
                    marker="o",
                    linewidth=2,
                    label=label,
                )
            ax.set_xlabel("Gust Level", fontsize=16, fontweight="bold")
            ax.set_ylabel("Relative Score (vs L0)", fontsize=16, fontweight="bold")
            ax.set_ylim(0.0, 1.05)
            ax.set_xticks([lvl for lvl in levels_sorted if lvl != 0])
            ax.grid(True, axis="y", alpha=0.3, linestyle=":", linewidth=0.8)
            ax.legend(loc="upper right", frameon=True, fontsize=10, ncol=2)
            fig.tight_layout()
            fig.savefig(trend_output, dpi=dpi, bbox_inches="tight")
            plt.close(fig)
            images_to_upload.append((f"plots/{trend_output.name}", trend_output))

    if summary_rows:
        summary_path = results_dir_resolved / "gust_levels_summary.json"
        try:
            with open(summary_path, "w", encoding="utf-8") as f:
                json.dump(summary_rows, f, indent=2)
            print(f"Saved summary JSON: {summary_path}")
        except Exception as e:
            print(f"Warning: failed to write summary JSON: {e}")

        breakdown_csv = results_dir_resolved / "gust_levels_breakdown.csv"
        try:
            import csv
            dim_score_cols = [f"score_{k}" for k in DIM_ORDER]
            raw_cols = [
                "h_max_dev_m",
                "h_std_m",
                "v_max_dev_m",
                "v_std_m",
                "att_max_deg",
                "actuator_peak",
                "actuator_baseline",
                "actuator_delta",
                "recovery_time_s",
                "wind_err_corr",
            ]
            fieldnames = [
                "task_type",
                "test_id",
                "level",
                "dimension_overall",
            ] + dim_score_cols + raw_cols
            with open(breakdown_csv, "w", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in summary_rows:
                    scores = row.get("dimension_scores", {}) or {}
                    raw = row.get("dimension_raw", {}) or {}
                    out = {
                        "task_type": row.get("task_type"),
                        "test_id": row.get("test_id"),
                        "level": row.get("level"),
                        "dimension_overall": row.get("dimension_overall"),
                    }
                    for k in DIM_ORDER:
                        out[f"score_{k}"] = scores.get(k)
                    for k in raw_cols:
                        out[k] = raw.get(k)
                    writer.writerow(out)
            print(f"Saved breakdown CSV: {breakdown_csv}")
        except Exception as e:
            print(f"Warning: failed to write breakdown CSV: {e}")

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
