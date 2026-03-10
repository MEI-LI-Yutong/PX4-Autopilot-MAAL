from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List, Tuple

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

COLORS = {
    "Wind-Resilient": "#2ecc71",      # Green
    "Wind-Recoverable": "#f39c12",      # Orange
    "Unstable": "#e74c3c",     # Red
    "Not launched": "#95a5a6"  # Gray
}


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
    ax.bar(levels_sorted, values_sorted, color=colors, edgecolor='black', linewidth=0.8, alpha=0.85)

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

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)


def plot_radar_chart(
    series: List[Dict[str, float]],
    labels: List[str],
    output_path: Path,
    dpi: int = 300,
    dims: List[str] | None = None,
    dim_labels: List[str] | None = None,
    colors: List[str] | None = None,
    rmax: float = 1.0,
) -> None:
    if dims is None:
        dims = ["track_h", "track_v", "attitude", "actuator", "recovery", "wind_sense"]
    if dim_labels is None:
        dim_labels = dims
    angles = np.linspace(0, 2 * np.pi, len(dims), endpoint=False).tolist()
    angles += angles[:1]

    fig = plt.figure(figsize=(8.4, 8.4))
    ax = plt.subplot(111, polar=True)

    if colors is None:
        colors = [COLORS["Unstable"], COLORS["Wind-Resilient"]]
    for idx, scores in enumerate(series):
        values = [scores.get(k, float("nan")) for k in dims]
        values = [0.0 if not np.isfinite(v) else float(v) for v in values]
        values += values[:1]
        label = labels[idx] if idx < len(labels) else f"series_{idx}"
        ax.plot(angles, values, linewidth=2, color=colors[idx % len(colors)], label=label)
        ax.fill(angles, values, alpha=0.2, color=colors[idx % len(colors)])

    ax.set_thetagrids(np.degrees(angles[:-1]), dim_labels, fontsize=12, fontweight="bold")
    ax.tick_params(axis="x", pad=20)
    for lbl in ax.get_xticklabels():
        lbl.set_ha("center")
        lbl.set_va("center")
    ax.set_ylim(0, rmax)
    ax.set_yticks([rmax * 0.25, rmax * 0.5, rmax * 0.75, rmax])
    ax.tick_params(axis="y", labelsize=11)
    ax.grid(True, alpha=0.3)
    if labels:
        ax.legend(loc="upper right", bbox_to_anchor=(1.22, 1.12), frameon=True, fontsize=11)

    fig.tight_layout(pad=1.8)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def plot_actuator_saturation_summary(
    rows: List[Dict[str, float]],
    output_path: Path,
    dpi: int = 300,
    title: str = "Actuator Saturation Summary",
) -> None:
    if not rows:
        return

    grouped: Dict[str, Dict[str, List[float]]] = {}
    kinds: Dict[str, str] = {}
    for row in rows:
        actuator = str(row.get("actuator"))
        if not actuator:
            continue
        grouped.setdefault(actuator, {"duration": [], "sat_any": []})
        grouped[actuator]["duration"].append(float(row.get("sat_duration_s", 0.0)))
        grouped[actuator]["sat_any"].append(float(row.get("sat_any", 0.0)))
        kinds[actuator] = str(row.get("kind", ""))

    if not grouped:
        return

    def _sort_key(name: str) -> Tuple[int, int, str]:
        kind = kinds.get(name, "")
        order = 0 if kind == "u" else 1
        m = re.search(r"\d+", name)
        idx = int(m.group(0)) if m else 0
        return (order, idx, name)

    actuators = sorted(grouped.keys(), key=_sort_key)
    mean_durations = []
    sat_rates = []
    for act in actuators:
        durs = grouped[act]["duration"]
        rates = grouped[act]["sat_any"]
        mean_durations.append(float(np.mean(durs)) if durs else 0.0)
        sat_rates.append(float(np.mean(rates)) if rates else 0.0)

    cmap = plt.cm.get_cmap("viridis")
    colors = [cmap(rate) for rate in sat_rates]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(actuators, mean_durations, color=colors, edgecolor="black", linewidth=0.8)
    ax.set_xlabel("Actuator", fontsize=14, fontweight="bold")
    ax.set_ylabel("Mean Saturation Duration (s)", fontsize=14, fontweight="bold")
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.grid(True, axis="y", alpha=0.3, linestyle=":", linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", labelrotation=45)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0.0, vmax=1.0))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.02)
    cbar.set_label("Saturation Rate", fontsize=12, fontweight="bold")

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def plot_actuator_timeseries(
    series_by_actuator: Dict[str, List[Dict[str, Any]]],
    output_path: Path,
    dpi: int = 300,
    sat_thresholds: Dict[str, float] | None = None,
) -> None:
    if not series_by_actuator:
        return

    actuators = sorted(series_by_actuator.keys())
    if not actuators:
        return

    levels = sorted({int(s["level"]) for series in series_by_actuator.values() for s in series if "level" in s})
    cmap = plt.cm.get_cmap("viridis", max(2, len(levels)))
    level_colors = {lvl: cmap(idx) for idx, lvl in enumerate(levels)}

    n_rows = len(actuators)
    fig, axes = plt.subplots(n_rows, 1, figsize=(10, max(3, n_rows * 2.2)), sharex=True)
    if n_rows == 1:
        axes = [axes]

    global_limits: Dict[str, Tuple[float, float]] = {}
    for prefix in ("u", "s"):
        all_vals: List[float] = []
        for actuator in actuators:
            if not actuator.startswith(prefix):
                continue
            for series in series_by_actuator.get(actuator, []):
                v = series.get("v")
                if v is None:
                    continue
                vals = np.asarray(v, dtype=float)
                vals = vals[np.isfinite(vals)]
                if vals.size:
                    all_vals.extend(vals.tolist())
        if all_vals:
            vals = np.asarray(all_vals, dtype=float)
            vals = vals[np.isfinite(vals)]
            if vals.size:
                y_lo = float(np.nanmin(vals))
                y_hi = float(np.nanmax(vals))
                if y_hi == y_lo:
                    pad = max(1.0, abs(y_hi) * 0.05)
                else:
                    pad = max((y_hi - y_lo) * 0.08, 1e-6)
                global_limits[prefix] = (y_lo - pad, y_hi + pad)

    for ax, actuator in zip(axes, actuators):
        series_list = sorted(series_by_actuator.get(actuator, []), key=lambda s: s.get("level", 0))
        y_values: List[float] = []
        for series in series_list:
            t = series.get("t")
            v = series.get("v")
            sat = series.get("sat")
            level = int(series.get("level", 0))
            if t is None or v is None or sat is None:
                continue
            color = level_colors.get(level, "#333333")
            ax.plot(t, v, linewidth=1.3, color=color, label=f"L{level:02d}")
            ax.fill_between(t, v, where=sat, color=color, alpha=0.15)
            y_values.extend(np.asarray(v, dtype=float).tolist())
        if y_values:
            y_vals = np.asarray(y_values, dtype=float)
            y_vals = y_vals[np.isfinite(y_vals)]
            if y_vals.size:
                prefix = actuator[0] if actuator else ""
                if prefix in global_limits:
                    y_lo, y_hi = global_limits[prefix]
                else:
                    y_lo = float(np.nanmin(y_vals))
                    y_hi = float(np.nanmax(y_vals))
                    if y_hi == y_lo:
                        pad = max(1.0, abs(y_hi) * 0.05)
                    else:
                        pad = max((y_hi - y_lo) * 0.08, 1e-6)
                    y_lo -= pad
                    y_hi += pad
                thr = None
                if sat_thresholds and actuator in sat_thresholds:
                    thr = float(sat_thresholds[actuator])
                    if y_lo <= thr <= y_hi:
                        ax.axhline(thr, color="#555555", linestyle="--", linewidth=1.0)
                        ax.axhline(-thr, color="#555555", linestyle="--", linewidth=1.0)
                    else:
                        ax.text(
                            0.98,
                            0.92,
                            f"sat ±{thr:g} (off-scale)",
                            transform=ax.transAxes,
                            ha="right",
                            va="top",
                            fontsize=9,
                            color="#555555",
                        )
                ax.set_ylim(y_lo, y_hi)
        ax.set_ylabel(actuator, fontsize=24, fontweight="bold")
        ax.grid(True, axis="y", alpha=0.3, linestyle=":", linewidth=0.8)

    axes[-1].set_xlabel("time (s)", fontsize=24, fontweight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(0.98, 0.98), frameon=True, fontsize=12, ncol=2)
    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
