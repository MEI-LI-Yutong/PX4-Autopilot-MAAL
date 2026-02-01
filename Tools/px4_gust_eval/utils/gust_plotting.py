from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Tuple

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
) -> None:
    dims = ["track_h", "track_v", "attitude", "actuator", "recovery", "wind_sense"]
    angles = np.linspace(0, 2 * np.pi, len(dims), endpoint=False).tolist()
    angles += angles[:1]

    fig = plt.figure(figsize=(6, 6))
    ax = plt.subplot(111, polar=True)

    colors = [COLORS["Unstable"], COLORS["Wind-Resilient"]]
    for idx, scores in enumerate(series):
        values = [scores.get(k, float("nan")) for k in dims]
        values = [0.0 if not np.isfinite(v) else float(v) for v in values]
        values += values[:1]
        ax.plot(angles, values, linewidth=2, color=colors[idx % len(colors)])
        ax.fill(angles, values, alpha=0.2, color=colors[idx % len(colors)])

    ax.set_thetagrids(np.degrees(angles[:-1]), dims, fontsize=18, fontweight="bold")
    ax.set_ylim(0, 1.0)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.tick_params(axis="y", labelsize=18)
    ax.grid(True, alpha=0.3)
    if labels:
        ax.legend(labels, loc="upper right", bbox_to_anchor=(1.3, 1.1), frameon=True, fontsize=18)

    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
