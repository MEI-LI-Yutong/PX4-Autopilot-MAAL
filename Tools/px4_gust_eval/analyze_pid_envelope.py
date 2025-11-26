#!/usr/bin/env python3
"""
PID Gust Envelope Analyzer

Workflow:
- Fetch NocoDB records (Title + wandb_runid history) using the same env vars as the runner.
- For each wandb run, pull the cached/remote summary table at runs.summary["gust_summary/table"].
- Cache the table locally to avoid repeated downloads.
- Aggregate gust metrics per PID variant and generate scienceplots-based figures to show how the
  parameter sweep affects wind performance (multi-run friendly).

Example:
  uv run --with wandb Tools/px4_gust_eval/analyze_pid_envelope.py \
    --table-id "$NOCODB_TABLE_ID" --view-id "$NOCODB_VIEW_ID" --token "$NOCODB_TOKEN" \
    --wandb-entity MAALab --wandb-project px4_gust_eval
"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np
import re

try:
    import scienceplots  # type: ignore  # noqa: F401

    plt.style.use(["science", "no-latex"])
except Exception:
    sns.set_theme(context="paper", style="whitegrid")

# Default PID variants to inspect (Title field in NocoDB)
DEFAULT_TITLES = [
    "default",
    "MC_PITCH_P_0.5x",
    "MC_PITCH_P_0.75x",
    "MC_PITCH_P_1.25x",
    "MC_PITCH_P_1.5x",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Analyze gust summary tables across PID sweeps")
    p.add_argument("--table-id", default=os.getenv("NOCODB_TABLE_ID"), help="NocoDB table id")
    p.add_argument("--view-id", default=os.getenv("NOCODB_VIEW_ID"), help="NocoDB view id")
    p.add_argument("--token", default=os.getenv("NOCODB_TOKEN"), help="NocoDB API token")
    p.add_argument(
        "--base-url",
        default=os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com"),
        help="NocoDB base URL",
    )
    p.add_argument("--max-records", type=int, default=200, help="Max NocoDB records to fetch")
    p.add_argument(
        "--titles",
        nargs="*",
        default=None,
        help="Filter records by Title; omit to include all",
    )
    p.add_argument(
        "--param-name",
        default="MC_PITCH_P",
        help="Primary PID parameter name to annotate (if present in records)",
    )
    p.add_argument(
        "--cache-dir",
        type=Path,
        default=Path("Tools/px4_gust_eval/cache/wandb_tables"),
        help="Directory to store gust summary caches",
    )
    p.add_argument(
        "--output-dir",
        type=Path,
        default=Path("Tools/px4_gust_eval/plots/pid_envelope"),
        help="Directory to write figures and aggregated CSV",
    )
    p.add_argument("--refresh", action="store_true", help="Force re-download W&B tables even if cached")
    p.add_argument("--offline", action="store_true", help="Do not contact W&B (use cache only)")
    p.add_argument("--wandb-entity", default=os.getenv("WANDB_ENTITY"), help="Weights & Biases entity/team")
    p.add_argument("--wandb-project", default=os.getenv("WANDB_PROJECT", "px4_gust_eval"),
                   help="Weights & Biases project")
    p.add_argument("--dpi", type=int, default=300, help="Figure DPI")
    p.add_argument(
        "--metrics",
        nargs="*",
        default=["h_max_dev", "v_max_dev"],
        help="Metrics to plot (default: h_max_dev v_max_dev)",
    )
    p.add_argument(
        "--plot-3d",
        action="store_true",
        help="Plot 3D envelopes (level vs scale vs metric) grouped by param_name",
    )
    p.add_argument(
        "--q-low-high",
        nargs=2,
        type=float,
        default=(0.1, 0.9),
        metavar=("QLOW", "QHIGH"),
        help="Quantile band for envelope plots (default: 0.1 0.9)",
    )
    return p.parse_args()


def fetch_nocodb_records(
    table_id: str,
    token: str,
    base_url: str,
    view_id: Optional[str],
    max_records: int,
) -> List[Dict[str, Any]]:
    if not (table_id and token):
        raise SystemExit("NocoDB table id and token are required (set NOCODB_TABLE_ID / NOCODB_TOKEN).")

    params = {"limit": max_records, "offset": 0}
    if view_id:
        params["viewId"] = view_id
    query = urllib.parse.urlencode(params)
    url = f"{base_url.rstrip('/')}/api/v2/tables/{table_id}/records?{query}"
    req = urllib.request.Request(url, headers={"xc-token": token})
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return data.get("list") or []
    except Exception as exc:
        raise SystemExit(f"Failed to fetch NocoDB records: {exc}") from exc


def parse_run_entries(raw: Any) -> List[Dict[str, Any]]:
    if raw is None:
        return []
    parsed: Any = raw
    if isinstance(raw, str):
        try:
            parsed = json.loads(raw)
        except Exception:
            return []
    if isinstance(parsed, dict):
        parsed = [parsed]
    runs: List[Dict[str, Any]] = []
    if isinstance(parsed, list):
        for item in parsed:
            if isinstance(item, dict) and item.get("run_id"):
                runs.append(item)
    return runs


def load_cached_table(cache_file: Path) -> Optional[pd.DataFrame]:
    if not cache_file.is_file():
        return None
    try:
        payload = json.loads(cache_file.read_text(encoding="utf-8"))
        rows = payload.get("rows", [])
        if not rows:
            return None
        return pd.DataFrame(rows)
    except Exception:
        return None


def table_to_records(table_obj: Any) -> List[Dict[str, Any]]:
    if table_obj is None:
        return []
    # Mapping types first (SummarySubDict, dict)
    table_map: Optional[Dict[str, Any]] = None
    if hasattr(table_obj, "items"):
        try:
            table_map = dict(table_obj)
        except Exception:
            table_map = None
    if isinstance(table_obj, dict):
        table_map = table_obj
    if table_map is not None:
        cols = table_map.get("columns") or []
        data = table_map.get("data") or table_map.get("rows") or []
        if cols and data:
            return [dict(zip(cols, row)) for row in data]

    # Handle wandb.Table or similar (attribute access guarded)
    try:
        to_df = getattr(table_obj, "to_dataframe", None)
        if callable(to_df):
            df = to_df()
            return df.to_dict(orient="records")
    except Exception:
        pass
    try:
        cols_attr = getattr(table_obj, "columns", None)
        data_attr = getattr(table_obj, "data", None)
        if cols_attr is not None and data_attr is not None:
            cols = list(cols_attr)
            data = list(data_attr)
            if cols and data:
                return [dict(zip(cols, row)) for row in data]
    except Exception:
        pass

    return []


def _load_table_file(json_path: Path) -> List[Dict[str, Any]]:
    try:
        payload = json.loads(json_path.read_text(encoding="utf-8"))
    except Exception:
        return []
    cols = payload.get("columns") or payload.get("cols") or []
    data = payload.get("data") or payload.get("rows") or []
    if cols and data:
        return [dict(zip(cols, row)) for row in data]
    return []


def fetch_wandb_table(
    api: Any,
    entity: str,
    project: str,
    run_id: str,
) -> List[Dict[str, Any]]:
    run_path = f"{entity}/{project}/{run_id}"
    try:
        run = api.run(run_path)
        table_obj = run.summary.get("gust_summary/table")
        # Direct table or dict
        records = table_to_records(table_obj)
        if records:
            return records
        # Artifact reference fallback
        art_ref = None
        table_map: Optional[Dict[str, Any]] = None
        if hasattr(table_obj, "items"):
            try:
                table_map = dict(table_obj)  # SummarySubDict → plain dict
            except Exception:
                table_map = None
        if isinstance(table_obj, dict):
            table_map = table_obj
        if isinstance(table_map, dict):
            art_ref = table_map.get("_latest_artifact_path") or table_map.get("artifact_path")
        if isinstance(table_obj, str):
            art_ref = table_obj
        if art_ref:
            try:
                art = api.artifact(art_ref)
                with tempfile.TemporaryDirectory() as tmpdir:
                    art_dir = Path(art.download(root=tmpdir))
                    candidates = list(art_dir.rglob("*.table.json"))
                    if candidates:
                        return _load_table_file(candidates[0])
            except Exception:
                pass
        # Search logged artifacts for gust_summary table
        try:
            for art in run.logged_artifacts():
                if "gust_summary" in art.name and art.type == "run_table":
                    with tempfile.TemporaryDirectory() as tmpdir:
                        art_dir = Path(art.download(root=tmpdir))
                        candidates = [p for p in art_dir.rglob("*.table.json") if "gust_summary" in p.name]
                        if not candidates:
                            candidates = list(art_dir.rglob("*.table.json"))
                        if candidates:
                            return _load_table_file(candidates[0])
        except Exception:
            pass
    except Exception as exc:
        print(f"[warn] Failed to fetch W&B run {run_path}: {exc}")
        return []


def ensure_dirs(*paths: Path) -> None:
    for p in paths:
        p.mkdir(parents=True, exist_ok=True)


def cache_table(cache_dir: Path, run_id: str, rows: List[Dict[str, Any]]) -> Path:
    ensure_dirs(cache_dir)
    payload = {
        "run_id": run_id,
        "cached_at": datetime.utcnow().isoformat() + "Z",
        "rows": rows,
    }
    out_path = cache_dir / f"gust_summary_{run_id}.json"
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return out_path


def collect_tables(
    records: List[Dict[str, Any]],
    args: argparse.Namespace,
) -> pd.DataFrame:
    filtered: List[Dict[str, Any]] = []
    title_filter = set(args.titles) if args.titles else None
    for rec in records:
        title = str(rec.get("Title") or "").strip()
        if title_filter is not None and title and title not in title_filter:
            continue
        filtered.append(rec)

    if args.offline:
        api = None
    else:
        try:
            import wandb  # type: ignore
        except ImportError as exc:
            raise SystemExit("wandb is required to fetch runs. Install via `pip install wandb`.") from exc
        api = wandb.Api(timeout=120)

    all_rows: List[pd.DataFrame] = []
    for rec in filtered:
        title = str(rec.get("Title") or "").strip() or f"record_{rec.get('Id')}"
        param_val = rec.get(args.param_name)
        run_entries = parse_run_entries(rec.get("wandb_runid"))
        if not run_entries:
            print(f"[info] Skip {title}: no wandb_runid entries")
            continue
        for idx, run_entry in enumerate(run_entries):
            run_id = str(run_entry.get("run_id"))
            cache_file = args.cache_dir / f"gust_summary_{run_id}.json"
            df = None
            if not args.refresh:
                df = load_cached_table(cache_file)
            if df is None and not args.offline:
                rows = fetch_wandb_table(api, args.wandb_entity, args.wandb_project, run_id)
                if rows:
                    cache_table(args.cache_dir, run_id, rows)
                    df = pd.DataFrame(rows)
            if df is None or df.empty:
                print(f"[warn] No gust summary for run {run_id} ({title})")
                continue
            df = df.copy()
            df["variant"] = title
            df["param_value"] = param_val
            df["run_index"] = idx
            df["run_id"] = run_id
            df["record_id"] = rec.get("Id")
            all_rows.append(df)
    if not all_rows:
        raise SystemExit("No gust summary data found. Check NocoDB titles, wandb_runid, or cache.")
    combined = pd.concat(all_rows, ignore_index=True)
    return combined


def parse_variant_info(title: str) -> Tuple[Optional[str], Optional[float]]:
    """Extract (param_name, scale) from Title like MC_PITCH_P_0.75x."""
    m = re.match(r"^([A-Z0-9_]+)_([0-9.]+)x$", title)
    if not m:
        return None, None
    param = m.group(1)
    try:
        scale = float(m.group(2))
    except Exception:
        scale = None
    return param, scale


def compute_variant_order(df: pd.DataFrame, focus_param: Optional[str]) -> List[str]:
    """Order variants so that for the focus param scales ascend with default (scale=1) centered."""
    focus_list: List[Tuple[float, str]] = []
    other_list: List[Tuple[str, float, str]] = []
    seen = set()

    for _, row in df.iterrows():
        variant = str(row.get("variant"))
        if variant in seen:
            continue
        seen.add(variant)
        param = row.get("param_name")
        scale = row.get("scale")

        # Normalize default
        if variant.lower() == "default":
            param = focus_param or param
            scale = 1.0

        try:
            scale_val = float(scale)
        except Exception:
            scale_val = float("inf")

        if focus_param and param == focus_param:
            focus_list.append((scale_val, variant))
        else:
            other_list.append((str(param or ""), scale_val, variant))

    focus_order = [v for _, v in sorted(focus_list, key=lambda x: x[0])]
    other_order = [v for _, _, v in sorted(other_list, key=lambda x: (x[0], x[1]))]
    return focus_order + other_order


def expand_default_across_params(df: pd.DataFrame, focus_param: Optional[str]) -> pd.DataFrame:
    """Clone default rows across all param_names so each param has a scale=1.0 baseline."""
    default_rows = df[df["variant"].str.lower() == "default"]
    if default_rows.empty:
        return df
    params = set(p for p in df["param_name"].dropna().unique())
    if focus_param:
        params.add(focus_param)
    clones = []
    for p in params:
        has_default = not df[(df["param_name"] == p) & (df["variant"].str.lower() == "default")].empty
        if has_default:
            continue
        c = default_rows.copy()
        c["param_name"] = p
        c["scale"] = 1.0
        clones.append(c)
    if not clones:
        return df
    return pd.concat([df, *clones], ignore_index=True)


def plot_envelope(df: pd.DataFrame, metric: str, output: Path, dpi: int, focus_param: Optional[str]) -> None:
    fig, ax = plt.subplots(figsize=(9, 5))
    variants_ordered = compute_variant_order(df, focus_param)
    palette = sns.color_palette("tab10", n_colors=len(variants_ordered))
    for color, variant in zip(palette, variants_ordered):
        g = df[df["variant"] == variant]
        if g.empty:
            continue
        agg = g.groupby("level")[metric].agg(["mean", "min", "max", "count"]).sort_index()
        levels = agg.index.to_numpy()
        ax.plot(levels, agg["mean"], label=f"{variant} (n={agg['count'].sum():.0f})", color=color, marker="o")
        ax.fill_between(levels, agg["min"], agg["max"], color=color, alpha=0.15)
    ax.set_xlabel("Gust level")
    ax.set_ylabel(metric)
    ax.set_title(f"{metric} envelope vs gust level")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    ensure_dirs(output.parent)
    fig.tight_layout()
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_box(df: pd.DataFrame, metric: str, output: Path, dpi: int, focus_param: Optional[str]) -> None:
    fig, ax = plt.subplots(figsize=(9, 5))
    order = compute_variant_order(df, focus_param)
    sns.boxplot(data=df, x="variant", y=metric, ax=ax, showfliers=False, order=order)
    sns.stripplot(data=df, x="variant", y=metric, ax=ax, color="black", alpha=0.35, jitter=0.2, order=order)
    ax.set_title(f"{metric} distribution across PID variants")
    ax.set_xlabel("PID variant (Title)")
    ax.set_ylabel(metric)
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    ensure_dirs(output.parent)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_box_by_param(df: pd.DataFrame, metric: str, output_dir: Path, dpi: int) -> None:
    """Generate one box plot per param_name."""
    if "param_name" not in df.columns:
        return
    for param in sorted([p for p in df["param_name"].dropna().unique()]):
        sub = df[df["param_name"] == param]
        if sub.empty:
            continue
        order = compute_variant_order(sub, param)
        fig, ax = plt.subplots(figsize=(9, 5))
        sns.boxplot(data=sub, x="variant", y=metric, ax=ax, showfliers=False, order=order)
        sns.stripplot(data=sub, x="variant", y=metric, ax=ax, color="black", alpha=0.35, jitter=0.2, order=order)
        ax.set_title(f"{metric} distribution ({param})")
        ax.set_xlabel("PID variant")
        ax.set_ylabel(metric)
        ax.grid(True, axis="y", linestyle="--", alpha=0.4)
        fig.tight_layout()
        ensure_dirs(output_dir)
        fig.savefig(output_dir / f"{metric}_box_{param}.png", dpi=dpi)
        plt.close(fig)


def plot_grade_stack(df: pd.DataFrame, output: Path, dpi: int) -> None:
    if not {"grade_h", "grade_v"}.issubset(df.columns):
        return
    melted = pd.melt(
        df,
        id_vars=["variant", "run_id", "level"],
        value_vars=["grade_h", "grade_v"],
        var_name="dimension",
        value_name="grade",
    )
    counts = melted.groupby(["variant", "grade"]).size().reset_index(name="count")
    pivot = counts.pivot(index="variant", columns="grade", values="count").fillna(0)
    if pivot.empty:
        return
    pivot = pivot.loc[pivot.sum(axis=1).sort_values(ascending=False).index]
    fig, ax = plt.subplots(figsize=(9, 5))
    pivot.plot(kind="bar", stacked=True, ax=ax)
    ax.set_ylabel("Count")
    ax.set_title("Grade distribution by PID variant (H/V combined)")
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    ensure_dirs(output.parent)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_surface_by_param(df: pd.DataFrame, metric: str, output_dir: Path, dpi: int) -> None:
    """Plot 3D surface: gust level vs scale vs metric, per param_name."""
    if "param_name" not in df.columns or "scale" not in df.columns:
        return
    for param_name, g in df.groupby("param_name"):
        if g["scale"].nunique() < 2 or g["level"].nunique() < 2:
            continue
        pivot = (
            g.groupby(["scale", "level"])[metric]
            .mean()
            .reset_index()
            .pivot(index="scale", columns="level", values=metric)
        )
        pivot = pivot.sort_index()
        levels = np.array(sorted(pivot.columns))
        scales = np.array(sorted(pivot.index))
        L, S = np.meshgrid(levels, scales)
        Z = pivot.reindex(index=scales, columns=levels).to_numpy()

        fig = plt.figure(figsize=(9, 6))
        ax = fig.add_subplot(111, projection="3d")
        surf = ax.plot_surface(L, S, Z, cmap="viridis", edgecolor="none", alpha=0.85)
        ax.set_xlabel("Gust level")
        ax.set_ylabel("Scale")
        ax.set_zlabel(metric)
        ax.set_title(f"{param_name} - {metric} envelope")
        fig.colorbar(surf, shrink=0.6, aspect=12, label=metric)
        ensure_dirs(output_dir)
        fig.tight_layout()
        fig.savefig(output_dir / f"{metric}_surface_{param_name}.png", dpi=dpi)
        plt.close(fig)


def compute_stats_by_param(df: pd.DataFrame, metric: str, q_low: float, q_high: float) -> Dict[str, pd.DataFrame]:
    """Return per-param aggregated stats: mean/median/q_low/q_high by (scale, level)."""
    stats: Dict[str, pd.DataFrame] = {}
    if "param_name" not in df.columns or "scale" not in df.columns:
        return stats
    grouped = (
        df.groupby(["param_name", "scale", "level"])[metric]
        .agg(
            mean="mean",
            median="median",
            q_low=lambda s: s.quantile(q_low),
            q_high=lambda s: s.quantile(q_high),
            count="count",
        )
        .reset_index()
    )
    for param_name, g in grouped.groupby("param_name"):
        stats[param_name] = g.sort_values(["scale", "level"])
    return stats


def plot_band_envelope(stats: Dict[str, pd.DataFrame], metric: str, output_dir: Path, dpi: int) -> None:
    """Plot per-param quantile band envelopes."""
    for param, g in stats.items():
        fig, ax = plt.subplots(figsize=(9, 5))
        scales = sorted(g["scale"].unique())
        palette = sns.color_palette("tab10", n_colors=len(scales))
        for color, scale in zip(palette, scales):
            sub = g[g["scale"] == scale].sort_values("level")
            levels = sub["level"].to_numpy()
            ax.plot(levels, sub["median"], color=color, label=f"{param}_{scale}x (n~{int(sub['count'].max())})")
            ax.fill_between(levels, sub["q_low"], sub["q_high"], color=color, alpha=0.2)
        ax.set_xlabel("Gust level")
        ax.set_ylabel(metric)
        ax.set_title(f"{param} - {metric} quantile band")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend()
        fig.tight_layout()
        ensure_dirs(output_dir)
        fig.savefig(output_dir / f"{metric}_band_{param}.png", dpi=dpi)
        plt.close(fig)


def plot_heatmap_with_best(stats: Dict[str, pd.DataFrame], metric: str, output_dir: Path, dpi: int) -> List[Dict[str, Any]]:
    """Heatmap of mean (by default) with best scale path overlay; returns best path summary."""
    best_rows: List[Dict[str, Any]] = []
    for param, g in stats.items():
        pivot = g.pivot(index="scale", columns="level", values="mean")
        pivot = pivot.sort_index()
        if pivot.empty:
            continue
        fig, ax = plt.subplots(figsize=(8, 6))
        sns.heatmap(pivot, cmap="viridis", ax=ax, cbar_kws={"label": f"{metric} (mean)"})
        ax.set_title(f"{param} - {metric} heatmap (mean)")
        ax.set_xlabel("Gust level")
        ax.set_ylabel("Scale")

        # Best path per level (min mean)
        for level in pivot.columns:
            col = pivot[level]
            best_scale = col.idxmin()
            best_val = col.min()
            best_rows.append({"param_name": param, "metric": metric, "level": level, "best_scale": best_scale, "best_val": best_val})
            ax.plot([pivot.columns.get_loc(level) + 0.5], [list(pivot.index).index(best_scale) + 0.5], marker="o", color="red")

        fig.tight_layout()
        ensure_dirs(output_dir)
        fig.savefig(output_dir / f"{metric}_heatmap_{param}.png", dpi=dpi)
        plt.close(fig)
    return best_rows


def grade_to_score(grade: str) -> float:
    if not isinstance(grade, str):
        return float("nan")
    g = grade.lower()
    if "resilient" in g:
        return 2.0
    if "recoverable" in g:
        return 1.0
    if "unstable" in g:
        return 0.0
    if "not launched" in g:
        return -1.0
    return float("nan")


def plot_grade_bars(df: pd.DataFrame, output_dir: Path, dpi: int) -> None:
    """Bar plots of average grade score by gust level with scale as hue, per param."""
    if "param_name" not in df.columns or "scale" not in df.columns:
        return
    # Map to numeric score
    for col in ("grade_h", "grade_v"):
        if col in df.columns:
            df[f"{col}_score"] = df[col].apply(grade_to_score)

    for dim in ("grade_h_score", "grade_v_score"):
        if dim not in df.columns:
            continue
        for param, g in df.groupby("param_name"):
            agg = (
                g.groupby(["level", "scale"])[dim]
                .mean()
                .reset_index()
            )
            if agg.empty:
                continue
            # Ensure ordering
            try:
                scale_order = sorted(agg["scale"].dropna().unique())
            except Exception:
                scale_order = None
            level_order = sorted(agg["level"].dropna().unique())

            fig, ax = plt.subplots(figsize=(9, 5))
            sns.barplot(data=agg, x="level", y=dim, hue="scale", order=level_order, hue_order=scale_order, ax=ax)
            ax.set_title(f"{param} - {dim} vs gust level")
            ax.set_xlabel("Gust level")
            ax.set_ylabel(f"{dim} (mean score)")
            ax.grid(True, axis="y", linestyle="--", alpha=0.4)
            fig.tight_layout()
            ensure_dirs(output_dir)
            fname = f"{dim}_bars_{param}.png"
            fig.savefig(output_dir / fname, dpi=dpi)
            plt.close(fig)


def main() -> None:
    args = parse_args()
    records = fetch_nocodb_records(args.table_id, args.token, args.base_url, args.view_id, args.max_records)
    df = collect_tables(records, args)

    # Enrich metadata from Title (param name, scale) if possible
    df["param_name"], df["scale"] = zip(*df["variant"].map(parse_variant_info))
    # Ensure default sits at neutral scale for the chosen axis param
    mask_default = df["variant"].str.lower() == "default"
    if mask_default.any():
        df.loc[mask_default, "scale"] = 1.0
        if args.param_name:
            df.loc[mask_default, "param_name"] = args.param_name

    # Clone default rows across all params so each param has a baseline
    df = expand_default_across_params(df, args.param_name)

    ensure_dirs(args.output_dir)
    agg_csv = args.output_dir / "pid_gust_metrics.csv"
    df.to_csv(agg_csv, index=False)
    metrics = [m for m in args.metrics if m in df.columns]
    for metric in metrics:
        plot_envelope(df, metric, args.output_dir / f"{metric}_envelope.png", args.dpi, args.param_name)
        # Per-param boxes
        plot_box_by_param(df, metric, args.output_dir, args.dpi)
        if args.plot_3d:
            plot_surface_by_param(df, metric, args.output_dir, args.dpi)
        # Quantile band envelopes per param
        stats = compute_stats_by_param(df, metric, args.q_low_high[0], args.q_low_high[1])
        plot_band_envelope(stats, metric, args.output_dir, args.dpi)
        # Heatmap + best path
        best_rows = plot_heatmap_with_best(stats, metric, args.output_dir, args.dpi)
        if best_rows:
            best_csv = args.output_dir / f"best_path_{metric}.csv"
            pd.DataFrame(best_rows).to_csv(best_csv, index=False)
    # Grade bar plots (anti-wind level vs param/scale)
    plot_grade_bars(df, args.output_dir, args.dpi)
    plot_grade_stack(df, args.output_dir / "grade_stack.png", args.dpi)
    print(f"Aggregated {len(df)} rows from {df['run_id'].nunique()} runs across {df['variant'].nunique()} variants.")
    print(f"Saved CSV to {agg_csv}")
    for img in sorted(args.output_dir.glob("*.png")):
        print(f"Saved plot: {img}")


if __name__ == "__main__":
    main()
