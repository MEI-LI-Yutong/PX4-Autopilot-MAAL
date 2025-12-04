#!/usr/bin/env python3
"""
PID Gust Envelope Analyzer (simplified)

Workflow:
- Fetch NocoDB records (Title + wandb_runid history) using the same env vars as the runner.
- For each wandb run, pull the cached/remote summary table at runs.summary["gust_summary/table"].
- Cache the table locally to avoid repeated downloads.
- Compute a single metric per PID variant: 平均位移误差 = (各风级的水平最大位移均值 + 垂直最大位移均值) 的平均。
- 输出 CSV，并为每个参数画一张按倍率排序的条形图（纵轴为上述平均位移误差）。
- 额外生成一张总览图：每个参数一个子图，方块颜色表示“首次出现 unstable 的风级”（未出现则视为 12 级）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import tempfile
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # for 3D scatter

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
    p.add_argument("--max-records", type=int, default=1000, help="Max NocoDB records to fetch")
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

    page_size = min(max_records, int(os.getenv("NOCODB_PAGE_SIZE", 100)))
    url_base = f"{base_url.rstrip('/')}/api/v2/tables/{table_id}/records"
    offset = 0
    records: List[Dict[str, Any]] = []
    while offset < max_records:
        limit = min(page_size, max_records - len(records))
        params = {"limit": limit, "offset": offset}
        if view_id:
            params["viewId"] = view_id
        query = urllib.parse.urlencode(params)
        url = f"{url_base}?{query}"
        req = urllib.request.Request(url, headers={"xc-token": token})
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except Exception as exc:
            raise SystemExit(f"Failed to fetch NocoDB records at offset {offset}: {exc}") from exc

        page = data.get("list") or []
        total = data.get("totalRows")
        print(f"[info] NocoDB page: fetched {len(page)} (offset={offset}, limit={limit}, total={total})")
        if not page:
            break
        records.extend(page)
        offset += limit
        if len(page) < limit:
            break
    return records


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


def compute_displacement_summary(df: pd.DataFrame) -> pd.DataFrame:
    """
    计算平均位移误差：
    - 先按 (param_name, variant, scale, level) 分组，取 h_max_dev、v_max_dev 的均值。
    - 对每个 (param_name, variant, scale) 求 (h_mean + v_mean) 在各风级的平均。
    """
    required = {"h_max_dev", "v_max_dev", "level", "variant"}
    if not required.issubset(df.columns):
        raise SystemExit(f"Missing required columns for displacement metric: {required - set(df.columns)}")
    if "param_name" not in df.columns:
        df["param_name"] = None
    if "scale" not in df.columns:
        df["scale"] = np.nan

    grouped = (
        df.groupby(["param_name", "variant", "scale", "level"])[["h_max_dev", "v_max_dev"]]
        .mean()
        .reset_index()
    )
    grouped["disp_level"] = grouped["h_max_dev"] + grouped["v_max_dev"]

    agg = (
        grouped.groupby(["param_name", "variant", "scale"])["disp_level"]
        .mean()
        .reset_index()
        .rename(columns={"disp_level": "avg_displacement"})
    )
    agg["family"], agg["term"] = zip(*agg["param_name"].map(_parse_family_term))
    return agg


def _is_unstable(grade: Any) -> bool:
    if not isinstance(grade, str):
        return False
    g = grade.lower()
    return ("unstable" in g) or ("not launched" in g) or ("crash" in g)


def _parse_family_term(param: Optional[str]) -> Tuple[Optional[str], Optional[str]]:
    if not param or not isinstance(param, str):
        return None, None
    m = re.match(r"^MC_([A-Z]+?)(?:RATE)?_(P|I|D)$", param)
    if not m:
        return None, None
    base = m.group(1)
    term = m.group(2)
    family = base.lower()
    if "rate" in param.lower():
        family = f"{family}rate"
    return family.upper(), term


def _is_unstable(grade: Any) -> bool:
    if not isinstance(grade, str):
        return False
    g = grade.lower()
    return ("unstable" in g) or ("not launched" in g) or ("crash" in g)


def compute_failure_levels(df: pd.DataFrame, default_level: float = 12.0) -> pd.DataFrame:
    """
    计算首次出现 unstable 的风级；若未出现则记为 default_level（默认 12）。
    使用 grade_h/grade_v（如果存在）判定。
    """
    if "param_name" not in df.columns:
        df["param_name"] = None
    if "scale" not in df.columns:
        df["scale"] = np.nan

    results: List[Dict[str, Any]] = []
    for (param, variant, scale), g in df.groupby(["param_name", "variant", "scale"]):
        levels = sorted(g["level"].dropna().unique())
        failure = None
        for lvl in levels:
            sub = g[g["level"] == lvl]
            # 若任一等级在 H/V 维度出现 unstable，则认定为失败等级
            cond_h = sub["grade_h"].apply(_is_unstable) if "grade_h" in sub.columns else pd.Series([False])
            cond_v = sub["grade_v"].apply(_is_unstable) if "grade_v" in sub.columns else pd.Series([False])
            if bool(cond_h.any() or cond_v.any()):
                failure = lvl
                break
        if failure is None:
            failure = default_level
        results.append({
            "param_name": param,
            "variant": variant,
            "scale": scale,
            "failure_level": failure,
            "family": _parse_family_term(param)[0],
            "term": _parse_family_term(param)[1],
        })
    return pd.DataFrame(results)


def _variant_labels(sub: pd.DataFrame) -> Tuple[List[str], List[str]]:
    """Return tick labels (variant short) and text labels (value formatted)."""
    labels = []
    for scale, var in zip(sub["scale"], sub["variant"]):
        try:
            lbl = f"{float(scale):g}x"
        except Exception:
            lbl = var
        labels.append(lbl)
    return labels, [f"{v:.2f}" for v in sub["avg_displacement"]]


def plot_displacement_bars(summary: pd.DataFrame, output_dir: Path, dpi: int) -> List[Path]:
    saved: List[Path] = []
    if summary.empty:
        return saved
    ensure_dirs(output_dir)
    for param, sub in summary.groupby("param_name"):
        # Order by numeric scale if present, otherwise Title order
        try:
            order = [v for _, v in sorted(((float(s), var) for s, var in zip(sub["scale"], sub["variant"])), key=lambda x: x[0])]
        except Exception:
            order = list(sub["variant"])
        fig, ax = plt.subplots(figsize=(9, 5))
        sns.barplot(data=sub, x="variant", y="avg_displacement", order=order, ax=ax, color="#4C72B0")
        ticks, texts = _variant_labels(sub if order is None else sub.set_index("variant").loc[order].reset_index())
        ax.set_xticklabels(ticks, rotation=30, ha="right")
        ax.set_xlabel("Scale")
        ax.set_ylabel("Avg displacement (h_max_dev + v_max_dev)")
        ax.set_title(f"{param} average displacement")
        ax.grid(True, axis="y", linestyle="--", alpha=0.4)
        for p, txt in zip(ax.patches, texts):
            ax.annotate(txt, (p.get_x() + p.get_width() / 2, p.get_height()),
                        ha="center", va="bottom", fontsize=8)
        fig.tight_layout()
        out = output_dir / f"{param}_avg_displacement.png"
        fig.savefig(out, dpi=dpi)
        plt.close(fig)
        saved.append(out)
    return saved


def _grid_for_metric(df: pd.DataFrame, value_col: str, title: str, cbar_label: str, output: Path, dpi: int) -> Optional[Path]:
    if df.empty or "param_name" not in df.columns:
        return None
    ensure_dirs(output.parent)
    df = df.copy()
    try:
        df["scale_num"] = df["scale"].astype(float)
    except Exception:
        df["scale_num"] = np.nan

    params = sorted([p for p in df["param_name"].dropna().unique()])
    if not params:
        return None

    vmin = df[value_col].min()
    vmax = df[value_col].max()
    cmap = plt.cm.RdYlGn if value_col == "failure_level" else plt.cm.viridis
    ncols = 4
    nrows = math.ceil(len(params) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 2.8 * nrows), squeeze=False)

    last_im = None
    for idx, param in enumerate(params):
        ax = axes[idx // ncols][idx % ncols]
        sub = df[df["param_name"] == param]
        if sub.empty:
            ax.axis("off")
            continue
        if sub["scale_num"].notna().any():
            sub = sub.sort_values("scale_num")
        else:
            sub = sub.sort_values("variant")
        values = sub[value_col].to_numpy()
        im_data = values.reshape(1, -1)
        last_im = ax.imshow(im_data, aspect="auto", cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_xticks(range(len(sub)))
        try:
            xlabels = [f"{float(s):g}x" for s in sub["scale"]]
        except Exception:
            xlabels = list(sub["variant"])
        ax.set_xticklabels(xlabels, rotation=30, ha="right", fontsize=8)
        ax.set_yticks([])
        ax.set_title(param, fontsize=11)
        # 仅在 failure 图上标数字，displacement 图不标注
        if value_col == "failure_level":
            for x, val in enumerate(values):
                ax.text(x, 0, f"{val:.0f}", ha="center", va="center", color="black", fontsize=8, fontweight="bold")

    for j in range(len(params), nrows * ncols):
        axes[j // ncols][j % ncols].axis("off")

    fig.suptitle(title, fontsize=12)
    # 留出右侧空间放色条，避免与子图重叠
    fig.subplots_adjust(right=0.86, top=0.92)
    if last_im is not None:
        cax = fig.add_axes([0.88, 0.15, 0.02, 0.7])
        cbar = fig.colorbar(last_im, cax=cax)
        cbar.set_label(cbar_label, fontsize=10)

    fig.tight_layout(rect=[0, 0, 0.86, 0.90])
    fig.savefig(output, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return output


def _family_heatmaps(summary: pd.DataFrame, value_col: str, title_prefix: str, cbar_label: str, output_dir: Path, dpi: int, annotate: bool) -> Optional[Path]:
    if summary.empty or "family" not in summary.columns:
        return None
    ensure_dirs(output_dir)
    families_all = sorted([f for f in summary["family"].dropna().unique()])
    if not families_all:
        return None
    # Group into three buckets: PITCH/PITCHRATE, ROLL/ROLLRATE, YAW/YAWRATE (case-insensitive)
    buckets = {
        "PITCH": [f for f in families_all if f.upper().startswith("PITCH")],
        "ROLL": [f for f in families_all if f.upper().startswith("ROLL")],
        "YAW": [f for f in families_all if f.upper().startswith("YAW")],
    }
    # If some family did not match, keep them in a misc bucket
    misc = [f for f in families_all if f not in buckets["PITCH"] + buckets["ROLL"] + buckets["YAW"]]

    vmax = summary[value_col].max()
    vmin = summary[value_col].min()
    cmap = plt.cm.RdYlGn if value_col == "failure_level" else plt.cm.viridis

    outputs = []
    def draw_bucket(bucket_name: str, fams: list[str]) -> None:
        if not fams:
            return
        ncols = min(2, len(fams))
        nrows = math.ceil(len(fams) / ncols)
        fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.5 * nrows), squeeze=False)
        last_im = None
        for idx, fam in enumerate(fams):
            ax = axes[idx // ncols][idx % ncols]
            sub = summary[summary["family"] == fam]
            if sub.empty:
                ax.axis("off")
                continue
            try:
                sub["scale_num"] = sub["scale"].astype(float)
            except Exception:
                sub["scale_num"] = np.nan
            scales = sorted([s for s in sub["scale_num"].unique() if not pd.isna(s)])
            if not scales:
                scales = list(range(len(sub)))
            terms = ["P", "I", "D"]
            pivot = sub.pivot_table(index="term", columns="scale_num", values=value_col, aggfunc="mean")
            pivot = pivot.reindex(index=terms)
            pivot = pivot[scales] if all(s in pivot.columns for s in scales) else pivot
            im = sns.heatmap(
                pivot,
                ax=ax,
                cmap=cmap,
                vmin=vmin,
                vmax=vmax,
                cbar=False,
                annot=annotate,
                fmt=".1f" if value_col != "failure_level" else ".0f",
                annot_kws={"fontsize": 8},
            )
            last_im = im.collections[0]
            ax.set_title("")  # no title
            ax.set_xlabel("Scale")
            ax.set_ylabel("Term")

        for j in range(len(fams), nrows * ncols):
            axes[j // ncols][j % ncols].axis("off")

        fig.subplots_adjust(right=0.88, top=0.92)
        if last_im is not None:
            cax = fig.add_axes([0.90, 0.15, 0.02, 0.7])
            cbar = fig.colorbar(last_im, cax=cax)
            cbar.set_label(cbar_label, fontsize=10)

        fig.suptitle("")  # no suptitle
        fig.tight_layout(rect=[0, 0, 0.88, 0.90])
        out = output_dir / f"{value_col}_family_{bucket_name}.png"
        fig.savefig(out, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        outputs.append(out)

    draw_bucket("PITCH", buckets["PITCH"])
    draw_bucket("ROLL", buckets["ROLL"])
    draw_bucket("YAW", buckets["YAW"])
    if misc:
        draw_bucket("MISC", misc)

    return outputs[0] if outputs else None


def _scatter3d_by_family(merged: pd.DataFrame, output_dir: Path, dpi: int) -> List[Path]:
    """
    3D scatter: x=scale, y=failure_level, z=avg_displacement, color by term, per family.
    Helps visualize resilience vs displacement across scales/terms.
    """
    saved: List[Path] = []
    if merged.empty or "family" not in merged.columns:
        return saved
    ensure_dirs(output_dir)
    for fam, sub in merged.groupby("family"):
        if sub.empty:
            continue
        try:
            sub["scale_num"] = sub["scale"].astype(float)
        except Exception:
            sub["scale_num"] = np.nan
        fig = plt.figure(figsize=(7, 5))
        ax = fig.add_subplot(111, projection="3d")
        terms = sorted(sub["term"].dropna().unique())
        colors = sns.color_palette("tab10", n_colors=len(terms))
        color_map = {t: colors[i] for i, t in enumerate(terms)}
        for _, row in sub.iterrows():
            c = color_map.get(row.get("term"))
            ax.scatter(
                row.get("scale_num"),
                row.get("failure_level"),
                row.get("avg_displacement"),
                color=c,
                s=50,
            )
        ax.set_xlabel("Scale")
        ax.set_ylabel("Failure level")
        ax.set_zlabel("Avg displacement")
        ax.set_title(f"{fam} 3D scatter")
        legend_handles = [plt.Line2D([0], [0], marker="o", color=c, linestyle="", label=t) for t, c in color_map.items()]
        ax.legend(handles=legend_handles, loc="best")
        fig.tight_layout()
        out = output_dir / f"{fam}_scatter3d.png"
        fig.savefig(out, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        saved.append(out)
    return saved


def plot_overview_grid(failure: pd.DataFrame, output_dir: Path, dpi: int) -> Optional[Path]:
    return _grid_for_metric(
        failure,
        value_col="failure_level",
        title="Failure level by scale",
        cbar_label="First unstable wind level (12 if none)",
        output=output_dir / "overview_failure_grid.png",
        dpi=dpi,
    )


def plot_displacement_grid(summary: pd.DataFrame, output_dir: Path, dpi: int) -> Optional[Path]:
    return _grid_for_metric(
        summary,
        value_col="avg_displacement",
        title="Average displacement by scale",
        cbar_label="Avg displacement (h_max_dev + v_max_dev)",
        output=output_dir / "overview_displacement_grid.png",
        dpi=dpi,
    )


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

    disp_summary = compute_displacement_summary(df)
    disp_csv = args.output_dir / "displacement_summary.csv"
    disp_summary.to_csv(disp_csv, index=False)

    failure_summary = compute_failure_levels(df)
    failure_csv = args.output_dir / "failure_levels.csv"
    failure_summary.to_csv(failure_csv, index=False)
    merged = pd.merge(
        disp_summary,
        failure_summary,
        on=["param_name", "variant", "scale", "family", "term"],
        how="inner",
        suffixes=("_disp", "_fail"),
    )

    saved_imgs = plot_displacement_bars(disp_summary, args.output_dir, args.dpi)
    overview_fail = plot_overview_grid(failure_summary, args.output_dir, args.dpi)
    overview_disp = plot_displacement_grid(disp_summary, args.output_dir, args.dpi)
    heatmap_fail = _family_heatmaps(failure_summary, "failure_level", "Failure level", "First unstable wind level (12 if none)", args.output_dir, args.dpi, annotate=True)
    heatmap_disp = _family_heatmaps(disp_summary, "avg_displacement", "Average displacement", "Avg displacement (h_max_dev + v_max_dev)", args.output_dir, args.dpi, annotate=False)
    scatter_imgs = _scatter3d_by_family(merged, args.output_dir, args.dpi)

    print(f"Aggregated {len(df)} rows from {df['run_id'].nunique()} runs across {df['variant'].nunique()} variants.")
    print(f"Saved raw CSV to {agg_csv}")
    print(f"Saved displacement summary to {disp_csv}")
    print(f"Saved failure level summary to {failure_csv}")
    for img in saved_imgs:
        print(f"Saved plot: {img}")
    if overview_fail:
        print(f"Saved failure overview grid: {overview_fail}")
    if overview_disp:
        print(f"Saved displacement overview grid: {overview_disp}")
    if heatmap_fail:
        print(f"Saved failure family heatmaps: {heatmap_fail}")
    if heatmap_disp:
        print(f"Saved displacement family heatmaps: {heatmap_disp}")
    for img in scatter_imgs:
        print(f"Saved 3D scatter: {img}")


if __name__ == "__main__":
    main()
