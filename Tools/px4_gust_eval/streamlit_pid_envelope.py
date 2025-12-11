#!/usr/bin/env python3
"""
Streamlit viewer for PID gust envelope results.

Usage:
  streamlit run Tools/px4_gust_eval/streamlit_pid_envelope.py \
    -- --summary Tools/px4_gust_eval/plots/pid_envelope/displacement_summary.csv \
    --failure Tools/px4_gust_eval/plots/pid_envelope/failure_levels.csv

Controls:
- Select family (e.g., PITCH, PITCHRATE, ROLL, ROLLRATE, YAW, YAWRATE).
- Filter scale range.
- 3D scatter (scale, failure_level, avg_displacement), color by term (P/I/D), point size by displacement.
- Hover shows Title/variant.
"""

from __future__ import annotations

import argparse
import pandas as pd
import streamlit as st
import plotly.express as px
from pathlib import Path
from typing import Optional


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Streamlit viewer for PID gust envelope")
    p.add_argument("--summary", type=Path, required=True, help="Path to displacement_summary.csv")
    p.add_argument("--failure", type=Path, required=True, help="Path to failure_levels.csv")
    return p.parse_args()


@st.cache_data(show_spinner=False)
def load_data(summary_path: Path, failure_path: Path) -> pd.DataFrame:
    disp = pd.read_csv(summary_path)
    fail = pd.read_csv(failure_path)
    merged = pd.merge(
        disp,
        fail,
        on=["param_name", "variant", "scale", "family", "term"],
        how="inner",
        suffixes=("_disp", "_fail"),
    )
    # Normalize scale numeric for sorting/filters
    try:
        merged["scale_num"] = merged["scale"].astype(float)
    except Exception:
        merged["scale_num"] = None
    return merged


def main() -> None:
    args = parse_args()
    df = load_data(args.summary, args.failure)

    st.title("PID Gust Envelope (Interactive 3D)")
    st.caption("Rotate/zoom the 3D scatter to inspect variants; color by term (P/I/D).")

    families = sorted(df["family"].dropna().unique()) if "family" in df else []
    if not families:
        st.error("No 'family' column found. Please run analyze_pid_envelope.py to regenerate summaries.")
        return

    selected_fam = st.selectbox("Family", families, index=0)
    sub = df[df["family"] == selected_fam]
    if sub.empty:
        st.warning(f"No data for family {selected_fam}")
        return

    # Scale filter
    valid_scales = sorted([float(s) for s in sub["scale"].dropna().unique()] + [1.0])
    if valid_scales:
        min_s, max_s = min(valid_scales), max(valid_scales)
    else:
        min_s, max_s = 0.0, 2.0
    scale_range = st.slider("Scale range", min_value=float(min_s), max_value=float(max_s), value=(float(min_s), float(max_s)), step=0.05)
    sub = sub[(sub["scale"].astype(float) >= scale_range[0]) & (sub["scale"].astype(float) <= scale_range[1])]

    st.markdown("**3D scatter (scale vs failure level vs avg displacement)**")
    fig = px.scatter_3d(
        sub,
        x="scale",
        y="failure_level",
        z="avg_displacement",
        color="term",
        size="avg_displacement",
        hover_data=["variant", "param_name"],
        labels={
            "scale": "Scale",
            "failure_level": "Failure level (first unstable; 12 if none)",
            "avg_displacement": "Avg displacement (h_max_dev + v_max_dev)",
            "term": "Term",
        },
    )
    fig.update_layout(height=700, legend_title="Term", margin=dict(l=0, r=0, b=0, t=30))
    st.plotly_chart(fig, use_container_width=True)

    st.markdown("**Data table**")
    st.dataframe(sub[["variant", "param_name", "term", "scale", "failure_level", "avg_displacement"]].sort_values(["term", "scale"]))


if __name__ == "__main__":
    main()
