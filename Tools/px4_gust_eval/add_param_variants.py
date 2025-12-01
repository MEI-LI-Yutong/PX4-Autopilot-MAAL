#!/usr/bin/env python3
"""
Interactive helper to append parameter sweep rows to NocoDB.

Behavior:
- Fetch the latest table (optionally via view) and print existing titles/values for the chosen param.
- Based on the default row, generate new variants with a configurable start scale, step, and count.
- Only the changed param is set in each new row (plus Title/exp_times); other fields stay empty.
- Skips titles that already exist.

Usage example:
  NOCODB_TABLE_ID=... \
  NOCODB_VIEW_ID=... \
  NOCODB_TOKEN=... \
  uv run Tools/px4_gust_eval/add_param_variants.py --param-name MC_PITCH_P
"""

from __future__ import annotations

import argparse
import json
import os
import re
import urllib.parse
import urllib.request
import urllib.error
from typing import Any, Dict, List, Optional, Sequence, Tuple


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactively append parameter variants to NocoDB")
    p.add_argument("--table-id", default=os.getenv("NOCODB_TABLE_ID"), help="NocoDB table id")
    p.add_argument("--view-id", default=os.getenv("NOCODB_VIEW_ID"), help="NocoDB view id (optional)")
    p.add_argument("--token", default=os.getenv("NOCODB_TOKEN"), help="NocoDB API token")
    p.add_argument("--base-url", default=os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com"), help="NocoDB base URL")
    p.add_argument("--param-name", help="Parameter column to scale (e.g., MC_PITCH_P) [deprecated, use --param-names]")
    p.add_argument("--param-names", nargs="+", help="One or more parameter columns to scale")
    p.add_argument("--step", type=float, help="Multiplier increment (e.g., 0.25)")
    p.add_argument("--start-scale", type=float, help="Start multiplier for new rows (defaults to max existing + step)")
    p.add_argument("--count", type=int, help="How many rows to add per parameter")
    p.add_argument("--max-records", type=int, default=500, help="Max records to fetch for preview and duplicate detection")
    p.add_argument("--dry-run", action="store_true", help="Preview without POSTing new rows")
    return p.parse_args()


def _api_get(args: argparse.Namespace, path: str, params: Dict[str, Any]) -> Dict[str, Any]:
    query = urllib.parse.urlencode(params)
    url = f"{args.base_url.rstrip('/')}{path}?{query}"
    req = urllib.request.Request(url, headers={"xc-token": args.token})
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode("utf-8"))


def fetch_records(args: argparse.Namespace) -> List[Dict[str, Any]]:
    if not (args.table_id and args.token):
        raise SystemExit("NOCODB_TABLE_ID and NOCODB_TOKEN are required.")
    params = {"limit": args.max_records, "offset": 0}
    if args.view_id:
        params["viewId"] = args.view_id
    data = _api_get(args, f"/api/v2/tables/{args.table_id}/records", params)
    return data.get("list") or []


def post_records(args: argparse.Namespace, rows: List[Dict[str, Any]]) -> None:
    url = f"{args.base_url.rstrip('/')}/api/v2/tables/{args.table_id}/records"
    body = json.dumps(rows).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={"xc-token": args.token, "Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            resp.read()
    except urllib.error.HTTPError as exc:  # type: ignore[attr-defined]
        detail = ""
        try:
            detail = exc.read().decode("utf-8")
        except Exception:
            detail = ""
        raise SystemExit(f"NocoDB POST failed ({exc.code}): {detail or exc.reason}") from exc


def parse_scale_from_title(title: str, param_name: str) -> Optional[float]:
    pattern = rf"^{re.escape(param_name)}_([\d\.]+)x$"
    m = re.match(pattern, title.strip(), flags=re.IGNORECASE)
    if not m:
        return None
    try:
        return float(m.group(1))
    except Exception:
        return None


def prompt_with_default(prompt: str, default: str) -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return default if raw == "" else raw


def collect_existing(records: List[Dict[str, Any]], param_name: str) -> Tuple[Dict[str, float], List[float]]:
    titles = {}
    scales: List[float] = []
    for r in records:
        title = str(r.get("Title") or "").strip()
        if not title:
            continue
        s = parse_scale_from_title(title, param_name)
        if s is None:
            continue
        val = r.get(param_name)
        if val is None or val == "":
            continue
        titles[title] = val
        scales.append(s)
    return titles, scales


def detect_order_field(records: List[Dict[str, Any]]) -> Optional[str]:
    # NocoDB tables often have nc_order when manual ordering is enabled.
    for r in records:
        if "nc_order" in r:
            return "nc_order"
    return None


def find_default_row(records: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for r in records:
        title = str(r.get("Title") or "").strip().lower()
        if title == "default":
            return r
    return None


def main() -> None:
    args = parse_args()
    records = fetch_records(args)
    print(f"Fetched {len(records)} record(s) from NocoDB.")

    param_names: List[str] = []
    if args.param_names:
        param_names.extend(args.param_names)
    if args.param_name:
        param_names.append(args.param_name)
    if not param_names:
        name = input("Param name to scale (e.g., MC_PITCH_P), or multiple separated by space: ").strip()
        if name:
            param_names = name.split()
    if not param_names:
        raise SystemExit("Param name is required.")

    default_row = find_default_row(records)
    if default_row is None:
        raise SystemExit("No 'default' row found (Title=default).")

    order_field = detect_order_field(records)
    if order_field:
        print("Note: Table has a system ordering column; API does not allow updating it. New rows may appear at the end. You can sort by Title in the view to keep variants grouped.")

    # Resolve common step/count defaults (prompt once if not provided and single param)
    step_default = 0.25
    count_default = 4
    step: Optional[float] = args.step
    count: Optional[int] = args.count

    if len(param_names) == 1 and step is None:
        try:
            step = float(prompt_with_default("Step (multiplier increment, e.g., 0.25)", str(step_default)))
        except Exception:
            raise SystemExit("Invalid step.")
    if len(param_names) == 1 and count is None:
        try:
            count = int(prompt_with_default("How many new rows to add", str(count_default)))
        except Exception:
            raise SystemExit("Invalid count.")

    if step is None:
        step = step_default
    if count is None:
        count = count_default
    if count <= 0:
        raise SystemExit("Count must be positive.")

    new_rows: List[Dict[str, Any]] = []
    for param_name in param_names:
        try:
            base_value = float(default_row.get(param_name))
        except Exception as exc:
            print(f"[warn] default row missing numeric value for {param_name}, skip.")
            continue

        existing_titles, existing_scales = collect_existing(records, param_name)
        if existing_titles:
            print(f"\nCurrent rows for {param_name}:")
            for title, val in sorted(existing_titles.items()):
                print(f"  {title}: {val}")

        start_scale = args.start_scale
        if start_scale is None:
            start_scale = (max(existing_scales) if existing_scales else 1.0) + step

        for i in range(count):
            scale = start_scale + i * step
            title = f"{param_name}_{scale:g}x"
            if title in existing_titles:
                print(f"[skip] exists: {title}")
                continue
            row = {"Title": title, param_name: base_value * scale, "exp_times": 0}
            new_rows.append(row)

    if not new_rows:
        print("No new rows to create (all titles already present?).")
        return

    print("\nPrepared rows:")
    print(json.dumps(new_rows, indent=2))

    if args.dry_run:
        print("Dry-run: nothing sent to NocoDB.")
        return

    confirm = input("Create these rows in NocoDB? [y/N]: ").strip().lower()
    if confirm not in ("y", "yes"):
        print("Aborted by user.")
        return

    post_records(args, new_rows)
    print("Rows created.")


if __name__ == "__main__":
    main()
