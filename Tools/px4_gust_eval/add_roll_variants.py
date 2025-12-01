#!/usr/bin/env python3
"""
Add roll-parameter sweep records to NocoDB.

Behavior:
- Fetch the "default" record from NocoDB to read baseline roll parameters.
- Generate 4 variants at 0.5x, 0.75x, 1.25x, 1.5x (based on the chosen param names).
- Only set Title + changed roll params (+exp_times=0); all other fields remain empty.
- Skip creation if a Title already exists.

Usage:
  WANDB_API_KEY=... \
  NOCODB_TABLE_ID=... \
  NOCODB_VIEW_ID=... \
  NOCODB_TOKEN=... \
  uv run Tools/px4_gust_eval/add_roll_variants.py --param-names MC_ROLL_P

Options:
  --param-names   Comma/space separated roll params to scale (default: MC_ROLL_P)
  --scales        Custom scales (default: 0.5 0.75 1.25 1.5)
  --dry-run       Print the would-be payload without POSTing
"""

from __future__ import annotations

import argparse
import json
import os
import urllib.parse
import urllib.request
from typing import Any, Dict, List, Set


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Add roll sweep rows to NocoDB")
    p.add_argument("--table-id", default=os.getenv("NOCODB_TABLE_ID"), help="NocoDB table id")
    p.add_argument("--view-id", default=os.getenv("NOCODB_VIEW_ID"), help="NocoDB view id")
    p.add_argument("--token", default=os.getenv("NOCODB_TOKEN"), help="NocoDB API token")
    p.add_argument("--base-url", default=os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com"), help="NocoDB base URL")
    p.add_argument("--max-records", type=int, default=500, help="Max records to fetch for duplicate detection")
    p.add_argument("--param-names", nargs="+", default=["MC_ROLL_P"], help="Roll param names to scale")
    p.add_argument("--scales", nargs="+", type=float, default=[0.5, 0.75, 1.25, 1.5], help="Scaling factors")
    p.add_argument("--dry-run", action="store_true", help="Print payload without creating records")
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
    with urllib.request.urlopen(req, timeout=15) as resp:
        resp.read()


def main() -> None:
    args = parse_args()
    records = fetch_records(args)
    existing_titles: Set[str] = set()
    default_row: Dict[str, Any] | None = None
    for r in records:
        title = str(r.get("Title") or "").strip()
        if title:
            existing_titles.add(title)
        if title.lower() == "default":
            default_row = r
    if default_row is None:
        raise SystemExit("No 'default' record found in NocoDB.")

    base_values: Dict[str, float] = {}
    for name in args.param_names:
        v = default_row.get(name)
        try:
            base_values[name] = float(v)
        except Exception:
            raise SystemExit(f"default row missing numeric value for {name}")

    new_rows: List[Dict[str, Any]] = []
    for scale in args.scales:
        for name in args.param_names:
            title = f"{name}_{scale}x"
            if title in existing_titles:
                print(f"[skip] exists: {title}")
                continue
            scaled = base_values[name] * scale
            row = {"Title": title, name: scaled, "exp_times": 0}
            new_rows.append(row)

    if not new_rows:
        print("Nothing to create (all variants exist?)")
        return

    print(f"Prepared {len(new_rows)} new rows:")
    print(json.dumps(new_rows, indent=2))

    if args.dry_run:
        print("Dry-run: no changes sent to NocoDB.")
        return

    post_records(args, new_rows)
    print("Rows created in NocoDB.")


if __name__ == "__main__":
    main()
