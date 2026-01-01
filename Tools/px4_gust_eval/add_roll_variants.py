#!/usr/bin/env python3
"""
Add rate-parameter random sweep records to NocoDB.

Behavior:
- Fetch all NocoDB records (with pagination) to detect duplicates and derive parameter bounds.
- Default to all *RATE PID families; or accept explicit param names.
- Sample random PID triplets within existing min/max envelopes so cube plots获得立体散点。
- 仅写入 Title + PID 字段 + exp_times=0；跳过已存在 Title。

Usage:
  WANDB_API_KEY=... \
  NOCODB_TABLE_ID=... \
  NOCODB_VIEW_ID=... \
  NOCODB_TOKEN=... \
  uv run Tools/px4_gust_eval/add_roll_variants.py --random-count 5

Options:
  --param-names    Explicit param columns; omit to auto-detect *RATE params
  --random-count   Number of random PID samples per parameter base (default: 5)
  --seed           Seed for random sampling
  --dry-run        Print the would-be payload without POSTing
"""

from __future__ import annotations

import argparse
import json
import os
import urllib.parse
import urllib.request
import random
import re
from typing import Any, Dict, List, Optional, Set, Tuple


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Add rate PID random sweep rows to NocoDB")
    p.add_argument("--table-id", default=os.getenv("NOCODB_TABLE_ID"), help="NocoDB table id")
    p.add_argument("--view-id", default=os.getenv("NOCODB_VIEW_ID"), help="NocoDB view id")
    p.add_argument("--token", default=os.getenv("NOCODB_TOKEN"), help="NocoDB API token")
    p.add_argument("--base-url", default=os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com"), help="NocoDB base URL")
    p.add_argument("--max-records", type=int, default=500, help="Max records to fetch for duplicate detection")
    p.add_argument("--param-names", nargs="+", default=None, help="Specific PID param columns (default: auto-detect *RATE)")
    p.add_argument("--dry-run", action="store_true", help="Print payload without creating records")
    p.add_argument("--random-count", type=int, default=5, help="Number of random PID rows per param base")
    p.add_argument("--seed", type=int, default=None, help="Random seed for reproducible PID samples")
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
    records: List[Dict[str, Any]] = []
    offset = 0
    page_size = min(int(os.getenv("NOCODB_PAGE_SIZE", 100)), args.max_records)
    while offset < args.max_records:
        limit = min(page_size, args.max_records - len(records))
        if limit <= 0:
            break
        params = {"limit": limit, "offset": offset}
        if args.view_id:
            params["viewId"] = args.view_id
        data = _api_get(args, f"/api/v2/tables/{args.table_id}/records", params)
        batch = data.get("list") or []
        records.extend(batch)
        if len(batch) < limit:
            break
        offset += limit
    return records


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


def _param_base(name: str) -> Optional[str]:
    m = re.match(r"^(.*)_(P|I|D)$", name.strip())
    return m.group(1) if m else None


def _compute_bounds(records: List[Dict[str, Any]], base: str) -> Optional[Dict[str, Tuple[float, float]]]:
    bounds: Dict[str, Tuple[float, float]] = {}
    for term in ("P", "I", "D"):
        col = f"{base}_{term}"
        values: List[float] = []
        for rec in records:
            v = rec.get(col)
            if v is None:
                continue
            try:
                values.append(float(v))
            except Exception:
                continue
        if not values:
            return None
        bounds[term] = (min(values), max(values))
    return bounds


def _detect_rate_params(records: List[Dict[str, Any]]) -> List[str]:
    cols: Set[str] = set()
    for rec in records:
        cols.update(k for k in rec.keys() if isinstance(k, str))
    params = [
        col for col in cols if isinstance(col, str) and re.match(r"^MC_.*RATE_(P|I|D)$", col)
    ]
    return sorted(params)


def main() -> None:
    args = parse_args()
    rng = random.Random(args.seed)
    records = fetch_records(args)
    existing_titles: Set[str] = set()
    for r in records:
        title = str(r.get("Title") or "").strip()
        if title:
            existing_titles.add(title)

    target_params = args.param_names or _detect_rate_params(records)
    if not target_params:
        raise SystemExit("No PID columns detected. Provide --param-names explicitly.")

    bases = sorted({_param_base(name) for name in target_params if _param_base(name)})
    if not bases:
        raise SystemExit("Failed to derive PID bases from provided params.")

    new_rows: List[Dict[str, Any]] = []

    if args.random_count > 0:
        for base in bases:
            bounds = _compute_bounds(records, base)
            if bounds is None:
                print(f"[warn] Skip random rows for {base}: missing bounds for one of P/I/D")
                continue
            for idx in range(args.random_count):
                suffix = rng.randint(0, 999999)
                title = f"{base}_RAND_{idx+1:03d}_{suffix:06d}"
                if title in existing_titles:
                    continue
                row = {"Title": title, "exp_times": 0}
                for term, (vmin, vmax) in bounds.items():
                    row[f"{base}_{term}"] = rng.uniform(vmin, vmax)
                new_rows.append(row)
                existing_titles.add(title)
    else:
        print("[info] random-count is 0; nothing to add.")

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
