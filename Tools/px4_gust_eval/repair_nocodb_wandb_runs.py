#!/usr/bin/env python3
"""
Repair NocoDB wandb_runid fields by re-scanning W&B runs.

Background:
- Earlier code overwrote (instead of append) the wandb_runid array in NocoDB.
- This script rebuilds wandb_runid per record by finding matching W&B runs.

Matching strategy (per NocoDB record):
1) Prefer W&B runs where config.nocodb_record.Id == <record Id>.
2) Fallback: W&B runs where config.nocodb_record.Title == <Title>.
3) Keep at most `exp_times` newest runs (exp_times is taken from the NocoDB record).

Updates are written back to NocoDB (unless --dry-run), replacing wandb_runid
with the rebuilt array. exp_times itself is not modified.

Usage example:
  WANDB_API_KEY=... \
  uv run Tools/px4_gust_eval/repair_nocodb_wandb_runs.py \
    --table-id "$NOCODB_TABLE_ID" --view-id "$NOCODB_VIEW_ID" --token "$NOCODB_TOKEN" \
    --wandb-entity MAALab --wandb-project px4_gust_eval
"""

from __future__ import annotations

import argparse
import json
import os
import urllib.parse
import urllib.request
from datetime import datetime
from typing import Any, Dict, List, Optional


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Rebuild NocoDB wandb_runid arrays using W&B runs")
    p.add_argument("--table-id", default=os.getenv("NOCODB_TABLE_ID"), required=False, help="NocoDB table id")
    p.add_argument("--view-id", default=os.getenv("NOCODB_VIEW_ID"), required=False, help="NocoDB view id")
    p.add_argument("--token", default=os.getenv("NOCODB_TOKEN"), required=False, help="NocoDB API token")
    p.add_argument("--base-url", default=os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com"), help="NocoDB base URL")
    p.add_argument("--max-records", type=int, default=300, help="Max NocoDB records to fetch")
    p.add_argument("--wandb-entity", default=os.getenv("WANDB_ENTITY"), help="Weights & Biases entity")
    p.add_argument("--wandb-project", default=os.getenv("WANDB_PROJECT", "px4_gust_eval"), help="Weights & Biases project")
    p.add_argument("--dry-run", action="store_true", help="Do not write back to NocoDB; just print planned updates")
    return p.parse_args()


def fetch_nocodb_records(args: argparse.Namespace) -> List[Dict[str, Any]]:
    if not (args.table_id and args.token):
        raise SystemExit("NocoDB table id and token are required (set NOCODB_TABLE_ID / NOCODB_TOKEN or CLI flags).")
    params = {"limit": args.max_records, "offset": 0}
    if args.view_id:
        params["viewId"] = args.view_id
    query = urllib.parse.urlencode(params)
    url = f"{args.base_url.rstrip('/')}/api/v2/tables/{args.table_id}/records?{query}"
    req = urllib.request.Request(url, headers={"xc-token": args.token})
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return data.get("list") or []
    except Exception as exc:
        raise SystemExit(f"Failed to fetch NocoDB records: {exc}") from exc


def find_wandb_runs(api: Any, entity: str, project: str, record: Dict[str, Any]) -> List[Any]:
    record_id = record.get("Id")
    title = record.get("Title")
    filters_id = {"config.nocodb_record.Id": record_id} if record_id is not None else None
    filters_title = {"config.nocodb_record.Title": title} if title else None
    runs: List[Any] = []
    try:
        if filters_id:
            runs = list(api.runs(f"{entity}/{project}", filters=filters_id))
    except Exception:
        runs = []
    if not runs and filters_title:
        try:
            runs = list(api.runs(f"{entity}/{project}", filters=filters_title))
        except Exception:
            runs = []
    return runs


def build_entries(runs: List[Any], limit: int) -> List[Dict[str, Any]]:
    # Sort newest first by created_at (fallback to summary timestamp)
    def _timestamp(r: Any) -> float:
        try:
            return r.created_at.timestamp()
        except Exception:
            pass
        try:
            return float(r.summary.get("_timestamp", 0.0))
        except Exception:
            return 0.0

    runs_sorted = sorted(runs, key=_timestamp, reverse=True)
    selected = runs_sorted[:max(0, int(limit))]
    entries: List[Dict[str, Any]] = []
    for r in selected:
        run_id = getattr(r, "id", None)
        ts = None
        try:
            ts = r.created_at.isoformat()
        except Exception:
            try:
                ts_val = r.summary.get("_timestamp")
                if ts_val is not None:
                    ts = datetime.fromtimestamp(float(ts_val)).isoformat()
            except Exception:
                ts = None
        if run_id:
            entries.append({"run_id": run_id, "run_at": ts})
    return entries


def patch_nocodb(args: argparse.Namespace, record_id: Any, entries: List[Dict[str, Any]]) -> None:
    url = f"{args.base_url.rstrip('/')}/api/v2/tables/{args.table_id}/records"
    payload_obj = {
        "Id": record_id,
        "wandb_runid": json.dumps(entries),
    }
    data = json.dumps([payload_obj]).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="PATCH", headers={
        "xc-token": args.token,
        "Content-Type": "application/json",
    })
    with urllib.request.urlopen(req, timeout=15) as resp:
        resp.read()


def main() -> None:
    args = parse_args()
    try:
        import wandb  # type: ignore
    except ImportError as exc:
        raise SystemExit("wandb is required. Install via pip.") from exc

    api = wandb.Api(timeout=120)
    records = fetch_nocodb_records(args)
    print(f"Fetched {len(records)} NocoDB records")

    for rec in records:
        record_id = rec.get("Id")
        title = rec.get("Title")
        exp_times = rec.get("exp_times", 0)
        try:
            desired = int(exp_times)
        except Exception:
            desired = 0

        runs = find_wandb_runs(api, args.wandb_entity, args.wandb_project, rec)
        if not runs:
            print(f"[skip] Id={record_id} Title={title}: no matching W&B runs found")
            continue

        entries = build_entries(runs, desired)
        print(f"[plan] Id={record_id} Title={title} exp_times={desired} -> {len(entries)} run(s)")
        for e in entries:
            print(f"   - {e['run_id']} at {e['run_at']}")

        if args.dry_run:
            continue
        try:
            patch_nocodb(args, record_id, entries)
            print(f"[ok] Updated NocoDB record {record_id}")
        except Exception as exc:
            print(f"[fail] NocoDB update for {record_id} failed: {exc}")


if __name__ == "__main__":
    main()
