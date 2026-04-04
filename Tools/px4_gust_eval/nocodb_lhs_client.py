#!/usr/bin/env python3
"""
Interactive NocoDB client for LHS-based SDF parameter generation.

Features:
- Read table metadata and preview records
- Generate LHS samples within configured ranges
- Write sdf_edit_json rows into NocoDB
- Update and delete records (with confirmation)

Usage:
  NOCODB_TABLE_ID=... \
  NOCODB_VIEW_ID=... \
  NOCODB_TOKEN=... \
  uv run Tools/px4_gust_eval/nocodb_lhs_client.py
"""

from __future__ import annotations

import argparse
import json
import os
import random
import urllib.parse
import urllib.request
import urllib.error
from typing import Any, Dict, List, Optional, Sequence, Tuple


DEFAULT_SCHEMA = {
    "sdf_path": "../../simulation/gz/models/x500_base/model.sdf",
    "link_name": "base_link",
    "params": {
        "mass": {"min": 1.0, "max": 3.0, "path": "inertial/mass"},
        "ixx": {"min": 0.02, "max": 0.06, "path": "inertial/inertia/ixx"},
        "iyy": {"min": 0.02, "max": 0.06, "path": "inertial/inertia/iyy"},
        "izz": {"min": 0.01, "max": 0.04, "path": "inertial/inertia/izz"},
    },
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive NocoDB LHS client (SDF patch rows)")
    p.add_argument("--table-id", default=os.getenv("NOCODB_TABLE_ID"), help="NocoDB table id")
    p.add_argument("--view-id", default=os.getenv("NOCODB_VIEW_ID"), help="NocoDB view id (optional)")
    p.add_argument("--token", default=os.getenv("NOCODB_TOKEN"), help="NocoDB API token")
    p.add_argument("--base-url", default=os.getenv("NOCODB_BASE_URL", "https://app.nocodb.com"), help="NocoDB base URL")
    p.add_argument("--schema", help="Path to schema JSON (optional)")
    p.add_argument("--max-records", type=int, default=200, help="Max records to fetch for preview")
    p.add_argument("--seed", type=int, help="Random seed for LHS sampling")
    p.add_argument("--dry-run", action="store_true", help="Preview without writing to NocoDB")
    return p.parse_args()


def _api_get(args: argparse.Namespace, path: str, params: Dict[str, Any]) -> Dict[str, Any]:
    query = urllib.parse.urlencode(params)
    url = f"{args.base_url.rstrip('/')}{path}?{query}"
    req = urllib.request.Request(url, headers={"xc-token": args.token})
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _api_post(args: argparse.Namespace, path: str, payload: List[Dict[str, Any]]) -> None:
    url = f"{args.base_url.rstrip('/')}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        method="POST",
        headers={"xc-token": args.token, "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        resp.read()


def _api_patch(args: argparse.Namespace, path: str, payload: List[Dict[str, Any]]) -> None:
    url = f"{args.base_url.rstrip('/')}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        method="PATCH",
        headers={"xc-token": args.token, "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        resp.read()


def _api_delete(args: argparse.Namespace, path: str, payload: List[Dict[str, Any]]) -> None:
    url = f"{args.base_url.rstrip('/')}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        method="DELETE",
        headers={"xc-token": args.token, "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        resp.read()


def fetch_records(args: argparse.Namespace) -> List[Dict[str, Any]]:
    if not (args.table_id and args.token):
        raise SystemExit("NOCODB_TABLE_ID and NOCODB_TOKEN are required.")
    page_size = min(args.max_records, int(os.getenv("NOCODB_PAGE_SIZE", 100)))
    records: List[Dict[str, Any]] = []
    offset = 0
    while offset < args.max_records:
        limit = min(page_size, args.max_records - len(records))
        params = {"limit": limit, "offset": offset}
        if args.view_id:
            params["viewId"] = args.view_id
        data = _api_get(args, f"/api/v2/tables/{args.table_id}/records", params)
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


def load_schema(path: Optional[str]) -> Dict[str, Any]:
    if not path:
        return DEFAULT_SCHEMA
    with open(path, "r") as f:
        return json.load(f)


def prompt_with_default(prompt: str, default: str) -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return default if raw == "" else raw


def lhs_samples(n: int, dims: int, seed: Optional[int]) -> List[List[float]]:
    rng = random.Random(seed)
    samples = [[0.0 for _ in range(dims)] for _ in range(n)]
    for j in range(dims):
        perm = list(range(n))
        rng.shuffle(perm)
        for i in range(n):
            u = (perm[i] + rng.random()) / n
            samples[i][j] = u
    return samples


def build_patches(schema: Dict[str, Any], values: Dict[str, float]) -> List[Dict[str, Any]]:
    link_name = schema.get("link_name")
    params = schema.get("params", {})
    patches: List[Dict[str, Any]] = []
    for key, val in values.items():
        entry = params.get(key, {})
        path = entry.get("path")
        if not (link_name and path):
            continue
        patches.append({"select": {"link": str(link_name), "path": str(path)}, "value": val})
    return patches


def build_sdf_edit(schema: Dict[str, Any], values: Dict[str, float]) -> Dict[str, Any]:
    sdf_path = schema.get("sdf_path")
    patches = build_patches(schema, values)
    return {"sdf_edit": {"path": sdf_path, "patches": patches}}


def list_table_info(records: List[Dict[str, Any]]) -> None:
    if not records:
        print("No records found.")
        return
    keys = list(records[0].keys())
    print("\nTable columns:")
    for k in keys:
        print(f"  - {k}")


def list_schema_info(schema: Dict[str, Any]) -> None:
    print("\nSchema:")
    print(f"  sdf_path: {schema.get('sdf_path')}")
    print(f"  link_name: {schema.get('link_name')}")
    params = schema.get("params", {})
    for name, cfg in params.items():
        print(f"  {name}: [{cfg.get('min')}, {cfg.get('max')}] path={cfg.get('path')}")


def generate_lhs_rows(schema: Dict[str, Any], count: int, seed: Optional[int], title_prefix: str) -> List[Dict[str, Any]]:
    params = schema.get("params", {})
    keys = list(params.keys())
    dims = len(keys)
    if dims == 0:
        raise SystemExit("Schema has no params.")
    samples = lhs_samples(count, dims, seed)
    rows: List[Dict[str, Any]] = []
    for i, uvec in enumerate(samples):
        values: Dict[str, float] = {}
        for k, u in zip(keys, uvec):
            cfg = params.get(k, {})
            vmin = float(cfg.get("min"))
            vmax = float(cfg.get("max"))
            values[k] = vmin + u * (vmax - vmin)
        payload = build_sdf_edit(schema, values)
        title = f"{title_prefix}{i:04d}"
        rows.append({"Title": title, "exp_times": 0, "sdf_edit_json": payload})
    return rows


def menu_loop(args: argparse.Namespace, schema: Dict[str, Any]) -> None:
    records = fetch_records(args)
    existing_titles = {str(r.get("Title") or "") for r in records}

    while True:
        print("\nMenu:")
        print("  1) 查看表字段")
        print("  2) 查看参数范围")
        print("  3) 预览 LHS 样本")
        print("  4) 写入 LHS 样本到 NocoDB")
        print("  5) 更新单条记录（sdf_edit_json）")
        print("  6) 删除单条记录")
        print("  0) 退出")
        choice = input("选择: ").strip()

        if choice == "1":
            list_table_info(records)
        elif choice == "2":
            list_schema_info(schema)
        elif choice == "3":
            count = int(prompt_with_default("样本数量", "500"))
            prefix = prompt_with_default("Title 前缀", "sdf_lhs_")
            rows = generate_lhs_rows(schema, count, args.seed, prefix)
            print(json.dumps(rows[:5], indent=2))
            print(f"... total {len(rows)} rows (preview only).")
        elif choice == "4":
            count = int(prompt_with_default("样本数量", "500"))
            prefix = prompt_with_default("Title 前缀", "sdf_lhs_")
            rows = generate_lhs_rows(schema, count, args.seed, prefix)
            rows = [r for r in rows if r["Title"] not in existing_titles]
            if not rows:
                print("No new rows to create (all titles already exist).")
                continue
            print(f"Prepared {len(rows)} rows. Example:")
            print(json.dumps(rows[0], indent=2))
            if args.dry_run:
                print("Dry-run: nothing sent to NocoDB.")
                continue
            confirm = input("Create these rows in NocoDB? [y/N]: ").strip().lower()
            if confirm not in ("y", "yes"):
                print("Aborted by user.")
                continue
            batch = 50
            for i in range(0, len(rows), batch):
                _api_post(args, f"/api/v2/tables/{args.table_id}/records", rows[i:i + batch])
            print("Rows created.")
            records = fetch_records(args)
            existing_titles = {str(r.get("Title") or "") for r in records}
        elif choice == "5":
            rec_id = input("记录 Id: ").strip()
            if not rec_id.isdigit():
                print("Invalid Id.")
                continue
            values = {}
            for name in schema.get("params", {}).keys():
                raw = input(f"{name} 值(留空跳过): ").strip()
                if raw == "":
                    continue
                values[name] = float(raw)
            if not values:
                print("No values provided.")
                continue
            payload = {"Id": int(rec_id), "sdf_edit_json": build_sdf_edit(schema, values)}
            if args.dry_run:
                print(json.dumps(payload, indent=2))
                print("Dry-run: nothing sent to NocoDB.")
                continue
            _api_patch(args, f"/api/v2/tables/{args.table_id}/records", [payload])
            print("Record updated.")
            records = fetch_records(args)
        elif choice == "6":
            rec_id = input("记录 Id: ").strip()
            if not rec_id.isdigit():
                print("Invalid Id.")
                continue
            print("\n⚠️ 危险操作检测！")
            print("操作类型：删除记录")
            print(f"影响范围：NocoDB 记录 Id={rec_id}")
            print("风险评估：数据不可恢复，可能影响实验追溯")
            confirm = input("\n请确认是否继续？[需要明确的\"是\"、\"确认\"、\"继续\"]: ").strip()
            if confirm not in ("是", "确认", "继续"):
                print("Aborted by user.")
                continue
            if args.dry_run:
                print("Dry-run: nothing sent to NocoDB.")
                continue
            _api_delete(args, f"/api/v2/tables/{args.table_id}/records", [{"Id": int(rec_id)}])
            print("Record deleted (if API supports delete by Id).")
            records = fetch_records(args)
        elif choice == "0":
            break
        else:
            print("Invalid choice.")


def main() -> None:
    args = parse_args()
    schema = load_schema(args.schema)
    menu_loop(args, schema)


if __name__ == "__main__":
    main()
