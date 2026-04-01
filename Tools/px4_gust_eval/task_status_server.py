#!/usr/bin/env python3
"""
Lightweight HTTP status server for long-running gust evaluation jobs.
"""

from __future__ import annotations

import html
import json
import os
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional
from urllib.parse import urlsplit


TERMINAL_STATES = {"completed", "failed"}


@dataclass
class TaskStatusTracker:
    """Thread-safe task progress tracker shared with the HTTP server."""

    suite_name: str = "gust_suite"
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _tasks: List[Dict[str, str]] = field(default_factory=list, init=False, repr=False)
    _states: Dict[str, str] = field(default_factory=dict, init=False, repr=False)
    _errors: Dict[str, str] = field(default_factory=dict, init=False, repr=False)
    _started_at: float = field(default_factory=time.time, init=False, repr=False)
    _updated_at: float = field(default_factory=time.time, init=False, repr=False)
    _finished_at: Optional[float] = field(default=None, init=False, repr=False)

    def set_tasks(self, suite_name: str, tasks: List[Dict[str, Any]]) -> None:
        with self._lock:
            self.suite_name = suite_name or "gust_suite"
            self._tasks = []
            self._states = {}
            self._errors = {}
            self._started_at = time.time()
            self._updated_at = self._started_at
            self._finished_at = None

            for idx, task in enumerate(tasks):
                test_id = str(task.get("test_id") or f"task_{idx}")
                description = str(task.get("description") or "")
                self._tasks.append({
                    "test_id": test_id,
                    "description": description,
                })
                self._states[test_id] = "pending"

    def mark_running(self, test_id: str) -> None:
        with self._lock:
            self._states[test_id] = "running"
            self._errors.pop(test_id, None)
            self._updated_at = time.time()

    def mark_finished(self, test_id: str, success: bool, error: Optional[str] = None) -> None:
        with self._lock:
            self._states[test_id] = "completed" if success else "failed"
            if error:
                self._errors[test_id] = str(error)
            else:
                self._errors.pop(test_id, None)
            self._updated_at = time.time()

    def mark_suite_finished(self) -> None:
        with self._lock:
            self._finished_at = time.time()
            self._updated_at = self._finished_at

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            items = []
            running = 0
            completed = 0
            passed = 0
            failed = 0

            for task in self._tasks:
                test_id = task["test_id"]
                state = self._states.get(test_id, "pending")
                if state == "running":
                    running += 1
                if state in TERMINAL_STATES:
                    completed += 1
                if state == "completed":
                    passed += 1
                if state == "failed":
                    failed += 1
                item = {
                    "test_id": test_id,
                    "description": task["description"],
                    "status": state,
                }
                error = self._errors.get(test_id)
                if error:
                    item["error"] = error
                items.append(item)

            total = len(self._tasks)
            pending = total - running - completed
            finished = self._finished_at is not None
            return {
                "suite_name": self.suite_name,
                "status": "finished" if finished else "running",
                "total_tasks": total,
                "running_tasks": running,
                "completed_tasks": completed,
                "pending_tasks": pending,
                "passed_tasks": passed,
                "failed_tasks": failed,
                "started_at": self._started_at,
                "updated_at": self._updated_at,
                "finished_at": self._finished_at,
                "tasks": items,
            }


class _StatusHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class TaskStatusServer:
    """Small HTML/JSON status server used as container liveness surface."""

    def __init__(
        self,
        tracker: TaskStatusTracker,
        host: Optional[str] = None,
        port: Optional[int] = None,
        logger: Any = None,
    ) -> None:
        self.tracker = tracker
        self.host = host or os.getenv("TASK_STATUS_HOST", "0.0.0.0")
        self.port = self._resolve_port(port)
        self.logger = logger
        self._httpd: Optional[_StatusHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def _resolve_port(self, port: Optional[int]) -> int:
        if port is not None:
            return int(port)
        raw_port = os.getenv("TASK_STATUS_PORT") or os.getenv("PORT") or "8080"
        try:
            return int(raw_port)
        except ValueError:
            return 8080

    def start(self) -> bool:
        handler = self._build_handler()
        try:
            self._httpd = _StatusHTTPServer((self.host, self.port), handler)
        except OSError as exc:
            if self.logger:
                self.logger.warning(f"Task status server failed to bind {self.host}:{self.port}: {exc}")
            return False
        self.port = int(self._httpd.server_port)

        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="task-status-server",
            daemon=True,
        )
        self._thread.start()
        if self.logger:
            self.logger.info(
                f"Task status server listening on http://{self.host}:{self.port} "
                f"(health: /healthz, json: /status)"
            )
        return True

    def stop(self) -> None:
        if self._httpd is None:
            return

        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        finally:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=5.0)
            self._httpd = None
            self._thread = None
            if self.logger:
                self.logger.info("Task status server stopped")

    def _build_handler(self) -> type[BaseHTTPRequestHandler]:
        tracker = self.tracker

        class StatusHandler(BaseHTTPRequestHandler):
            server_version = "TaskStatusHTTP/1.0"

            def do_GET(self) -> None:
                path = urlsplit(self.path).path
                snapshot = tracker.snapshot()
                if path in ("", "/"):
                    self._send_response(self._render_html(snapshot), "text/html; charset=utf-8")
                    return
                if path == "/status":
                    body = json.dumps(snapshot, ensure_ascii=False, indent=2).encode("utf-8")
                    self._send_response(body, "application/json; charset=utf-8")
                    return
                if path in ("/healthz", "/readyz", "/livez"):
                    body = b"ok\n"
                    self._send_response(body, "text/plain; charset=utf-8")
                    return
                self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

            def log_message(self, format: str, *args: Any) -> None:
                return

            def _send_response(self, body: bytes, content_type: str) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _render_html(self, snapshot: Dict[str, Any]) -> bytes:
                cards = [
                    ("Running", snapshot["running_tasks"]),
                    ("Completed", snapshot["completed_tasks"]),
                    ("Pending", snapshot["pending_tasks"]),
                    ("Total", snapshot["total_tasks"]),
                ]
                card_html = "".join(
                    (
                        "<div class='card'>"
                        f"<div class='label'>{html.escape(label)}</div>"
                        f"<div class='value'>{value}</div>"
                        "</div>"
                    )
                    for label, value in cards
                )

                rows = []
                for task in snapshot["tasks"]:
                    status = task["status"]
                    error = task.get("error", "")
                    rows.append(
                        "<tr>"
                        f"<td>{html.escape(task['test_id'])}</td>"
                        f"<td>{html.escape(task['description'])}</td>"
                        f"<td><span class='status status-{html.escape(status)}'>{html.escape(status)}</span></td>"
                        f"<td>{html.escape(error)}</td>"
                        "</tr>"
                    )

                body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5">
  <title>{html.escape(snapshot["suite_name"])} Status</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f3f6f8;
      --panel: #ffffff;
      --text: #17212b;
      --muted: #62707d;
      --line: #d7dee4;
      --accent: #0f766e;
      --warn: #b45309;
      --ok: #166534;
      --fail: #b91c1c;
      --run: #1d4ed8;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif;
      background: linear-gradient(180deg, #eef5f4 0%, var(--bg) 100%);
      color: var(--text);
    }}
    main {{
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px 16px 40px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    p {{
      margin: 0;
      color: var(--muted);
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
      margin: 20px 0 24px;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 16px;
      box-shadow: 0 8px 20px rgba(15, 23, 42, 0.05);
    }}
    .label {{
      color: var(--muted);
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
    }}
    .value {{
      font-size: 34px;
      font-weight: 700;
      margin-top: 6px;
    }}
    .panel {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 16px;
      overflow: hidden;
      box-shadow: 0 8px 20px rgba(15, 23, 42, 0.05);
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
    }}
    th, td {{
      padding: 12px 14px;
      border-bottom: 1px solid var(--line);
      text-align: left;
      vertical-align: top;
      font-size: 14px;
    }}
    th {{
      background: #f8fafb;
      color: var(--muted);
      font-weight: 600;
    }}
    tr:last-child td {{
      border-bottom: none;
    }}
    .status {{
      display: inline-block;
      border-radius: 999px;
      padding: 4px 10px;
      font-size: 12px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .status-pending {{
      background: #f1f5f9;
      color: #475569;
    }}
    .status-running {{
      background: #dbeafe;
      color: var(--run);
    }}
    .status-completed {{
      background: #dcfce7;
      color: var(--ok);
    }}
    .status-failed {{
      background: #fee2e2;
      color: var(--fail);
    }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
      margin-bottom: 16px;
      color: var(--muted);
      font-size: 14px;
    }}
    .links {{
      margin-top: 12px;
      font-size: 14px;
    }}
    a {{
      color: var(--accent);
      text-decoration: none;
    }}
    a:hover {{
      text-decoration: underline;
    }}
    @media (max-width: 700px) {{
      th:nth-child(2),
      td:nth-child(2),
      th:nth-child(4),
      td:nth-child(4) {{
        display: none;
      }}
    }}
  </style>
</head>
<body>
  <main>
    <h1>{html.escape(snapshot["suite_name"])}</h1>
    <p>Current status: {html.escape(snapshot["status"])}</p>
    <div class="grid">{card_html}</div>
    <div class="meta">
      <span>Passed: {snapshot["passed_tasks"]}</span>
      <span>Failed: {snapshot["failed_tasks"]}</span>
      <span>Last Update: {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(snapshot["updated_at"]))}</span>
    </div>
    <div class="panel">
      <table>
        <thead>
          <tr>
            <th>Task</th>
            <th>Description</th>
            <th>Status</th>
            <th>Error</th>
          </tr>
        </thead>
        <tbody>
          {''.join(rows)}
        </tbody>
      </table>
    </div>
    <div class="links">
      <a href="/healthz">/healthz</a>
      <a href="/status">/status</a>
    </div>
  </main>
</body>
</html>
"""
                return body.encode("utf-8")

        return StatusHandler
