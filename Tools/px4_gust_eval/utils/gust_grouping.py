from __future__ import annotations

from collections import defaultdict
from typing import Dict, List


def detect_task_type(test_config: Dict) -> str:
    """Detect task type from test_id pattern."""
    test_id = test_config.get("test_id", "")

    # Use test_id pattern to detect type
    if "_z_" in test_id:
        return "Vertical (Z)"
    if "_y_" in test_id:
        return "Horizontal (Y)"
    if "gust_lvl_" in test_id:
        return "Horizontal (X)"

    return "Unknown"


def group_tests_by_type(tests: List[Dict]) -> Dict[str, List[Dict]]:
    """Group tests by their type (wind direction)."""
    groups = defaultdict(list)
    for test in tests:
        task_type = detect_task_type(test)
        groups[task_type].append(test)
    return dict(groups)


def wind_axis_from_task_type(task_type: str) -> str:
    if "(X)" in task_type:
        return "x"
    if "(Y)" in task_type:
        return "y"
    if "(Z)" in task_type:
        return "z"
    return "unknown"
