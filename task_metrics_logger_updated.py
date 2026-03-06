#!/usr/bin/env python3
"""
task_metrics_logger.py

Single-CSV metrics logger for robot baseline subtasks + human interruptions.

This version is backward-compatible with the JSON human-task format and it can 
also handle Lorence's keyboard publisher format:

  std_msgs/String on topic 'userTopic' with msg.data like:
    "interruption server all <urgency_num> <priority_num> <InterruptID> moveto ..."

Key fix vs prior versions:
- Task classification uses an explicit "is_priority" / "priority_interrupt" flag when present.
- If the flag is absent, we fall back to a heuristic:
    priority in {1,3,4,5,...} => "priority"
    priority == 2 => urgent/non_urgent by urgency
This matches your note that urgent/non-urgent always comes through with priority=2.

CSV columns (ONLY these 10):

  1) robot_subtask_start_time
  2) robot_subtask_end_time
  3) human_interruption_generation_timestamp
  4) robot_deferring_human_request_ms
  5) task_type
  6) timestamp_start_attending_human_request
  7) timestamp_end_attending_human_request
  8) robot_responding_to_human_request
  9) robot_total_task_start_time
 10) robot_total_task_end_time
"""

import argparse
import csv
import json
import os
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional, Set, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String



# Da Helpers

def unix_to_iso(ts_unix: float) -> str:
    return datetime.fromtimestamp(ts_unix).isoformat()


def safe_float(x, default=None):
    try:
        return float(x)
    except Exception:
        return default


def truthy_flag(val) -> Optional[bool]:
    """Parse a flag which might be bool/int/str. Returns True/False/None."""
    if val is None or val == "":
        return None
    if isinstance(val, bool):
        return val
    if isinstance(val, (int, float)):
        return bool(val)
    if isinstance(val, str):
        v = val.strip().lower()
        if v in ("true", "t", "1", "yes", "y"):
            return True
        if v in ("false", "f", "0", "no", "n"):
            return False
    return None


def _normalize_urgency_value(urgency) -> Optional[int]:
    """Return urgency as int if possible (non_urgent=0, urgent>=1)."""
    if urgency in ("", None):
        return None
    if isinstance(urgency, (int, float)):
        return int(urgency)
    if isinstance(urgency, str):
        u = urgency.strip().lower().replace("-", "").replace(" ", "")
        if u == "":
            return None
        if "nonurgent" in u:
            return 0
        if "urgent" in u:
            return 1
        # numeric string
        try:
            return int(u)
        except Exception:
            return None
    try:
        return int(urgency)
    except Exception:
        return None


def _normalize_priority_value(priority) -> Optional[int]:
    if priority in ("", None):
        return None
    if isinstance(priority, (int, float)):
        return int(priority)
    if isinstance(priority, str):
        p = priority.strip()
        if p == "":
            return None
        try:
            return int(p)
        except Exception:
            return None
    try:
        return int(priority)
    except Exception:
        return None


def normalize_task_type(urgency, priority, is_priority: Optional[bool]) -> str:
    """
    Decide between: "urgent", "non_urgent", "priority", or "".

    Precedence:
      1) Explicit is_priority flag, if provided.
      2) Otherwise heuristic:
          - if priority is present and priority != 2 => "priority"
          - else use urgency (0 => non_urgent, >=1 => urgent)
    """
    if is_priority is True:
        return "priority"
    if is_priority is False:
        u = _normalize_urgency_value(urgency)
        if u is None:
            return ""
        return "urgent" if u >= 1 else "non_urgent"

    # Heuristic fallback (no flag)
    p = _normalize_priority_value(priority)
    if p is not None and p != 2:
        return "priority"

    u = _normalize_urgency_value(urgency)
    if u is None:
        return ""
    return "urgent" if u >= 1 else "non_urgent"


def parse_forryan_user_topic(raw: str) -> Optional[Tuple[str, str, str]]:
    """
    Parse Lorence keyboard publisher message.

    Expected:
      interruption server all <urgency_num> <priority_num> <InterruptID> <action...>

    Returns (urgency_str, priority_str, task_id_str) or None.
    """
    if not isinstance(raw, str):
        return None
    parts = raw.strip().split()
    if len(parts) < 6:
        return None
    if parts[0].lower() != "interruption":
        return None
    # parts[3]=urgency, parts[4]=priority, parts[5]=id  (given the scripts in ForRyan(1).zip)
    try:
        urgency = parts[3]
        priority = parts[4]
        task_id = str(parts[5])
        return urgency, priority, task_id
    except Exception:
        return None

# Correlation state :3

@dataclass
class HumanInterrupt:
    task_id: str
    created_unix: float
    urgency: str = ""
    priority: str = ""
    is_priority: Optional[bool] = None
    action: str = ""
    robot_hint: str = ""


@dataclass
class TaskRecord:
    robot_id: str = ""
    task_id: str = ""

    # robot event timestamps
    subtask_start_unix: Optional[float] = None
    subtask_end_unix: Optional[float] = None
    task_received_unix: Optional[float] = None
    task_start_unix: Optional[float] = None
    task_end_unix: Optional[float] = None

    # fields to classify task
    urgency: str = ""
    priority: str = ""
    is_priority: Optional[bool] = None
    baseline_task: Optional[bool] = None

    # bookkeeping
    is_human_task: bool = False
    finalized: bool = False


DEFAULT_COLUMNS_10 = [
    "robot_subtask_start_time",
    "robot_subtask_end_time",
    "human_interruption_generation_timestamp",
    "robot_deferring_human_request_ms",
    "task_type",
    "timestamp_start_attending_human_request",
    "timestamp_end_attending_human_request",
    "robot_responding_to_human_request",
    "robot_total_task_start_time",
    "robot_total_task_end_time",
]

OPTIONAL_ID_COLUMNS = ["robot_id", "task_id", "action"]


class TaskMetricsLogger(Node):
    def __init__(
        self,
        robot_topics: List[str],
        human_topics: List[str],
        out_csv: str,
        include_ids: bool,
        include_raw: bool,
    ):
        super().__init__("task_metrics_logger")

        self.robot_topics = robot_topics
        self.human_topics = human_topics
        self.out_csv = out_csv
        self.include_ids = include_ids
        self.include_raw = include_raw

        self.columns = list(DEFAULT_COLUMNS_10)
        if self.include_ids:
            self.columns = OPTIONAL_ID_COLUMNS + self.columns
        if self.include_raw:
            self.columns.append("raw_event_json")

        # Mission (total task) start/end per robot
        self.robot_start_unix: Dict[str, float] = {}
        self.robot_end_unix: Dict[str, float] = {}

        # Human interrupts indexed by task_id
        self.human_interrupts: Dict[str, HumanInterrupt] = {}
        self.human_task_ids: Set[str] = set()

        # Task records indexed by robot_id::task_id
        self.records_by_key: Dict[str, TaskRecord] = {}

        # Ensure output directory exists
        out_dir = os.path.dirname(os.path.abspath(self.out_csv))
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)

        self.f = open(self.out_csv, "w", newline="")
        self.writer = csv.DictWriter(self.f, fieldnames=self.columns)
        self.writer.writeheader()
        self.f.flush()

        # Subscriptions
        self.robot_subs = []
        for t in self.robot_topics:
            self.robot_subs.append(self.create_subscription(String, t, self.on_robot_event, 200))

        self.human_subs = []
        for t in self.human_topics:
            self.human_subs.append(self.create_subscription(String, t, self.on_human_task, 200))

        self.get_logger().info(f"Robot event topics: {', '.join(self.robot_topics)}")
        self.get_logger().info(f"Human task topics: {', '.join(self.human_topics)}")
        self.get_logger().info(f"Writing CSV to: {self.out_csv}")

    # Da Callbacks

    def on_human_task(self, msg: String) -> None:
        raw = msg.data

        # First try JSON (your existing pipeline)
        data = None
        try:
            data = json.loads(raw)
        except Exception:
            data = None

        if isinstance(data, dict):
            created_unix = safe_float(data.get("created_unix"), default=time.time())
            task_id = str(data.get("task_id") or f"human_{int(created_unix*1000)}")
            urgency = data.get("urgency", "")
            priority = data.get("priority", "")
            action = data.get("action", "")

            # Accept several possible flag names
            is_priority = (
                truthy_flag(data.get("is_priority"))
                or truthy_flag(data.get("priority_interrupt"))
                or truthy_flag(data.get("is_priority_task"))
            )
            # If an explicit urgency flag exists, prefer it as "not priority"
            urgency_interrupt = truthy_flag(data.get("urgency_interrupt"))
            if urgency_interrupt is True:
                is_priority = False

            h = HumanInterrupt(
                task_id=task_id,
                created_unix=float(created_unix),
                urgency=str(urgency) if urgency is not None else "",
                priority=str(priority) if priority is not None else "",
                is_priority=is_priority,
                action=str(action) if action is not None else "",
            )
            self.human_interrupts[task_id] = h
            self.human_task_ids.add(task_id)
            return

        # Fallback: parse ForRyan(1).zip userTopic string
        parsed = parse_forryan_user_topic(raw)
        if parsed is None:
            self.get_logger().warn(f"[human_task] Unrecognized format (not JSON): {raw}")
            return

        urgency_s, priority_s, short_id = parsed
        created_unix = time.time()

        # Make task_id stable and distinct from robot task_ids if those are UUID-like
        task_id = f"user_{short_id}"

        # Lorence publisher doesn't include an explicit flag.
        # Infer priority tasks as: priority != 2
        pr_i = _normalize_priority_value(priority_s)
        inferred_is_priority = (pr_i is not None and pr_i != 2)

        # action payload = everything after the id token (index 6 onward)
        parts = raw.strip().split()
        action = " ".join(parts[6:]) if len(parts) > 6 else ""

        h = HumanInterrupt(
            task_id=task_id,
            created_unix=float(created_unix),
            urgency=str(urgency_s),
            priority=str(priority_s),
            is_priority=inferred_is_priority,
            action=action,
        )
        self.human_interrupts[task_id] = h
        self.human_task_ids.add(task_id)

    def on_robot_event(self, msg: String) -> None:
        raw = msg.data
        try:
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"[robot_event] Bad JSON: {e}. Raw: {raw}")
            return

        ts_unix = safe_float(data.get("ts_unix"), default=time.time())
        if ts_unix is None:
            ts_unix = time.time()

        robot_id = data.get("robot") or data.get("robot_id") or ""
        event = data.get("event", "")
        task_id_raw = str(data.get("task_id", "") or "")

        # Some pipelines may publish user task_ids without the "user_" prefix.
        # If we saw a human interrupt "user_<n>", accept matches to "<n>" too.
        task_id = task_id_raw
        if task_id_raw and f"user_{task_id_raw}" in self.human_task_ids:
            task_id = f"user_{task_id_raw}"

        urgency = data.get("urgency", "")
        priority = data.get("priority", "")
        baseline_task = truthy_flag(data.get("baseline_task", ""))

        # flags (newer pipeline)
        is_priority = (
            truthy_flag(data.get("is_priority"))
            or truthy_flag(data.get("priority_interrupt"))
            or truthy_flag(data.get("is_priority_task"))
        )
        urgency_interrupt = truthy_flag(data.get("urgency_interrupt"))
        if urgency_interrupt is True:
            is_priority = False

        # Update mission start/end per robot
        if robot_id:
            if robot_id not in self.robot_start_unix:
                self.robot_start_unix[robot_id] = float(ts_unix)
            if event == "mission_start":
                self.robot_start_unix[robot_id] = float(ts_unix)

            self.robot_end_unix[robot_id] = float(ts_unix)
            if event == "mission_end":
                self.robot_end_unix[robot_id] = float(ts_unix)

        if not task_id:
            return

        key = f"{robot_id}::{task_id}"
        rec = self.records_by_key.get(key)
        if rec is None:
            rec = TaskRecord(robot_id=robot_id, task_id=task_id)
            self.records_by_key[key] = rec

        # Update classification fields
        if urgency not in ("", None):
            rec.urgency = str(urgency)
        if priority not in ("", None):
            rec.priority = str(priority)
        if is_priority is not None:
            rec.is_priority = is_priority
        if baseline_task is not None:
            rec.baseline_task = baseline_task

        # Determine if this looks like a human task
        if task_id in self.human_task_ids:
            rec.is_human_task = True
        elif rec.baseline_task is False:
            rec.is_human_task = True
        elif task_id.lower().startswith(("human_", "interrupt_", "usr_", "user_")):
            rec.is_human_task = True

        # Update event timestamps
        if event == "subtask_start":
            if rec.subtask_start_unix is None:
                rec.subtask_start_unix = float(ts_unix)
        elif event == "subtask_end":
            rec.subtask_end_unix = float(ts_unix)
        elif event == "task_received":
            if rec.task_received_unix is None:
                rec.task_received_unix = float(ts_unix)
        elif event == "task_start":
            if rec.task_start_unix is None:
                rec.task_start_unix = float(ts_unix)
        elif event == "task_end":
            rec.task_end_unix = float(ts_unix)

        # Emit a row once we have task_end for a human task
        if rec.is_human_task and rec.task_end_unix is not None and not rec.finalized:
            self._finalize_human_task(rec, raw_event_json=raw if self.include_raw else None)

    # Row building

    def _finalize_human_task(self, rec: TaskRecord, raw_event_json: Optional[str]) -> None:
        human = self.human_interrupts.get(rec.task_id)

        robot_subtask_start = unix_to_iso(rec.subtask_start_unix) if rec.subtask_start_unix else ""
        robot_subtask_end = unix_to_iso(rec.subtask_end_unix) if rec.subtask_end_unix else ""
        human_created = unix_to_iso(human.created_unix) if human else ""
        attending_start = unix_to_iso(rec.task_start_unix) if rec.task_start_unix else ""
        attending_end = unix_to_iso(rec.task_end_unix) if rec.task_end_unix else ""
        robot_responding = unix_to_iso(rec.task_received_unix) if rec.task_received_unix else ""

        defer_ms = ""
        if human and rec.task_start_unix is not None:
            defer_ms = int(round((rec.task_start_unix - human.created_unix) * 1000.0))

        # task type
        urgency_for_type = rec.urgency if rec.urgency else (human.urgency if human else "")
        priority_for_type = rec.priority if rec.priority else (human.priority if human else "")
        is_priority_for_type = rec.is_priority if rec.is_priority is not None else (human.is_priority if human else None)

        task_type = normalize_task_type(urgency_for_type, priority_for_type, is_priority_for_type)

        mission_start_iso = unix_to_iso(self.robot_start_unix.get(rec.robot_id, 0.0)) if rec.robot_id in self.robot_start_unix else ""
        mission_end_iso = unix_to_iso(self.robot_end_unix.get(rec.robot_id, 0.0)) if rec.robot_id in self.robot_end_unix else ""

        row = {
            "robot_subtask_start_time": robot_subtask_start,
            "robot_subtask_end_time": robot_subtask_end,
            "human_interruption_generation_timestamp": human_created,
            "robot_deferring_human_request_ms": defer_ms,
            "task_type": task_type,
            "timestamp_start_attending_human_request": attending_start,
            "timestamp_end_attending_human_request": attending_end,
            "robot_responding_to_human_request": robot_responding,
            "robot_total_task_start_time": mission_start_iso,
            "robot_total_task_end_time": mission_end_iso,
        }

        if self.include_ids:
            row = {
                "robot_id": rec.robot_id,
                "task_id": rec.task_id,
                "action": (human.action if human else ""),
                **row,
            }

        if self.include_raw and raw_event_json is not None:
            row["raw_event_json"] = raw_event_json

        self.writer.writerow(row)
        self.f.flush()
        rec.finalized = True

    def close(self) -> None:
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass


def default_filename() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/Desktop/task_metrics_{stamp}.csv")


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--robot-topic",
        action="append",
        default=[],
        help="Robot event topic(s) (std_msgs/String JSON). Repeatable. Default: /task_events",
    )
    p.add_argument(
        "--human-topic",
        action="append",
        default=[],
        help="Human task topic(s). Can be JSON (/robot1/human_task) and/or ForRyan keyboard publisher (userTopic). Repeatable.",
    )
    p.add_argument("--out", default=default_filename())
    p.add_argument("--include-ids", action="store_true", help="Add robot_id/task_id/action columns (in addition to the 10 categories).")
    p.add_argument("--include-raw", action="store_true", help="Add raw_event_json column for debugging.")
    args = p.parse_args()

    robot_topics = args.robot_topic if args.robot_topic else ["/task_events"]
    # Default to both, so you don't have to remember which pipeline you're using.
    human_topics = args.human_topic if args.human_topic else ["/robot1/human_task", "userTopic"]

    rclpy.init()
    node = TaskMetricsLogger(
        robot_topics=robot_topics,
        human_topics=human_topics,
        out_csv=args.out,
        include_ids=args.include_ids,
        include_raw=args.include_raw,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
