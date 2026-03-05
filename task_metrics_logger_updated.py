#!/usr/bin/env python3
"""
task_metrics_logger_updated.py

Single-CSV metrics logger for robot baseline subtasks + human interruptions.

The node listens to:
  1) Robot task event stream (e.g. /task_events)
  2) Human interruption tasks (e.g. /robot1/human_task)

CSV columns (ONLY these 10):

  1) robot_subtask_start_time
  2) robot_subtask_end_time
  3) human_interrupt_generation_ts
  4) robot_deferring_human_request_ms
  5) task_type
  6) attend_human_start_ts
  7) attend_human_end_ts
  8) robot_responding_to_human_request
  9) robot_total_task_start_time
 10) robot_total_task_end_time

Row write rules:
- Baseline row is written on: baseline_task_end
- Human interrupt row is written on: interrupt_action_end

Total task start/end:
- Start = mission_start if present; otherwise first event seen for that robot
- End   = updated on every event; finalized on mission_end if present

Pro Gamer Tip:
  Run the logger with:

  ros2 topic echo /task_events
  ros2 topic echo /robot1/human_task

  to verify that messages are being published correctly.

  If the CSV file remains empty, check that both topics
  are actively publishing messages.
"""

import argparse
import csv
import json
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Optional, Set, List

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


def normalize_task_type(urgency, priority) -> str:
    """
    Returns one of: "urgent", "non_urgent", "priority", or "".
    Heuristics:
      - If priority is truthy and not 0/"0"/"" -> "priority"
      - Else if urgency is a string containing 'urgent' -> urgent/non_urgent
      - Else if urgency is numeric: >=1 -> "urgent", 0 -> "non_urgent"
    """
    # priority wins
    if priority not in ("", None):
        # treat "0"/0 as NOT A priority
        if isinstance(priority, str) and priority.strip() != "" and priority.strip() != "0":
            return "priority"
        if isinstance(priority, (int, float)) and float(priority) != 0.0:
            return "priority"

    if urgency in ("", None):
        return ""

    if isinstance(urgency, str):
        u = urgency.strip().lower().replace("-", "").replace(" ", "")
        if "nonurgent" in u or "non_urgent" in u:
            return "non_urgent"
        if "urgent" in u:
            return "urgent"
        # unknown string
        return ""

    # numeric urgency
    try:
        u = int(urgency)
        return "urgent" if u >= 1 else "non_urgent"
    except Exception:
        return ""


def truthy_baseline(val) -> Optional[bool]:
    """
    Parse baseline_task which might be bool/int/str. Returns True/False/None.
    """
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


# Correlation state :3

@dataclass
class HumanInterrupt:
    task_id: str
    created_unix: float
    urgency: str = ""
    action: str = ""
    # optional: which robot it was targeted to, if this is known
    robot_hint: str = ""


@dataclass
class TaskRecord:
    # identifiers
    robot_id: str = ""
    task_id: str = ""

    # robot event timestamps
    subtask_start_unix: Optional[float] = None  # first subtask_start
    subtask_end_unix: Optional[float] = None    # last subtask_end
    task_received_unix: Optional[float] = None
    task_start_unix: Optional[float] = None
    task_end_unix: Optional[float] = None

    # fields to classify task
    urgency: str = ""
    priority: str = ""
    baseline_task: Optional[bool] = None

    # derived & bookkeeping
    is_human_task: bool = False
    finalized: bool = False


# Main node!!!

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

# 3 Optional identifiers just in case?
OPTIONAL_ID_COLUMNS = [
    "robot_id",
    "task_id",
    "action",
]


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

        # Task records indexed by (robot_id, task_id) and by task_id for convenience
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

    # Callbacks

    def on_human_task(self, msg: String) -> None:
        raw = msg.data
        try:
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"[human_task] Bad JSON: {e}. Raw: {raw}")
            return

        # created timestamp
        created_unix = safe_float(data.get("created_unix"), default=time.time())
        task_id = str(data.get("task_id") or f"human_{int(created_unix*1000)}")
        urgency = data.get("urgency", "")
        action = data.get("action", "")

        h = HumanInterrupt(
            task_id=task_id,
            created_unix=float(created_unix),
            urgency=str(urgency) if urgency is not None else "",
            action=str(action) if action is not None else "",
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
        task_id = str(data.get("task_id", "") or "")
        urgency = data.get("urgency", "")
        priority = data.get("priority", "")
        baseline_task = truthy_baseline(data.get("baseline_task", ""))

        # Update mission start/end per robot
        if robot_id:
            if robot_id not in self.robot_start_unix:
                self.robot_start_unix[robot_id] = float(ts_unix)
            # mission_start overrides
            if event == "mission_start":
                self.robot_start_unix[robot_id] = float(ts_unix)

            self.robot_end_unix[robot_id] = float(ts_unix)
            if event == "mission_end":
                self.robot_end_unix[robot_id] = float(ts_unix)

        # Ignore if no task_id; but still keep mission times
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
        if baseline_task is not None:
            rec.baseline_task = baseline_task

        # Determine if this looks like a human task
        # - explicit from human task topic (preferred)
        # - baseline_task False (Trey's user tasks)
        # - task_id prefix heuristic
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

        # If this is a human task and task_end is known, emit a row once (1).
        if rec.is_human_task and rec.task_end_unix is not None and not rec.finalized:
            self._finalize_human_task(rec, raw_event_json=raw if self.include_raw else None)

    # Row building

    def _finalize_human_task(self, rec: TaskRecord, raw_event_json: Optional[str]) -> None:
        # Fetch human interrupt (if seen)
        human = self.human_interrupts.get(rec.task_id)

        # timestamps in ISO strings (or blank)
        robot_subtask_start = unix_to_iso(rec.subtask_start_unix) if rec.subtask_start_unix else ""
        robot_subtask_end = unix_to_iso(rec.subtask_end_unix) if rec.subtask_end_unix else ""
        human_created = unix_to_iso(human.created_unix) if human else ""
        attending_start = unix_to_iso(rec.task_start_unix) if rec.task_start_unix else ""
        attending_end = unix_to_iso(rec.task_end_unix) if rec.task_end_unix else ""
        robot_responding = unix_to_iso(rec.task_received_unix) if rec.task_received_unix else ""

        # deferment (ms): attending_start - human_created
        defer_ms = ""
        if human and rec.task_start_unix is not None:
            defer_ms = int(round((rec.task_start_unix - human.created_unix) * 1000.0))

        # task type (urgent/non_urgent/priority)
        # prefer human urgency if robot did not provide
        urgency_for_type = rec.urgency if rec.urgency else (human.urgency if human else "")
        priority_for_type = rec.priority
        task_type = normalize_task_type(urgency_for_type, priority_for_type)

        # mission times
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
        help="Human task topic(s) (std_msgs/String JSON). Repeatable. Default: /robot1/human_task",
    )
    p.add_argument("--out", default=default_filename())
    p.add_argument("--include-ids", action="store_true", help="Add robot_id/task_id/action columns (in addition to the 10 categories).")
    p.add_argument("--include-raw", action="store_true", help="Add raw_event_json column for debugging.")
    args = p.parse_args()

    robot_topics = args.robot_topic if args.robot_topic else ["/task_events"]
    human_topics = args.human_topic if args.human_topic else ["/robot1/human_task"]

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
