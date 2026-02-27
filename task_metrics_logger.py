#!/usr/bin/env python3
"""
task_metrics_logger.py (professor-format metrics)

Subscribes to std_msgs/String JSON events and writes a "metrics-oriented" CSV with columns:

    Robot_id subtask start time
    Robot_id subtask end time
    Human interruption generation - timestamp
    Robot deferring human request (ms)
    Task type - urgent/nonurgent/priority
    Time stamp start attending human request
    Time stamp end attending human request
    Robot_id responding to human request
    Robot_id total task start time
    Robot_id total task end time

How it maps to events from interrupt_allocator_demo.py:
- baseline_task_start / baseline_task_end -> subtask start/end
- interrupt_received -> created_unix (human generation), urgency/priority
- interrupt_deferral -> deferred_ms
- interrupt_action_start / interrupt_action_end -> attending start/end
- mission_start / mission_end -> total task start/end

It writes rows on:
- baseline_task_end (a completed baseline subtask row)
- interrupt_action_end (a completed interrupt row)
- mission_end (a completed mission row)
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime
from typing import Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def iso(ts: Optional[float]) -> str:
    if ts is None:
        return ""
    try:
        return datetime.fromtimestamp(float(ts)).isoformat()
    except Exception:
        return ""


def fnum(x) -> str:
    if x is None:
        return ""
    try:
        return f"{float(x):.6f}"
    except Exception:
        return ""


DEFAULT_COLUMNS = [
    # core identifiers
    "robot_id",
    "row_type",  # baseline_subtask | interrupt | mission

    # baseline subtask metrics
    "subtask_name",
    "subtask_start_unix",
    "subtask_start_iso",
    "subtask_end_unix",
    "subtask_end_iso",

    # human interrupt metrics
    "human_task_id",
    "human_created_unix",     # "Human interruption generation - timestamp"
    "human_created_iso",
    "robot_received_unix",
    "robot_received_iso",
    "deferred_ms",            # "Robot deferring human request - ms"
    "task_type",              # urgent/nonurgent
    "priority",               # numeric if provided
    "movement",               # action or move_to etc.

    "attend_start_unix",      # "Time stamp start attending human request"
    "attend_start_iso",
    "attend_end_unix",        # "Time stamp end attending human request"
    "attend_end_iso",

    "responding_robot_id",    # "Robot_id responding to human request"

    # total task metrics (mission)
    "total_task_start_unix",
    "total_task_start_iso",
    "total_task_end_unix",
    "total_task_end_iso",

    # optional passthrough/debug
    "event",
    "ts_unix",
    "ts_iso",
]


class TaskMetricsLogger(Node):
    def __init__(self, topic: str, out_csv: str, include_raw: bool):
        super().__init__("task_metrics_logger_professor")

        self.topic = topic
        self.out_csv = out_csv
        self.include_raw = include_raw

        self.columns = list(DEFAULT_COLUMNS)
        if include_raw:
            self.columns.append("raw_json")

        out_dir = os.path.dirname(os.path.abspath(self.out_csv))
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)

        self.f = open(self.out_csv, "w", newline="")
        self.writer = csv.DictWriter(self.f, fieldnames=self.columns)
        self.writer.writeheader()
        self.f.flush()

        # ---- State (to join events) ----
        # Mission start per robot
        self.mission_start: Dict[str, Optional[float]] = {}

        # Baseline subtask start per robot: {robot: (task_name, start_ts)}
        self.subtask_open: Dict[str, Tuple[str, float]] = {}

        # Interrupt join state per robot & task_id
        # { (robot, task_id): {...} }
        self.interrupt_state: Dict[Tuple[str, str], Dict[str, Any]] = {}

        self.sub = self.create_subscription(String, self.topic, self.on_event, 200)

        self.get_logger().info(f"Listening on: {self.topic}")
        self.get_logger().info(f"Writing CSV to: {self.out_csv}")

    def write_row(self, row: Dict[str, Any]) -> None:
        if self.include_raw and "raw_json" not in row:
            row["raw_json"] = ""
        self.writer.writerow(row)
        self.f.flush()

    def base_row(self, robot_id: str, event: str, ts_unix: float) -> Dict[str, Any]:
        ms = self.mission_start.get(robot_id, None)
        return {
            "robot_id": robot_id,
            "row_type": "",
            "subtask_name": "",
            "subtask_start_unix": "",
            "subtask_start_iso": "",
            "subtask_end_unix": "",
            "subtask_end_iso": "",
            "human_task_id": "",
            "human_created_unix": "",
            "human_created_iso": "",
            "robot_received_unix": "",
            "robot_received_iso": "",
            "deferred_ms": "",
            "task_type": "",
            "priority": "",
            "movement": "",
            "attend_start_unix": "",
            "attend_start_iso": "",
            "attend_end_unix": "",
            "attend_end_iso": "",
            "responding_robot_id": "",
            "total_task_start_unix": fnum(ms),
            "total_task_start_iso": iso(ms),
            "total_task_end_unix": "",
            "total_task_end_iso": "",
            "event": event,
            "ts_unix": fnum(ts_unix),
            "ts_iso": iso(ts_unix),
        }

    def on_event(self, msg: String) -> None:
        raw = msg.data
        try:
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"Bad JSON event: {e}. Raw: {raw}")
            return

        event = str(data.get("event", ""))
        robot_id = str(data.get("robot", data.get("robot_id", ""))) or "unknown_robot"

        ts_unix = data.get("ts_unix", time.time())
        try:
            ts_unix = float(ts_unix)
        except Exception:
            ts_unix = time.time()

        # ---------- Mission total task ----------
        if event == "mission_start":
            self.mission_start[robot_id] = ts_unix
            # (optional) could write a row here, but professor mainly wants start/end known
            return

        if event == "mission_end":
            row = self.base_row(robot_id, event, ts_unix)
            row["row_type"] = "mission"
            row["total_task_end_unix"] = fnum(ts_unix)
            row["total_task_end_iso"] = iso(ts_unix)
            if self.include_raw:
                row["raw_json"] = raw
            self.write_row(row)
            return

        # ---------- Baseline subtask ----------
        if event == "baseline_task_start":
            task_name = str(data.get("task_name", "baseline_unknown"))
            self.subtask_open[robot_id] = (task_name, ts_unix)
            return

        if event == "baseline_task_end":
            task_name = str(data.get("task_name", "baseline_unknown"))
            start_name, start_ts = self.subtask_open.get(robot_id, (task_name, None))
            if start_ts is None:
                # no start recorded; still write end-only row
                start_ts = None
                start_name = task_name

            row = self.base_row(robot_id, event, ts_unix)
            row["row_type"] = "baseline_subtask"
            row["subtask_name"] = start_name
            row["subtask_start_unix"] = fnum(start_ts)
            row["subtask_start_iso"] = iso(start_ts)
            row["subtask_end_unix"] = fnum(ts_unix)
            row["subtask_end_iso"] = iso(ts_unix)

            # clear open subtask
            if robot_id in self.subtask_open:
                del self.subtask_open[robot_id]

            if self.include_raw:
                row["raw_json"] = raw
            self.write_row(row)
            return

        # ---------- Interrupt join ----------
        # Task id key (must exist for join)
        task_id = data.get("task_id", None)
        if task_id is not None:
            task_id = str(task_id)

        key = (robot_id, task_id) if task_id else None

        if event == "interrupt_received" and key is not None:
            st = self.interrupt_state.get(key, {})
            st["robot_id"] = robot_id
            st["task_id"] = task_id
            st["task_type"] = data.get("urgency", data.get("urgency_class", ""))
            st["priority"] = data.get("priority", data.get("urgency_num", data.get("urgency_num", "")))
            st["human_created_unix"] = data.get("created_unix", data.get("created_unix", data.get("created_unix", "")))
            st["robot_received_unix"] = data.get("received_unix", ts_unix)
            # movement/action
            st["movement"] = data.get("action", data.get("movement", ""))
            self.interrupt_state[key] = st
            return

        if event == "interrupt_deferral" and key is not None:
            st = self.interrupt_state.get(key, {})
            st["robot_id"] = robot_id
            st["task_id"] = task_id
            st["deferred_ms"] = data.get("deferred_ms", "")
            # also accept these if present
            st["task_type"] = data.get("urgency", st.get("task_type", ""))
            st["priority"] = data.get("priority", st.get("priority", ""))
            st["human_created_unix"] = data.get("created_unix", st.get("human_created_unix", ""))
            st["robot_received_unix"] = data.get("received_unix", st.get("robot_received_unix", ""))
            st["movement"] = data.get("action", st.get("movement", ""))
            self.interrupt_state[key] = st
            return

        if event == "interrupt_action_start" and key is not None:
            st = self.interrupt_state.get(key, {})
            st["robot_id"] = robot_id
            st["task_id"] = task_id
            st["attend_start_unix"] = ts_unix
            st["task_type"] = data.get("urgency", st.get("task_type", ""))
            st["priority"] = data.get("priority", st.get("priority", ""))
            st["movement"] = data.get("action", st.get("movement", ""))
            self.interrupt_state[key] = st
            return

        if event == "interrupt_action_end" and key is not None:
            st = self.interrupt_state.get(key, {})
            st["robot_id"] = robot_id
            st["task_id"] = task_id
            st["attend_end_unix"] = ts_unix
            st["task_type"] = data.get("urgency", st.get("task_type", ""))
            st["priority"] = data.get("priority", st.get("priority", ""))
            st["movement"] = data.get("action", st.get("movement", ""))

            # Build one completed interrupt row
            row = self.base_row(robot_id, event, ts_unix)
            row["row_type"] = "interrupt"

            hc = st.get("human_created_unix", "")
            rr = st.get("robot_received_unix", "")
            ds = st.get("deferred_ms", "")

            # normalize numeric-ish fields to strings
            try:
                hc = float(hc) if hc not in ("", None) else None
            except Exception:
                hc = None
            try:
                rr = float(rr) if rr not in ("", None) else None
            except Exception:
                rr = None
            try:
                ds = float(ds) if ds not in ("", None) else None
            except Exception:
                ds = None

            row["human_task_id"] = task_id
            row["human_created_unix"] = fnum(hc)
            row["human_created_iso"] = iso(hc)
            row["robot_received_unix"] = fnum(rr)
            row["robot_received_iso"] = iso(rr)
            row["deferred_ms"] = f"{ds:.3f}" if ds is not None else ""

            row["task_type"] = str(st.get("task_type", ""))
            row["priority"] = fnum(st.get("priority", None))
            row["movement"] = str(st.get("movement", ""))

            astart = st.get("attend_start_unix", None)
            try:
                astart = float(astart) if astart not in ("", None) else None
            except Exception:
                astart = None
            row["attend_start_unix"] = fnum(astart)
            row["attend_start_iso"] = iso(astart)
            row["attend_end_unix"] = fnum(ts_unix)
            row["attend_end_iso"] = iso(ts_unix)

            row["responding_robot_id"] = robot_id

            if self.include_raw:
                row["raw_json"] = raw

            self.write_row(row)

            # clear interrupt state
            if key in self.interrupt_state:
                del self.interrupt_state[key]
            return

        # Other events: ignore (safety_recovery, safety_stop, etc.)
        return

    def close(self) -> None:
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass


def default_filename() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/Desktop/prof_metrics_{stamp}.csv")


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--topic",
        default="/robot1/task_events",
        help="Single robot: /robot1/task_events   Team: /team/task_events",
    )
    p.add_argument("--out", default=default_filename(), help="Output CSV path")
    p.add_argument("--include-raw", action="store_true", help="Add raw_json column containing original JSON")
    args = p.parse_args()

    rclpy.init()
    node = TaskMetricsLogger(topic=args.topic, out_csv=args.out, include_raw=args.include_raw)
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
