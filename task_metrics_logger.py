#!/usr/bin/env python3
"""
task_metrics_logger.py

Subscribes to /robot1/task_events (std_msgs/String containing JSON) and logs rows to CSV.

Each event message is expected to look like:
{
  "ts_unix": 1234567890.12,
  "robot": "robot1",
  "event": "interrupt_received" | "interrupt_action_start" | ...,
  ... optional fields ...
}

This logger is intentionally simple and robust:
- It writes one row per event
- It flushes to disk so you don't lose data on shutdown
- It tolerates missing fields
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_COLUMNS = [
    "ts_unix",
    "ts_iso",
    "robot",
    "event",
    "task_id",
    "urgency",
    "action",
    "task_name",
]


class TaskMetricsLogger(Node):
    def __init__(self, topic: str, out_csv: str, include_raw: bool):
        super().__init__("task_metrics_logger")

        self.topic = topic
        self.out_csv = out_csv
        self.include_raw = include_raw

        self.columns = list(DEFAULT_COLUMNS)
        if self.include_raw:
            self.columns.append("raw_json")

        # Ensure output directory exists
        out_dir = os.path.dirname(os.path.abspath(self.out_csv))
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)

        self.f = open(self.out_csv, "w", newline="")
        self.writer = csv.DictWriter(self.f, fieldnames=self.columns)
        self.writer.writeheader()
        self.f.flush()

        self.sub = self.create_subscription(String, self.topic, self.on_event, 50)

        self.get_logger().info(f"Listening on: {self.topic}")
        self.get_logger().info(f"Writing CSV to: {self.out_csv}")
        self.get_logger().info(f"Columns: {', '.join(self.columns)}")

    def on_event(self, msg: String) -> None:
        raw = msg.data

        try:
            data = json.loads(raw)
            ts_unix = float(data.get("ts_unix", time.time()))
        except Exception as e:
            # If JSON fails, still log something useful
            self.get_logger().error(f"Bad JSON event: {e}. Raw: {raw}")
            data = {}
            ts_unix = time.time()

        ts_iso = datetime.fromtimestamp(ts_unix).isoformat()

        row = {
            "ts_unix": ts_unix,
            "ts_iso": ts_iso,
            "robot": data.get("robot", ""),
            "event": data.get("event", ""),
            "task_id": data.get("task_id", ""),
            "urgency": data.get("urgency", ""),
            "action": data.get("action", ""),
            "task_name": data.get("task_name", ""),
        }

        if self.include_raw:
            row["raw_json"] = raw

        self.writer.writerow(row)
        self.f.flush()

    def close(self) -> None:
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass


def default_filename() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/Desktop/robot1_metrics_{stamp}.csv")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/robot1/task_events")
    p.add_argument("--out", default=default_filename(), help="Output CSV path")
    p.add_argument("--include-raw", action="store_true", help="Add a raw_json column containing the original JSON string")
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

