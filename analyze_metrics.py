#!/usr/bin/env python3
"""
analyze_metrics.py

Analyze robot1_metrics_*.csv produced by task_metrics_logger.py.

Computes:
- baseline subtask durations (square/zigzag) from baseline_task_start/end
- mission duration from mission_start/end
- response time to human requests:
    response = interrupt_action_start - interrupt_received
- total interruption time:
    total_interrupt = interrupt_action_end - interrupt_received
- breakdown by urgency/action

Also saves simple matplotlib plots (separate figures, no subplots).
"""

import argparse
import csv
import math
import os
from collections import defaultdict
from statistics import mean, median

import matplotlib.pyplot as plt


def safe_float(x, default=None):
    try:
        return float(x)
    except Exception:
        return default


def load_rows(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            r["ts_unix"] = safe_float(r.get("ts_unix"), None)
            rows.append(r)
    # drop rows without timestamp
    rows = [r for r in rows if r["ts_unix"] is not None]
    rows.sort(key=lambda r: r["ts_unix"])
    return rows


def summarize(values):
    if not values:
        return None
    vals = sorted(values)
    return {
        "n": len(vals),
        "min": vals[0],
        "p25": vals[int(0.25 * (len(vals)-1))],
        "median": median(vals),
        "mean": mean(vals),
        "p75": vals[int(0.75 * (len(vals)-1))],
        "max": vals[-1],
    }


def print_summary(title, stats):
    if stats is None:
        print(f"{title}: (no data)")
        return
    print(
        f"{title}: n={stats['n']}  "
        f"min={stats['min']:.3f}s  p25={stats['p25']:.3f}s  "
        f"median={stats['median']:.3f}s  mean={stats['mean']:.3f}s  "
        f"p75={stats['p75']:.3f}s  max={stats['max']:.3f}s"
    )


def pair_intervals(rows, start_event, end_event, key_fields):
    """
    Pair starts and ends in time order per key_fields.
    Returns list of dicts: {key..., start_ts, end_ts, duration}
    """
    open_stack = defaultdict(list)
    intervals = []

    for r in rows:
        ev = r.get("event", "")
        ts = r["ts_unix"]
        key = tuple(r.get(k, "") for k in key_fields)

        if ev == start_event:
            open_stack[key].append(ts)

        elif ev == end_event:
            if open_stack[key]:
                start_ts = open_stack[key].pop(0)  # FIFO pairing
                intervals.append({
                    **{k: r.get(k, "") for k in key_fields},
                    "start_ts": start_ts,
                    "end_ts": ts,
                    "duration": ts - start_ts,
                })

    return intervals


def compute_interrupt_metrics(rows):
    """
    Uses task_id to match:
      interrupt_received
      interrupt_action_start
      interrupt_action_end
    """
    recv = {}
    start = {}
    end = {}
    meta = {}

    for r in rows:
        ev = r.get("event", "")
        task_id = r.get("task_id", "")
        if not task_id:
            continue
        ts = r["ts_unix"]

        if ev == "interrupt_received":
            recv[task_id] = ts
            meta[task_id] = (r.get("urgency", ""), r.get("action", ""))

        elif ev == "interrupt_action_start":
            start[task_id] = ts
            meta.setdefault(task_id, (r.get("urgency", ""), r.get("action", "")))

        elif ev == "interrupt_action_end":
            end[task_id] = ts
            meta.setdefault(task_id, (r.get("urgency", ""), r.get("action", "")))

    response_times = []
    total_interrupt_times = []
    by_urgency = defaultdict(list)
    by_action = defaultdict(list)
    by_pair = defaultdict(list)

    for task_id, t_recv in recv.items():
        if task_id in start:
            rt = start[task_id] - t_recv
            response_times.append(rt)
            urg, act = meta.get(task_id, ("", ""))
            by_urgency[urg].append(rt)
            by_action[act].append(rt)
            by_pair[(urg, act)].append(rt)

        if task_id in end:
            total = end[task_id] - t_recv
            total_interrupt_times.append(total)

    return {
        "response_times": response_times,
        "total_interrupt_times": total_interrupt_times,
        "response_by_urgency": by_urgency,
        "response_by_action": by_action,
        "response_by_pair": by_pair,
    }


def plot_box(data_map, title, ylabel, out_path):
    """
    data_map: dict[label] -> list[float]
    """
    labels = []
    data = []
    for k, v in data_map.items():
        if v:
            labels.append(str(k))
            data.append(v)

    if not data:
        return

    plt.figure()
    plt.boxplot(data, labels=labels, showfliers=True)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close()


def plot_bar(means_map, title, ylabel, out_path):
    labels = list(means_map.keys())
    vals = [means_map[k] for k in labels]
    if not vals:
        return

    plt.figure()
    plt.bar(labels, vals)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(out_path, dpi=160)
    plt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Path to robot1_metrics_*.csv")
    ap.add_argument("--outdir", default=None, help="Directory to write plots (default: alongside CSV)")
    args = ap.parse_args()

    csv_path = os.path.expanduser(args.csv)
    rows = load_rows(csv_path)

    if not rows:
        print("No rows found.")
        return

    outdir = args.outdir or os.path.dirname(os.path.abspath(csv_path))
    os.makedirs(outdir, exist_ok=True)

    print(f"Loaded {len(rows)} rows from {csv_path}")
    robot = rows[0].get("robot", "robot1")
    print(f"Robot: {robot}")
    print()

    # Mission duration
    mission_starts = [r["ts_unix"] for r in rows if r.get("event") == "mission_start"]
    mission_ends = [r["ts_unix"] for r in rows if r.get("event") == "mission_end"]
    if mission_starts and mission_ends:
        mission_duration = mission_ends[-1] - mission_starts[0]
        print(f"Mission duration (including interruptions): {mission_duration:.3f}s")
    else:
        print("Mission duration: missing mission_start or mission_end events")
    print()

    # Baseline task durations (square / zigzag)
    baseline_intervals = pair_intervals(
        rows,
        start_event="baseline_task_start",
        end_event="baseline_task_end",
        key_fields=["task_name"],
    )

    by_task = defaultdict(list)
    for itv in baseline_intervals:
        by_task[itv["task_name"]].append(itv["duration"])

    print("Baseline subtask durations:")
    for task_name in sorted(by_task.keys()):
        print_summary(f"  {task_name}", summarize(by_task[task_name]))
    print()

    # Human interrupt metrics
    im = compute_interrupt_metrics(rows)

    print("Human interrupt response time (action_start - received):")
    print_summary("  overall", summarize(im["response_times"]))
    for urg in sorted(im["response_by_urgency"].keys()):
        print_summary(f"  urgency={urg}", summarize(im["response_by_urgency"][urg]))
    for (urg, act) in sorted(im["response_by_pair"].keys()):
        print_summary(f"  urgency={urg}, action={act}", summarize(im["response_by_pair"][(urg, act)]))
    print()

    print("Total interruption time (action_end - received):")
    print_summary("  overall", summarize(im["total_interrupt_times"]))
    print()

    # Plots
    # 1) response time by urgency (box)
    plot_box(
        im["response_by_urgency"],
        title="Response time by urgency",
        ylabel="seconds",
        out_path=os.path.join(outdir, "response_time_by_urgency.png"),
    )

    # 2) baseline task duration means (bar)
    baseline_means = {k: mean(v) for k, v in by_task.items() if v}
    plot_bar(
        baseline_means,
        title="Mean baseline duration by task",
        ylabel="seconds",
        out_path=os.path.join(outdir, "baseline_mean_duration.png"),
    )

    # 3) response time by urgency+action (box)
    pair_map = {f"{k[0]}:{k[1]}": v for k, v in im["response_by_pair"].items() if v}
    plot_box(
        pair_map,
        title="Response time by urgency + action",
        ylabel="seconds",
        out_path=os.path.join(outdir, "response_time_by_urgency_action.png"),
    )

    print(f"Saved plots to: {outdir}")
    print("  - response_time_by_urgency.png")
    print("  - baseline_mean_duration.png")
    print("  - response_time_by_urgency_action.png")


if __name__ == "__main__":
    main()

