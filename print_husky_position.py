#!/usr/bin/env python3

import subprocess
import time

MODEL_NAME = "husky"

def get_pose():
    try:
        # Call `gz model` to get pose: x y z roll pitch yaw
        out = subprocess.check_output(
            ["gz", "model", "-m", MODEL_NAME, "-p"],
            stderr=subprocess.STDOUT,
            text=True
        ).strip()

        parts = out.split()
        if len(parts) != 6:
            print(f"Unexpected output from gz model: {out}")
            return

        x, y, z, roll, pitch, yaw = map(float, parts)
        print(f"x={x:.3f}, y={y:.3f}, z={z:.3f}")
    except subprocess.CalledProcessError as e:
        print("Could not read Husky pose:", e.output.strip())
    except FileNotFoundError:
        print("ERROR: `gz` command not found. Is Gazebo installed and on PATH?")

def main():
    print("Printing Husky XYZ using `gz model`... (Ctrl+C to stop)")
    while True:
        get_pose()
        time.sleep(0.2)   # 5 Hz

if __name__ == "__main__":
    main()


