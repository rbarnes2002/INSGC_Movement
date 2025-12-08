#!/usr/bin/env python3

import subprocess
import time

# Names of the two Husky models you spawned
MODEL_NAMES = ["husky1", "husky2"]


def get_pose(model_name):
    try:
        # gz model -m <model_name> -p  → outputs:  x y z roll pitch yaw
        out = subprocess.check_output(
            ["gz", "model", "-m", model_name, "-p"],
            stderr=subprocess.STDOUT,
            text=True
        ).strip()

        parts = out.split()
        if len(parts) != 6:
            print(f"Unexpected output from gz for {model_name}: {out}")
            return None

        x, y, z, roll, pitch, yaw = map(float, parts)
        return x, y, z

    except subprocess.CalledProcessError as e:
        print(f"Could not read pose for {model_name}: {e.output.strip()}")
        return None

    except FileNotFoundError:
        print("ERROR: `gz` command not found. Is Gazebo installed and on PATH?")
        exit(1)


def main():
    print("Printing positions of Husky1 and Husky2 (Ctrl+C to stop)\n")

    while True:
        for name in MODEL_NAMES:
            pose = get_pose(name)
            if pose:
                x, y, z = pose
                print(f"{name}:  x={x:.3f}, y={y:.3f}, z={z:.3f}")
            else:
                print(f"{name}:  pose unavailable")

        print("-" * 40)
        time.sleep(0.2)   # 5 Hz update rate


if __name__ == "__main__":
    main()

