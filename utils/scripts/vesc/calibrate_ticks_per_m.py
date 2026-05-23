#!/usr/bin/env python3

import argparse
import math
import sys

import rospy
from mower_msgs.msg import ESCStatus
from xbot_msgs.msg import AbsolutePose


def signed_int32(value):
    if value >= 2 ** 31:
        return value - 2 ** 32
    return value


def capture_sample(args):
    pose = rospy.wait_for_message(args.pose_topic, AbsolutePose, timeout=args.timeout)
    left = rospy.wait_for_message(args.left_topic, ESCStatus, timeout=args.timeout)
    right = rospy.wait_for_message(args.right_topic, ESCStatus, timeout=args.timeout)
    return {
        "x": pose.pose.pose.position.x,
        "y": pose.pose.pose.position.y,
        "accuracy": pose.position_accuracy,
        "left_tacho": signed_int32(left.tacho),
        "right_tacho": signed_int32(right.tacho),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Estimate OM_WHEEL_TICKS_PER_M from a short straight Mowrator drive with good RTK."
    )
    parser.add_argument("--pose-topic", default="/xbot_positioning/xb_pose")
    parser.add_argument("--left-topic", default="/ll/diff_drive/left_esc_status")
    parser.add_argument("--right-topic", default="/ll/diff_drive/right_esc_status")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--min-distance", type=float, default=1.5)
    args = parser.parse_args()

    rospy.init_node("calibrate_ticks_per_m", anonymous=True)

    print("Wait for stable RTK, then keep the mower pointed straight.")
    print("Press Enter to capture the start sample, drive straight forward 2-5 meters, then press Enter again.")
    input()
    start = capture_sample(args)
    print(
        f"Start captured: x={start['x']:.3f}, y={start['y']:.3f}, "
        f"accuracy={start['accuracy']:.3f} m, left={start['left_tacho']}, right={start['right_tacho']}"
    )

    input()
    end = capture_sample(args)
    print(
        f"End captured: x={end['x']:.3f}, y={end['y']:.3f}, "
        f"accuracy={end['accuracy']:.3f} m, left={end['left_tacho']}, right={end['right_tacho']}"
    )

    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    distance_m = math.hypot(dx, dy)
    left_delta = end["left_tacho"] - start["left_tacho"]
    right_delta = end["right_tacho"] - start["right_tacho"]

    if distance_m < args.min_distance:
        print(
            f"Measured distance was only {distance_m:.3f} m. Drive farther in a straight line and try again.",
            file=sys.stderr,
        )
        return 1

    avg_ticks = (abs(left_delta) + abs(right_delta)) / 2.0
    if avg_ticks <= 0:
        print("Wheel tick delta was zero. Make sure ESC power is on and the mower actually moved.", file=sys.stderr)
        return 1

    ticks_per_m = avg_ticks / distance_m
    left_ticks_per_m = abs(left_delta) / distance_m
    right_ticks_per_m = abs(right_delta) / distance_m

    print("")
    print(f"Distance moved: {distance_m:.3f} m")
    print(f"Left tacho delta: {left_delta}")
    print(f"Right tacho delta: {right_delta}")
    print(f"Recommended ticks_per_m: {ticks_per_m:.3f}")
    print(f"Left estimate: {left_ticks_per_m:.3f}")
    print(f"Right estimate: {right_ticks_per_m:.3f}")
    print("")
    print("Update these with the measured value:")
    print(f"  export OM_WHEEL_TICKS_PER_M={ticks_per_m:.3f}")
    print(f"  ll.services.diff_drive.ticks_per_m: {ticks_per_m:.3f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
