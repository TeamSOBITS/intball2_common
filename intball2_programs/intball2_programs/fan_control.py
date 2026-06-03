#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
import rclpy.node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

_KJ: float = 4.082482905  # 推力から duty への換算係数 (N^0.5 -> duty)
_FAN_COUNT: int = 8


class FanControlNode:
    """Direct fan duty control for IntBall2 via /ctl/duty.

    Accepts an external rclpy.node.Node and creates a publisher on it,
    allowing instance sharing with other nodes in the same process.
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        self._node = node
        self._duties: list[float] = [0.0] * _FAN_COUNT
        self._pub = node.create_publisher(Float64MultiArray, "/ctl/duty", 1)
        self._node.get_logger().info(
            "FanControlNode initialized, publishing to /ctl/duty"
        )

    def set_duty(self, fan_id: int, duty: float) -> None:
        """Set duty for a single fan (fan_id: 0-7) and publish."""
        if not 0 <= fan_id < _FAN_COUNT:
            self._node.get_logger().warn(
                f"fan_id {fan_id} out of range [0, {_FAN_COUNT - 1}], ignored"
            )
            return
        clamped = max(0.0, min(1.0, duty))
        if clamped != duty:
            self._node.get_logger().warn(
                f"duty {duty} clamped to {clamped} for fan {fan_id}"
            )
        self._duties[fan_id] = clamped
        self._node.get_logger().info(f"fan[{fan_id}] duty -> {clamped:.3f}")
        self.publish()

    def set_all_duty(self, duty: float) -> None:
        """Set all 8 fans to the same duty and publish."""
        clamped = max(0.0, min(1.0, duty))
        if clamped != duty:
            self._node.get_logger().warn(f"duty {duty} clamped to {clamped}")
        self._duties = [clamped] * _FAN_COUNT
        self._node.get_logger().info(f"all fans duty -> {clamped:.3f}")
        self.publish()

    def publish(self) -> None:
        """Publish the current duty array to /ctl/duty."""
        self._node.get_logger().debug(
            f'publish duties: {[f"{d:.3f}" for d in self._duties]}'
        )
        self._pub.publish(self._make_msg())

    def force_to_duty(self, f: float) -> float:
        """Convert thrust [N] to duty ratio. Negative values are treated as 0."""
        return _KJ * math.sqrt(max(0.0, f))

    def duty_to_force(self, duty: float) -> float:
        """Convert duty ratio to thrust [N]."""
        return (duty / _KJ) ** 2

    def _make_msg(self) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="fan_duty", size=_FAN_COUNT, stride=1)
        ]
        msg.layout.data_offset = 0
        msg.data = list(self._duties)
        return msg


def parse_args() -> argparse.Namespace:
    if len(sys.argv) == 1:
        _build_parser().print_help()
        sys.exit(1)
    return _build_parser().parse_args()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="IntBall2 direct fan control (publishes to /ctl/duty)"
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--fan", type=int, metavar="N", help="fan index to control (0-7)"
    )
    group.add_argument(
        "--all",
        type=float,
        metavar="DUTY",
        dest="all_duty",
        help="set all fans to DUTY [0.0-1.0]",
    )
    parser.add_argument(
        "--duty",
        type=float,
        default=0.0,
        metavar="DUTY",
        help="duty ratio [0.0-1.0] (used with --fan)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        metavar="SEC",
        help="publish duration in seconds (default: 1.0)",
    )
    return parser


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("fan_control_node")
    fan = FanControlNode(node)

    if args.all_duty is not None:
        fan.set_all_duty(args.all_duty)
    else:
        fan.set_duty(args.fan, args.duty)

    node.get_logger().info(f"Publishing for {args.duration:.1f} s at 50 Hz ...")
    end = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < end:
            fan.publish()
            rclpy.spin_once(node, timeout_sec=1.0 / 50.0)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Shutting down fan_control_node")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
