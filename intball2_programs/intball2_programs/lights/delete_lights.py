#!/usr/bin/env python3
"""iss_light1〜16をまとめてdeleteするワンショットコマンド。"""
import sys

import rclpy
from rclpy.node import Node

from intball2_programs.lights.light_models import LIGHT_MODELS
from intball2_programs.ros import DeleteModelServiceClient, ModelStatesSubscriber


def main():
    rclpy.init()
    node = Node('delete_lights_client')
    delete_service = DeleteModelServiceClient(node)
    model_states = ModelStatesSubscriber(node)

    try:
        model_states.wait_until_received(timeout_sec=5.0)
        existing_names = set(model_states.model_states.name)
    except RuntimeError as exc:
        node.get_logger().warn(str(exc))
        existing_names = set()

    deleted, skipped, failed = [], [], []

    for name in LIGHT_MODELS:
        if name not in existing_names:
            skipped.append(name)
            continue
        if delete_service.call(name):
            deleted.append(name)
        else:
            failed.append(name)

    print('\n=== delete_lights summary ===')
    print(f'Deleted ({len(deleted)}): {", ".join(deleted) if deleted else "-"}')
    print(f'Skipped, not found ({len(skipped)}): {", ".join(skipped) if skipped else "-"}')
    print(f'Failed ({len(failed)}): {", ".join(failed) if failed else "-"}')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
