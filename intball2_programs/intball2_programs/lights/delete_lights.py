#!/usr/bin/env python3
"""iss_light_1〜16をまとめてdeleteするワンショットコマンド。

通常の消灯操作は`spawn_lights`をCtrl+Cで止めることで行う(常駐終了時に自動delete される)。
本コマンドは`spawn_lights`が`kill -9`やクラッシュ等で異常終了し、消し忘れた場合の
手動クリーンアップ用フォールバックとして使う(docs/gazebo_light_spawn_plan.md参照)。
"""
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
