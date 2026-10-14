#!/usr/bin/env python3
"""照明モデル(ALL_LIGHTS_MODEL_NAME)をdeleteするワンショットコマンド。

通常の消灯操作は`spawn_lights`をCtrl+Cで止めることで行う(常駐終了時に自動delete される)。
本コマンドは`spawn_lights`が`kill -9`やクラッシュ等で異常終了し、消し忘れた場合の
手動クリーンアップ用フォールバックとして使う(docs/gazebo_light_spawn_plan.md参照)。
"""
import sys

import rclpy
from rclpy.node import Node

from intball2_programs.lights.light_models import ALL_LIGHTS_MODEL_NAME
from intball2_programs.ros import DeleteModelServiceClient, GetModelStateServiceClient


def main():
    rclpy.init()
    node = Node('delete_lights_client')
    delete_service = DeleteModelServiceClient(node)
    get_state = GetModelStateServiceClient(node)

    if not get_state.wait_for_service(timeout_sec=10.0):
        code = 1
    elif get_state.call(ALL_LIGHTS_MODEL_NAME) is None:
        print(f'{ALL_LIGHTS_MODEL_NAME}: not found, nothing to delete.')
        code = 0
    elif delete_service.call(ALL_LIGHTS_MODEL_NAME):
        print(f'{ALL_LIGHTS_MODEL_NAME}: deleted.')
        code = 0
    else:
        print(f'{ALL_LIGHTS_MODEL_NAME}: failed to delete.')
        code = 1

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
