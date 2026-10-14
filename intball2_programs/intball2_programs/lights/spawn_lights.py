#!/usr/bin/env python3
"""ISS室内照明(iss_light_1〜16)を一斉に点灯し、ISSに追従させ続けるコマンド。

点灯するlightは全て1つのモデル(ALL_LIGHTS_MODEL_NAME)にまとめてspawnするため、
spawn/delete各1回で一斉に点灯・消灯できる。`-l`で点灯するlightを選べる(省略時は全light)。

各lightの位置は`locations/spawn_locations.yaml`(iss_bodyからの相対オフセット)を直接読んで
求める(TFには依存しない)。ISS自体はGazebo world座標上で動く(ドリフトする)ため、spawn後も
0.1秒周期でモデルをISSの現在姿勢へ`/gazebo/set_model_state`し直して追従させる
(spawn/iss_follower.py)。

常駐コマンドであり、Ctrl+C(SIGINT)で停止するとモデルを自動deleteする
(起動中=点灯、Ctrl+C=消灯)。

`delete_lights`は通常の消灯操作ではなく、本コマンドが異常終了して消し忘れた場合の
手動クリーンアップ用フォールバック。
"""
import argparse
import os
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml

from intball2_programs.lights.light_models import (
    ALL_LIGHTS_MODEL_NAME, LIGHT_MODELS, build_lights_model_xml,
)
from intball2_programs.ros import DeleteModelServiceClient, SpawnModelServiceClient
from intball2_programs.spawn.iss_follower import IssFollower, WORLD_FRAME


def load_spawn_locations():
    share_dir = get_package_share_directory('intball2_programs')
    yaml_path = os.path.join(share_dir, 'locations', 'spawn_locations.yaml')
    with open(yaml_path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def parse_args(argv):
    parser = argparse.ArgumentParser(description='ISS室内照明を一斉に点灯し、ISSに追従させる。')
    parser.add_argument(
        '-l', dest='lights', nargs='+', choices=list(LIGHT_MODELS), metavar='NAME',
        help='点灯するlight名 (省略時は全light)。例: -l iss_light_1 iss_light_3',
    )
    return parser.parse_args(argv)


class SpawnLightsClient(Node):
    def __init__(self):
        super().__init__('spawn_lights_client')
        self.spawn_service = SpawnModelServiceClient(self)
        self.delete_service = DeleteModelServiceClient(self)
        self.iss_follower = IssFollower(self)


def _exit(node, code):
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


def main():
    args = parse_args(remove_ros_args(sys.argv)[1:])
    light_names = list(dict.fromkeys(args.lights)) if args.lights else list(LIGHT_MODELS)

    # rclpyの既定動作では、SIGINT受信時に自前のシグナルハンドラがcontextを即座に
    # shutdownしてしまい、以降のクリーンアップ(delete)が壊れたcontext相手になって
    # 失敗する。ここでは自動ハンドラを無効化し、通常のKeyboardInterruptとして
    # Python側で受け取ってから、生きたnodeで後片付けする(spawn.pyと同じパターン)。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    spawn_locations = load_spawn_locations()
    node = SpawnLightsClient()

    undefined = [name for name in light_names if name not in spawn_locations]
    if undefined:
        node.get_logger().error(f'Not found in spawn_locations.yaml: {", ".join(undefined)}')
        _exit(node, 1)

    if not node.spawn_service.wait_for_service(timeout_sec=10.0):
        _exit(node, 1)
    if not node.iss_follower.wait_until_ready():
        _exit(node, 1)

    if node.iss_follower.exists(ALL_LIGHTS_MODEL_NAME):
        node.get_logger().error(
            f'{ALL_LIGHTS_MODEL_NAME} already exists in Gazebo. Run "delete_lights" first.'
        )
        _exit(node, 1)

    model_xml = build_lights_model_xml(light_names, spawn_locations)
    pose = node.iss_follower.to_world_pose()
    if not node.spawn_service.call(ALL_LIGHTS_MODEL_NAME, model_xml, pose, WORLD_FRAME):
        _exit(node, 1)

    print(f'\nLights on ({len(light_names)}): {", ".join(light_names)}')
    print('Syncing to ISS every 0.1s. Press Ctrl+C to turn off.')

    def tick():
        node.iss_follower.request_update()
        node.iss_follower.publish(ALL_LIGHTS_MODEL_NAME)

    timer = node.create_timer(0.1, tick)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n[Shutdown] KeyboardInterrupt received.')
    except RuntimeError as exc:
        # SIGINTがspin_once()内のC拡張呼び出しの最中に届くと、KeyboardInterruptではなく
        # RuntimeErrorとして飛んでくることがある(spawn.pyと同じ既知の挙動)。
        print(f'\n[Shutdown] Received during SIGINT: {exc}')
    finally:
        timer.cancel()
        print('[Cleanup] Deleting lights...')
        if node.delete_service.call(ALL_LIGHTS_MODEL_NAME):
            print(f'[Cleanup] Deleted: {ALL_LIGHTS_MODEL_NAME}')
        else:
            print(f'[Cleanup] Failed to delete {ALL_LIGHTS_MODEL_NAME}. Run "delete_lights".')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
