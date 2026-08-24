#!/usr/bin/env python3
"""iss_light_1〜16をまとめてspawnし、ISSに追従させ続けるコマンド。

各lightの位置は`locations/spawn_locations.yaml`(iss_bodyからの相対オフセット)を直接読んで
求める(TFには依存しない)。ISS自体はGazebo world座標上で動く(ドリフトする)ため、spawn後も
0.1秒周期でISSの現在姿勢(/gazebo/model_states)と相対オフセットを合成し直し、
`/gazebo/set_model_state`を送り続けて追従させる(既存の単体`spawn_model`と同じ仕組み)。

常駐コマンドであり、Ctrl+C(SIGINT)で停止すると同期対象だった全light(spawn成功分＋
起動時に既に存在していた分)を自動deleteする。「起動中=点灯、Ctrl+C=消灯」という
単体`spawn_model`と同じライフサイクル(docs/gazebo_light_spawn_plan.md参照)。

`delete_lights`は通常の消灯操作ではなく、本コマンドが異常終了して消し忘れた場合の
手動クリーンアップ用フォールバック。
"""
import os
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
import yaml

from intball2_programs.lights.light_models import LIGHT_MODELS, build_light_model_xml
from intball2_programs.ros import (
    DeleteModelServiceClient,
    ModelStatePublisher,
    ModelStatesSubscriber,
    SpawnModelServiceClient,
)
from intball2_programs.spawn.spawn import ISS_MODEL_NAME, WORLD_FRAME, rotate_vector


def load_spawn_locations():
    share_dir = get_package_share_directory('intball2_programs')
    yaml_path = os.path.join(share_dir, 'locations', 'spawn_locations.yaml')
    with open(yaml_path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


class SpawnLightsClient(Node):
    def __init__(self, spawn_locations):
        super().__init__('spawn_lights_client')
        self.spawn_service = SpawnModelServiceClient(self)
        self.delete_service = DeleteModelServiceClient(self)
        self.model_states = ModelStatesSubscriber(self)
        self.model_state_pub = ModelStatePublisher(self)
        self.spawn_locations = spawn_locations

    def resolve_pose(self, light_name):
        """spawn_locations.yamlのlight_nameエントリと、ISSの現在姿勢からPoseを求める。

        light_nameが未定義、またはISSの姿勢が未受信の場合はNoneを返す。
        """
        entry = self.spawn_locations.get(light_name)
        if entry is None:
            return None
        iss_pose = self.model_states.get_pose(ISS_MODEL_NAME)
        if iss_pose is None:
            return None
        translation = [
            entry['translation']['x'], entry['translation']['y'], entry['translation']['z'],
        ]
        offset_rot = rotate_vector(iss_pose.orientation, translation)
        position = Point(
            x=iss_pose.position.x + offset_rot[0],
            y=iss_pose.position.y + offset_rot[1],
            z=iss_pose.position.z + offset_rot[2],
        )
        return Pose(position=position, orientation=iss_pose.orientation)

    def sync_once(self, light_name):
        """light_nameの現在の絶対座標を計算し、/gazebo/set_model_stateへ再送する。"""
        pose = self.resolve_pose(light_name)
        if pose is not None:
            self.model_state_pub.publish(light_name, pose, WORLD_FRAME)


def main():
    # rclpyの既定動作では、SIGINT受信時に自前のシグナルハンドラがcontextを即座に
    # shutdownしてしまい、以降のクリーンアップ(delete)が壊れたcontext相手になって
    # 失敗する。ここでは自動ハンドラを無効化し、通常のKeyboardInterruptとして
    # Python側で受け取ってから、生きたnodeで後片付けする(spawn.pyと同じパターン)。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    spawn_locations = load_spawn_locations()
    node = SpawnLightsClient(spawn_locations)

    if not node.spawn_service.wait_for_service(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        node.model_states.wait_until_received(timeout_sec=5.0)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    existing_names = set(node.model_states.model_states.name)

    spawned, skipped, failed = [], [], []

    for name, meta in LIGHT_MODELS.items():
        if name in existing_names:
            node.get_logger().warn(f'{name}: already exists in /gazebo/model_states, skipping.')
            skipped.append(name)
            continue

        pose = node.resolve_pose(name)
        if pose is None:
            node.get_logger().error(f'{name}: not found in spawn_locations.yaml, skipping spawn.')
            failed.append((name, 'location not defined'))
            continue

        model_xml = build_light_model_xml(name, meta)
        if node.spawn_service.call(name, model_xml, pose, WORLD_FRAME):
            spawned.append(name)
        else:
            failed.append((name, 'spawn service failed'))

    print('\n=== spawn_lights summary ===')
    print(f'Spawned ({len(spawned)}): {", ".join(spawned) if spawned else "-"}')
    print(f'Skipped, already existed ({len(skipped)}): {", ".join(skipped) if skipped else "-"}')
    if failed:
        print(f'Failed ({len(failed)}):')
        for name, reason in failed:
            print(f'  - {name}: {reason}')
    else:
        print('Failed (0): -')

    # 同期・Ctrl+C時の削除対象は「今回spawnできたもの」+「起動時に既に存在していたもの」。
    # 存在しないものを追いかけても仕方ないので対象から除く。
    active_lights = spawned + skipped

    if not active_lights:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1 if failed else 0)

    print(f'\nSyncing {len(active_lights)} light(s) to ISS every 0.1s. Press Ctrl+C to turn off.')

    def tick():
        for name in active_lights:
            node.sync_once(name)

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
        deleted, delete_failed = [], []
        for name in active_lights:
            if node.delete_service.call(name):
                deleted.append(name)
            else:
                delete_failed.append(name)
        print(f'[Cleanup] Deleted ({len(deleted)}): {", ".join(deleted) if deleted else "-"}')
        if delete_failed:
            print(f'[Cleanup] Failed to delete ({len(delete_failed)}): {", ".join(delete_failed)}')
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
