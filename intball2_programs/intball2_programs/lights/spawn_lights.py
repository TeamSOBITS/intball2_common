#!/usr/bin/env python3
"""iss_light1〜16をまとめてspawnするワンショットコマンド。

個別1個のspawn(`spawn_model -m iss_light1 ...`)とは違い、常駐せず一度spawnしたら
即終了する。各lightの位置は`locations/spawn_locations.yaml`(iss_bodyからの相対オフセット)
を直接読んで求める。TFには依存しないため、`spawn_location_broadcaster`の起動有無に関係なく
常に動作する(docs/gazebo_light_spawn_plan.md参照)。ISS自体は動くため、絶対座標は
そのつどISSの現在姿勢(/gazebo/model_states)と相対オフセットを合成して求める。
"""
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose

from intball2_programs.lights.light_models import LIGHT_MODELS, build_light_model_xml
from intball2_programs.ros import ModelStatesSubscriber, SpawnModelServiceClient
from intball2_programs.spawn import ISS_MODEL_NAME, WORLD_FRAME, rotate_vector


def load_spawn_locations():
    share_dir = get_package_share_directory('intball2_programs')
    yaml_path = os.path.join(share_dir, 'locations', 'spawn_locations.yaml')
    with open(yaml_path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


class SpawnLightsClient(Node):
    def __init__(self, spawn_locations):
        super().__init__('spawn_lights_client')
        self.spawn_service = SpawnModelServiceClient(self)
        self.model_states = ModelStatesSubscriber(self)
        self.spawn_locations = spawn_locations

    def resolve_pose(self, light_name):
        """spawn_locations.yamlのlight_nameエントリからspawn用のPoseを求める。未定義ならNoneを返す。"""
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


def main():
    rclpy.init()
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

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
