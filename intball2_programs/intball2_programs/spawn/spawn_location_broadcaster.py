#!/usr/bin/env python3
"""spawn対象の固定位置(locations/spawn_locations.yaml)をTFとして配信するノード。

sobits_intball2_gncのlocation_broadcaster.py(自律移動の経由点専用)と同じ仕組みだが、
spawn対象の位置管理用として独立して用意する(docs/gazebo_light_spawn_plan.md参照)。

`spawn_lights`/`delete_lights`はこのノードに依存せずspawn_locations.yamlを直接読むため、
本ノードの主な用途は以下の2つ:
  - RVizでTFフレームを見ながらspawn_locations.yamlの位置を調整する
  - `spawn_model -m iss_light1 -f iss_light1`のように、既存の汎用`-f`フレーム指定機構を
    単体spawnで使いたい場合
"""
import os

import rclpy
from rclpy.node import Node
import yaml
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped


class SpawnLocationBroadcaster(Node):
    def __init__(self):
        super().__init__('spawn_location_broadcaster')

        self.reference_frame = 'iss_body'

        pkg_share = get_package_share_directory('intball2_programs')
        self.location_path = os.path.join(pkg_share, 'locations', 'spawn_locations.yaml')

        self.br = tf2_ros.TransformBroadcaster(self)
        self.locations_data = {}
        self.last_mtime = 0

        self.get_logger().info(f'Broadcaster started. Watching: {self.location_path}')
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.load_yaml_if_updated()
        self.publish_transforms()

    def load_yaml_if_updated(self):
        if not os.path.exists(self.location_path):
            return

        current_mtime = os.path.getmtime(self.location_path)
        if current_mtime > self.last_mtime:
            try:
                with open(self.location_path, 'r') as f:
                    data = yaml.safe_load(f)
                    self.locations_data = data or {}
                    self.last_mtime = current_mtime
                    self.get_logger().info(
                        f'YAML reloaded. {len(self.locations_data)} locations found.')
            except Exception as e:
                self.get_logger().warn(f'Failed to reload YAML: {e}')

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()

        for name, pose in self.locations_data.items():
            t = TransformStamped()

            t.header.stamp = now
            t.header.frame_id = self.reference_frame
            t.child_frame_id = name

            t.transform.translation.x = pose['translation']['x']
            t.transform.translation.y = pose['translation']['y']
            t.transform.translation.z = pose['translation']['z']

            t.transform.rotation.x = pose['rotation']['x']
            t.transform.rotation.y = pose['rotation']['y']
            t.transform.rotation.z = pose['rotation']['z']
            t.transform.rotation.w = pose['rotation']['w']

            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SpawnLocationBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
