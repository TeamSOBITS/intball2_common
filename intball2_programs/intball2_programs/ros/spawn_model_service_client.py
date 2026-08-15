#!/usr/bin/env python3
"""gazebo_msgs/SpawnModelサービスを呼び出す汎用クライアント。"""
import rclpy
from gazebo_msgs.srv import SpawnModel


class SpawnModelServiceClient:
    def __init__(self, node, service='/gazebo/spawn_sdf_model'):
        self._node = node
        self._service_name = service
        self._cli = node.create_client(SpawnModel, service)

    def wait_for_service(self, timeout_sec=10.0):
        if not self._cli.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(f'Service {self._service_name} not available.')
            return False
        return True

    def call(self, name, model_xml, pose, reference_frame):
        req = SpawnModel.Request()
        req.model_name = name
        req.model_xml = model_xml
        req.robot_namespace = ''
        req.initial_pose = pose
        req.reference_frame = reference_frame

        self._node.get_logger().info(
            f'Spawning "{name}" in "{reference_frame}" at '
            f'({pose.position.x}, {pose.position.y}, {pose.position.z})...'
        )
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=15.0)

        if future.result() is None:
            self._node.get_logger().error('Service call timed out or failed.')
            return False

        res = future.result()
        if res.success:
            self._node.get_logger().info(f'OK: {res.status_message}')
            return True
        self._node.get_logger().error(f'NG: {res.status_message}')
        return False
