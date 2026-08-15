#!/usr/bin/env python3
"""gazebo_msgs/DeleteModelサービスを呼び出す汎用クライアント。"""
import rclpy
from gazebo_msgs.srv import DeleteModel


class DeleteModelServiceClient:
    def __init__(self, node, service='/gazebo/delete_model'):
        self._node = node
        self._cli = node.create_client(DeleteModel, service)

    def call(self, name, wait_timeout_sec=2.0, call_timeout_sec=5.0):
        if not self._node.context.ok():
            return False
        if not self._cli.wait_for_service(timeout_sec=wait_timeout_sec):
            self._node.get_logger().warn('Delete service not available.')
            return False
        req = DeleteModel.Request()
        req.model_name = name
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=call_timeout_sec)
        if future.result() is None:
            self._node.get_logger().warn('DeleteModel call timed out.')
            return False
        res = future.result()
        if res.success:
            self._node.get_logger().info(f'DeleteModel OK: {res.status_message}')
            return True
        self._node.get_logger().warn(f'DeleteModel NG: {res.status_message}')
        return False
