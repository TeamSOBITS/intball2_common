#!/usr/bin/env python3
"""platform_msgs/UserNodeStatusを配信する汎用パブリッシャ。"""
from platform_msgs.msg import UserNodeStatus


class UserNodeStatusPublisher:
    def __init__(self, node, topic='/ib2_user/status', qos=1):
        self._node = node
        self._pub = node.create_publisher(UserNodeStatus, topic, qos)

    def publish(self, status_text):
        msg = UserNodeStatus()
        msg.stamp = self._node.get_clock().now().to_msg()
        data = status_text.encode('utf-8')[:800]
        msg.msg = list(data.ljust(800, b'\x00'))
        self._pub.publish(msg)
