#!/usr/bin/env python3
"""builtin_interfaces/Timeで完了通知を配信する汎用パブリッシャ。"""
from builtin_interfaces.msg import Time as BITime


class CompletePublisher:
    def __init__(self, node, topic='/ib2_user/complete', qos=1):
        self._node = node
        self._pub = node.create_publisher(BITime, topic, qos)

    def publish(self):
        self._pub.publish(self._node.get_clock().now().to_msg())
