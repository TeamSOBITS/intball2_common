#!/usr/bin/env python3
"""visualization_msgs/MarkerArrayを配信する汎用パブリッシャ。"""
from visualization_msgs.msg import MarkerArray


class MarkerArrayPublisher:
    def __init__(self, node, topic='visualization_marker_array', qos=10):
        self._pub = node.create_publisher(MarkerArray, topic, qos)

    def publish(self, markers):
        self._pub.publish(MarkerArray(markers=markers))

    def get_subscription_count(self):
        return self._pub.get_subscription_count()
