#!/usr/bin/env python3
"""tf2のBuffer/TransformListenerを薄くラップするヘルパー。"""
import rclpy
import tf2_ros


class TFClient:
    def __init__(self, node):
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, node)

    def lookup_translation(self, target_frame, source_frame, logger=None):
        """target_frame から見た source_frame の原点の平行移動[x, y, z]を返す。

        取得できない場合はNoneを返す(呼び出し側でフォールバック値を使うこと)。
        """
        try:
            transform = self._buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            if logger is not None:
                logger.warn(f'TF lookup failed for {source_frame} -> {target_frame}: {exc}')
            return None
        t = transform.transform.translation
        return [t.x, t.y, t.z]
