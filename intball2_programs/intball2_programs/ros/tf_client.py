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

    def lookup_pose(self, target_frame, source_frame, logger=None, allow_zero_stamp=False):
        """target_frame から見た source_frame の(平行移動[x, y, z], 姿勢[qx, qy, qz, qw])を返す。

        取得できない場合や、listenerがまだ実データを受信しておらずstamp=0の
        ダミー値が返った場合はNoneを返す(呼び出し側でリトライすること)。
        URDFの固定関節などの静的なTF(/tf_static)だけでつながるフレーム間は、
        正しい値でもstamp=0になるため、allow_zero_stamp=Trueにして受け付ける。
        """
        pose = self.lookup_pose_stamped(target_frame, source_frame, logger, allow_zero_stamp)
        if pose is None:
            return None
        translation, rotation, _stamp = pose
        return translation, rotation

    def lookup_pose_stamped(self, target_frame, source_frame, logger=None,
                            allow_zero_stamp=False):
        """lookup_pose() と同じ値に、TFの時刻(header.stamp)[s]を加えた3つ組を返す。"""
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
        stamp = transform.header.stamp
        if not allow_zero_stamp and stamp.sec == 0 and stamp.nanosec == 0:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w], stamp.sec + stamp.nanosec * 1e-9
