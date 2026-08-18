#!/usr/bin/env python3
import argparse
import math
import os
import sys
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker

from intball2_programs.sim_models import (
    SIM_MODELS, ASTROBEE_COLLISION_BLOCK, ASTROBEE_VISUAL_PARTS,
    build_sim_model_xml, local_mesh_uri, astrobee_mesh_uri,
)
from intball2_programs.lights.light_models import LIGHT_MODELS, build_light_model_xml
from intball2_programs.ros import (
    DeleteModelServiceClient,
    MarkerArrayPublisher,
    ModelStatePublisher,
    ModelStatesSubscriber,
    SpawnModelServiceClient,
    TFClient,
)


DEFAULT_MODEL = 'box_obstacle'
DEFAULT_SIZE = (0.45, 0.25, 1.7)
DEFAULT_OFFSET = (0.0, 0.0, 0.0)
DEFAULT_RPY = (0.0, 0.0, 0.0)
DEFAULT_FRAME = 'iss_body'
WORLD_FRAME = 'world'
ISS_MODEL_NAME = 'iss'
ISS_TF_FRAME = 'iss_body'

# astrobee_freeflyer は複数visual/複数collisionの特殊構成のため、SIM_MODELS辞書ではなく
# 専用テンプレート(models/astrobee_freeflyer.sdf)を使う(sim_models.py参照)。
ASTROBEE_MODEL_NAME = 'astrobee_freeflyer'


class SpawnBoxClient(Node):
    def __init__(self):
        super().__init__('spawn_box_client')
        self.spawn_service = SpawnModelServiceClient(self)
        self.delete_service = DeleteModelServiceClient(self)
        self.model_states = ModelStatesSubscriber(self)
        self.model_state_pub = ModelStatePublisher(self)
        self.tf_client = TFClient(self)
        self.marker_pub = MarkerArrayPublisher(self, topic='spawned_model_markers')

    def wait_service(self, timeout_sec=10.0):
        return self.spawn_service.wait_for_service(timeout_sec)

    def wait_for_model_states(self, timeout_sec=5.0):
        self.model_states.wait_until_received(timeout_sec)

    def spawn(self, name, model_xml, pose, reference_frame):
        return self.spawn_service.call(name, model_xml, pose, reference_frame)

    def delete_model(self, name):
        return self.delete_service.call(name)

    def build_pose(self, frame, offset, rpy):
        iss_pose = self.model_states.get_pose(ISS_MODEL_NAME)
        if iss_pose is None:
            raise RuntimeError(f'Model not found in /gazebo/model_states: {ISS_MODEL_NAME}')

        relative_pos = self._resolve_offset(frame, offset)
        iss_rotation = iss_pose.orientation
        offset_rot = rotate_vector(iss_rotation, relative_pos)
        position = Point(
            x=iss_pose.position.x + offset_rot[0],
            y=iss_pose.position.y + offset_rot[1],
            z=iss_pose.position.z + offset_rot[2],
        )
        orientation = quaternion_multiply(iss_rotation, rpy_deg_to_quaternion(*rpy))
        return Pose(position=position, orientation=orientation)

    def build_relative_pose(self, frame, offset, rpy):
        """rviz表示用。iss_bodyフレームでの相対姿勢(Gazebo world座標への変換なし)を返す。

        rvizのFixed Frameは常にiss_body(このrviz.rvizのGlobal Options参照)なので、
        build_pose()のようにiss_pose(Gazebo world内での実際のISS姿勢)を合成する必要がない。
        """
        relative_pos = self._resolve_offset(frame, offset)
        return Pose(
            position=Point(x=relative_pos[0], y=relative_pos[1], z=relative_pos[2]),
            orientation=rpy_deg_to_quaternion(*rpy),
        )

    def _resolve_offset(self, target_frame, manual_offset):
        if not target_frame or target_frame == ISS_TF_FRAME:
            return manual_offset
        translation = self.tf_client.lookup_translation(
            ISS_TF_FRAME, target_frame, logger=self.get_logger()
        )
        if translation is None:
            return manual_offset
        return [translation[i] + manual_offset[i] for i in range(3)]

    def publish_sync(self, instance_name, frame, offset, rpy):
        try:
            pose = self.build_pose(frame, offset, rpy)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            return
        self.model_state_pub.publish(instance_name, pose, WORLD_FRAME)


def build_collision_block(size_x, size_y, size_z):
    return (
        '      <collision name="collision">\n'
        '        <geometry>\n'
        f'          <box><size>{size_x} {size_y} {size_z}</size></box>\n'
        '        </geometry>\n'
        '      </collision>\n'
    )


def rpy_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def quaternion_conjugate(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def quaternion_multiply(q1, q2):
    return Quaternion(
        w=q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        x=q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        y=q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        z=q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
    )


def rotate_vector(q, vector):
    vec_quat = Quaternion(x=vector[0], y=vector[1], z=vector[2], w=0.0)
    q_conj = quaternion_conjugate(q)
    rotated = quaternion_multiply(quaternion_multiply(q, vec_quat), q_conj)
    return rotated.x, rotated.y, rotated.z


def _new_marker(ns, marker_id, frame_id, stamp, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    return marker


def _local_rotation_quaternion(pose_orientation, rpy_rad):
    roll, pitch, yaw = rpy_rad
    local = rpy_deg_to_quaternion(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    return quaternion_multiply(pose_orientation, local)


def build_mesh_markers(ns, meta, pose, frame_id, stamp):
    """SIM_MODELS(tape/ctb_*/float_*)の1メッシュ分のMarkerを返す。"""
    marker = _new_marker(ns, 0, frame_id, stamp, Marker.MESH_RESOURCE)
    marker.mesh_resource = local_mesh_uri(meta)
    marker.mesh_use_embedded_materials = True
    scale = float(meta.get('scale', 1))
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.pose.position = pose.position
    marker.pose.orientation = _local_rotation_quaternion(
        pose.orientation, meta.get('pose_rpy', (0.0, 0.0, 0.0))
    )
    marker.color.a = 1.0
    return [marker]


def build_astrobee_markers(ns, pose, frame_id, stamp):
    """astrobee_freeflyer(複数visual構成)のMarker一覧を返す。"""
    markers = []
    for idx, part in enumerate(ASTROBEE_VISUAL_PARTS):
        marker = _new_marker(ns, idx, frame_id, stamp, Marker.MESH_RESOURCE)
        marker.mesh_resource = astrobee_mesh_uri(part)
        marker.mesh_use_embedded_materials = True
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.pose.position = pose.position
        marker.pose.orientation = _local_rotation_quaternion(pose.orientation, part['rpy'])
        marker.color.a = 1.0
        markers.append(marker)
    return markers


def build_box_markers(ns, size, pose, frame_id, stamp, color=(0.2, 0.6, 1.0, 0.6)):
    """メッシュを持たないモデル(box/human/laptop)向けの簡易CUBE表示。"""
    marker = _new_marker(ns, 0, frame_id, stamp, Marker.CUBE)
    marker.scale.x, marker.scale.y, marker.scale.z = size
    marker.pose = pose
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return [marker]


def build_delete_markers(ns, count, frame_id):
    markers = []
    for idx in range(count):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = idx
        marker.action = Marker.DELETE
        markers.append(marker)
    return markers


def load_model_template(model_name):
    share_dir = get_package_share_directory('intball2_programs')
    model_path = os.path.join(share_dir, 'models', f'{model_name}.sdf')
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f'Model SDF not found: {model_path}')
    with open(model_path, 'r', encoding='utf-8') as handle:
        return handle.read()


def build_model_xml(template, model_name, size, collision_enabled, fixed_collision_block=None):
    size_x, size_y, size_z = size
    if fixed_collision_block is not None:
        collision_block = fixed_collision_block if collision_enabled else ''
    else:
        collision_block = build_collision_block(size_x, size_y, size_z) if collision_enabled else ''
    return template.format(
        model_name=model_name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        collision_block=collision_block,
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Spawn an SDF model in Gazebo.')
    parser.add_argument('-m', dest='model', default=DEFAULT_MODEL)
    parser.add_argument('-n', dest='name', default=None)
    parser.add_argument('-f', dest='frame', default=DEFAULT_FRAME)
    parser.add_argument('-s', dest='size', nargs=3, type=float, default=list(DEFAULT_SIZE))
    parser.add_argument('-o', dest='offset', nargs=3, type=float, default=list(DEFAULT_OFFSET))
    parser.add_argument('-r', dest='rpy', nargs=3, type=float, default=list(DEFAULT_RPY))
    parser.add_argument('-c', dest='collision', action='store_true')
    return parser.parse_args(argv)


def _parse_args_with_legacy_fallback(argv, logger):
    if not argv or argv[0].startswith('-'):
        return parse_args(argv)
    logger.warn(
        'Positional arguments are deprecated. Use -m/-n/-f/-s/-o/-r/-c instead.'
    )
    name = argv[0]
    x = float(argv[1]) if len(argv) > 1 else 0.0
    y = float(argv[2]) if len(argv) > 2 else 0.0
    z = float(argv[3]) if len(argv) > 3 else 1.0
    parsed = parse_args([])
    parsed.name = name
    parsed.offset = [x, y, z]
    parsed.frame = ISS_TF_FRAME
    return parsed


def main():
    # rclpyの既定動作では、SIGINT受信時に自前のシグナルハンドラがcontextを即座に
    # shutdownしてしまい、以降のクリーンアップ(Marker DELETE配信・Gazeboモデル削除)が
    # 壊れたcontext相手になって失敗する。ここでは自動ハンドラを無効化し、通常の
    # KeyboardInterruptとしてPython側で受け取ってから、生きたnodeで後片付けする。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SpawnBoxClient()
    parsed = _parse_args_with_legacy_fallback(sys.argv[1:], node.get_logger())

    instance_name = parsed.name or f'{parsed.model}_{uuid.uuid4().hex[:8]}'

    if not node.wait_service():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        if parsed.model in LIGHT_MODELS:
            model_xml = build_light_model_xml(instance_name, LIGHT_MODELS[parsed.model])
        elif parsed.model in SIM_MODELS:
            model_xml = build_sim_model_xml(instance_name, SIM_MODELS[parsed.model], parsed.collision)
        else:
            fixed_collision_block = ASTROBEE_COLLISION_BLOCK if parsed.model == ASTROBEE_MODEL_NAME else None
            template = load_model_template(parsed.model)
            model_xml = build_model_xml(
                template, instance_name, parsed.size, parsed.collision,
                fixed_collision_block=fixed_collision_block,
            )
        node.wait_for_model_states()
        pose = node.build_pose(parsed.frame, parsed.offset, parsed.rpy)
        ok = node.spawn(instance_name, model_xml, pose, WORLD_FRAME)
    except FileNotFoundError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not ok:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if parsed.model in LIGHT_MODELS:
        # 光源はRVizでの可視化不要のためMarkerを出さない(docs/gazebo_light_spawn_plan.md参照)。
        build_markers = lambda pose, stamp: []  # noqa: E731
    elif parsed.model in SIM_MODELS:
        meta = SIM_MODELS[parsed.model]
        build_markers = lambda pose, stamp: build_mesh_markers(  # noqa: E731
            instance_name, meta, pose, ISS_TF_FRAME, stamp
        )
    elif parsed.model == ASTROBEE_MODEL_NAME:
        build_markers = lambda pose, stamp: build_astrobee_markers(  # noqa: E731
            instance_name, pose, ISS_TF_FRAME, stamp
        )
    else:
        build_markers = lambda pose, stamp: build_box_markers(  # noqa: E731
            instance_name, parsed.size, pose, ISS_TF_FRAME, stamp
        )

    marker_count = 0

    def tick():
        nonlocal marker_count
        node.publish_sync(instance_name, parsed.frame, parsed.offset, parsed.rpy)
        relative_pose = node.build_relative_pose(parsed.frame, parsed.offset, parsed.rpy)
        markers = build_markers(relative_pose, node.get_clock().now().to_msg())
        marker_count = len(markers)
        node.marker_pub.publish(markers)

    timer = node.create_timer(0.1, tick)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n[Shutdown] KeyboardInterrupt received.')
    except RuntimeError as exc:
        # SIGINTがspin_once()内のC拡張呼び出し(take_message)の最中に届くと、
        # KeyboardInterruptではなくRuntimeErrorとして飛んでくることがある
        # (rclpy/pybind11側の既知の挙動。signal_handler_options=NOにしても発生しうる)。
        # 後片付け自体はこのfinallyで行うので、ここでは握りつぶして正常終了させる。
        print(f'\n[Shutdown] Received during SIGINT: {exc}')
    finally:
        timer.cancel()
        if marker_count:
            node.marker_pub.publish(build_delete_markers(instance_name, marker_count, ISS_TF_FRAME))
        print(f'[Cleanup] Attempting to delete model: {instance_name}')
        if node.delete_model(instance_name):
            print(f'[Cleanup] Deleted: {instance_name}')
        else:
            print('[Cleanup] Delete call timed out or failed (Gazebo closed?).')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
