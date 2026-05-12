#!/usr/bin/env python3
import argparse
import math
import os
import sys
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion


DEFAULT_MODEL = 'box_obstacle'
DEFAULT_SIZE = (0.45, 0.25, 1.7)
DEFAULT_OFFSET = (0.0, 0.0, 0.0)
DEFAULT_RPY = (0.0, 0.0, 0.0)
DEFAULT_FRAME = 'iss_body'
WORLD_FRAME = 'world'
ISS_MODEL_NAME = 'iss'
ISS_TF_FRAME = 'iss_body'


class SpawnBoxClient(Node):
    def __init__(self):
        super().__init__('spawn_box_client')
        self.cli = self.create_client(SpawnModel, '/gazebo/spawn_sdf_model')
        self.delete_cli = self.create_client(DeleteModel, '/gazebo/delete_model')
        self.state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        self.model_states = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self._model_states_cb, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _model_states_cb(self, msg):
        self.model_states = msg

    def wait_service(self, timeout_sec=10.0):
        if not self.cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('Service /gazebo/spawn_sdf_model not available.')
            return False
        return True

    def wait_for_model_states(self, timeout_sec=5.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            if self.model_states is not None:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('No /gazebo/model_states received yet')

    def spawn(self, name, model_xml, pose, reference_frame):
        req = SpawnModel.Request()
        req.model_name = name
        req.model_xml = model_xml
        req.robot_namespace = ''
        req.initial_pose = pose
        req.reference_frame = reference_frame

        self.get_logger().info(
            f'Spawning "{name}" in "{reference_frame}" at '
            f'({pose.position.x}, {pose.position.y}, {pose.position.z})...'
        )
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            self.get_logger().error('Service call timed out or failed.')
            return False

        res = future.result()
        if res.success:
            self.get_logger().info(f'OK: {res.status_message}')
            return True
        self.get_logger().error(f'NG: {res.status_message}')
        return False

    def delete_model(self, name):
        if not self.context.ok():
            return False
        if not self.delete_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Service /gazebo/delete_model not available.')
            return False
        req = DeleteModel.Request()
        req.model_name = name
        future = self.delete_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().warn('DeleteModel call timed out.')
            return False
        res = future.result()
        if res.success:
            self.get_logger().info(f'DeleteModel OK: {res.status_message}')
            return True
        self.get_logger().warn(f'DeleteModel NG: {res.status_message}')
        return False

    def build_pose(self, frame, offset, rpy):
        if self.model_states is None:
            raise RuntimeError('No /gazebo/model_states received yet')
        if ISS_MODEL_NAME not in self.model_states.name:
            raise RuntimeError(f'Model not found in /gazebo/model_states: {ISS_MODEL_NAME}')
        iss_pose = self.model_states.pose[self.model_states.name.index(ISS_MODEL_NAME)]

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

    def _resolve_offset(self, target_frame, manual_offset):
        if not target_frame or target_frame == ISS_TF_FRAME:
            return manual_offset
        try:
            transform = self.tf_buffer.lookup_transform(
                ISS_TF_FRAME, target_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF lookup failed for {target_frame} -> {ISS_TF_FRAME}: {exc}. '
                'Using manual offset only.'
            )
            return manual_offset
        t = transform.transform.translation
        return [t.x + manual_offset[0], t.y + manual_offset[1], t.z + manual_offset[2]]

    def publish_sync(self, instance_name, frame, offset, rpy):
        try:
            pose = self.build_pose(frame, offset, rpy)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            return
        state = ModelState()
        state.model_name = instance_name
        state.pose = pose
        state.reference_frame = WORLD_FRAME
        self.state_pub.publish(state)


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


def load_model_template(model_name):
    share_dir = get_package_share_directory('intball2_programs')
    model_path = os.path.join(share_dir, 'models', f'{model_name}.sdf')
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f'Model SDF not found: {model_path}')
    with open(model_path, 'r', encoding='utf-8') as handle:
        return handle.read()


def build_model_xml(template, model_name, size, collision_enabled):
    size_x, size_y, size_z = size
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


def _delete_model_with_fresh_context(model_name):
    # After SIGINT, the main context is already shut down by rclpy's signal handler,
    # so a fresh context is required to reach the delete service.
    context = rclpy.context.Context()
    context.init()
    node = rclpy.create_node('spawn_cleanup', context=context)
    try:
        cli = node.create_client(DeleteModel, '/gazebo/delete_model')
        if not cli.wait_for_service(timeout_sec=2.0):
            print('[Cleanup] Delete service not available (Gazebo closed?).')
            return
        req = DeleteModel.Request()
        req.model_name = model_name
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=7.0)
        if future.result() is not None:
            print(f'[Cleanup] Deleted: {model_name}')
        else:
            print('[Cleanup] Delete call timed out.')
    except Exception as exc:
        print(f'[Cleanup] Error: {exc}')
    finally:
        node.destroy_node()
        context.try_shutdown()


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
    rclpy.init()
    node = SpawnBoxClient()
    parsed = _parse_args_with_legacy_fallback(sys.argv[1:], node.get_logger())

    instance_name = parsed.name or f'{parsed.model}_{uuid.uuid4().hex[:8]}'

    try:
        template = load_model_template(parsed.model)
    except FileNotFoundError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not node.wait_service():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        node.wait_for_model_states()
        model_xml = build_model_xml(template, instance_name, parsed.size, parsed.collision)
        pose = node.build_pose(parsed.frame, parsed.offset, parsed.rpy)
        ok = node.spawn(instance_name, model_xml, pose, WORLD_FRAME)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not ok:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    timer = node.create_timer(
        0.1,
        lambda: node.publish_sync(instance_name, parsed.frame, parsed.offset, parsed.rpy),
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n[Shutdown] KeyboardInterrupt received.')
    finally:
        timer.cancel()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print(f'[Cleanup] Attempting to delete model: {instance_name}')
        _delete_model_with_fresh_context(instance_name)


if __name__ == '__main__':
    main()
