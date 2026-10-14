"""geometry_msgs/Quaternionを使う姿勢計算(spawn.py/iss_follower.pyで共用)。"""
import math

from geometry_msgs.msg import Quaternion


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
