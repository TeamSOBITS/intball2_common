#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from ib2_msgs.msg import Navigation, IMU

class Intball2AdvancedController(Node):
    def __init__(self):
        # ノード名の初期化
        super().__init__('intball2_vel_bridge_imu')

        # --- ゲイン調整 ---
        self.k_p_lin = 2.0   # 速度偏差へのゲイン
        self.k_d_lin = 0.2   # 加速度（ブレーキ）へのゲイン
        self.k_p_ang = 0.5   # 角速度偏差へのゲイン

        self.target_twist = Twist()
        self.current_twist = Twist()
        self.current_acc = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # --- Pub/Sub ---
        # QoSはデフォルトの10を使用
        self.pub = self.create_publisher(WrenchStamped, '/ctl/wrench', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Navigation, '/sensor_fusion/navigation', self.nav_cb, 10)
        self.create_subscription(IMU, '/imu/imu', self.imu_cb, 10)

        # --- タイマー処理 ---
        # ROS 1の rospy.Rate(20) と whileループの代わりに、Timerを使用 (1/20 = 0.05秒)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def cmd_vel_cb(self, msg):
        self.target_twist = msg

    def nav_cb(self, msg):
        self.current_twist = msg.twist

    def imu_cb(self, msg):
        # 加速度と角速度を保存
        self.current_acc = {'x': msg.acc_x, 'y': msg.acc_y, 'z': msg.acc_z}
        self.current_gyro = {'x': msg.gyro_x, 'y': msg.gyro_y, 'z': msg.gyro_z}

    def timer_callback(self):
        ws = WrenchStamped()
        # ROS 2での現在時刻の取得
        ws.header.stamp = self.get_clock().now().to_msg()
        ws.header.frame_id = "body"

        # --- 平行移動 (力) ---
        ws.wrench.force.x = self.k_p_lin * (self.target_twist.linear.x - self.current_twist.linear.x) - (self.k_d_lin * self.current_acc['x'])
        ws.wrench.force.y = self.k_p_lin * (self.target_twist.linear.y - self.current_twist.linear.y) - (self.k_d_lin * self.current_acc['y'])
        ws.wrench.force.z = self.k_p_lin * (self.target_twist.linear.z - self.current_twist.linear.z) - (self.k_d_lin * self.current_acc['z'])

        # --- 回転 (トルク) ---
        ws.wrench.torque.x = self.k_p_ang * (self.target_twist.angular.x - self.current_gyro['x'])
        ws.wrench.torque.y = self.k_p_ang * (self.target_twist.angular.y - self.current_gyro['y'])
        ws.wrench.torque.z = self.k_p_ang * (self.target_twist.angular.z - self.current_gyro['z'])
        
        lin_diff_x = self.target_twist.linear.x - self.current_twist.linear.x
        lin_diff_y = self.target_twist.linear.y - self.current_twist.linear.y
        
        # ROS 2でのロギング
        self.get_logger().info(f"Target: 0, Current Vel X: {self.current_twist.linear.x:.3f}, Diff X: {lin_diff_x:.3f}")
        self.get_logger().info(f"Target: 0, Current Vel Y: {self.current_twist.linear.y:.3f}, Diff Y: {lin_diff_y:.3f}")

        self.pub.publish(ws)

def main(args=None):
    rclpy.init(args=args)
    node = Intball2AdvancedController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()