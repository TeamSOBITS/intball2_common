import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
from ament_index_python.packages import get_package_share_directory

class PlyPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'ply_points', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # パッケージのインストールパスを取得
        package_share_dir = get_package_share_directory('intball2_programs')
        
        # models と map のどちらに配置されていても読み込めるようにしています
        ply_path = os.path.join(package_share_dir, 'models', 'iss_30000.ply')
        if not os.path.exists(ply_path):
            ply_path = os.path.join(package_share_dir, 'map', 'iss_30000.ply')
        
        self.get_logger().info(f'Loading PLY file from: {ply_path}')
        
        if not os.path.exists(ply_path):
            self.get_logger().error(f'PLY file NOT found at {ply_path}!')
            return

        # PLYファイルの読み込み
        pcd = o3d.io.read_point_cloud(ply_path)
        points = np.asarray(pcd.points)

        scale_factor = 0.6 
        points = points * scale_factor

        roll_deg  = -90.0   # X軸まわりの回転（傾き）
        pitch_deg = 10.0   # Y軸まわりの回転（仰俯角）
        yaw_deg   = 10.0  # Z軸まわりの回転（方位角）

        # ラジアンに変換
        r = np.radians(roll_deg)
        p = np.radians(pitch_deg)
        y = np.radians(yaw_deg)

        # X軸まわりの回転行列
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(r), -np.sin(r)],
            [0, np.sin(r),  np.cos(r)]
        ])
        # Y軸まわりの回転行列
        R_y = np.array([
            [np.cos(p), 0, np.sin(p)],
            [0, 1, 0],
            [-np.sin(p), 0, np.cos(p)]
        ])
        # Z軸まわりの回転行列
        R_z = np.array([
            [np.cos(y), -np.sin(y), 0],
            [np.sin(y),  np.cos(y), 0],
            [0, 0, 1]
        ])

        # 3つの回転行列を合成 (Z * Y * X の順で適用)
        R = np.dot(R_z, np.dot(R_y, R_x))

        # 点群全体を回転
        points = np.dot(points, R.T)
        # 2. 位置（平行移動）の微調整（メートル単位）
        # 3Dモデルに重なるように、X（前後）、Y（左右）、Z（上下）の移動量を設定します
        shift_x = 3.0  # 前後にずらす量
        shift_y = 0.0  # 左右にずらす量
        shift_z = -0.7  # 上下にずらす量
        
        points[:, 0] += shift_x
        points[:, 1] += shift_y
        points[:, 2] += shift_z
        
        self.get_logger().info(f'Loaded {len(points)} points.')

        # 【チェック】PLYファイル自体に色情報があるか判定
        has_colors = pcd.has_colors()
        self.get_logger().info(f'PLY file has colors: {has_colors}')

        if has_colors:
            colors = np.asarray(pcd.colors)
        else:
            # 元ファイルに色がない場合、プログラムが正常か確認するため、高さ(Z)に応じたグラデーション色を自動生成
            self.get_logger().warn('PLY file does not contain color information. Generating dummy colors (Z-gradient)...')
            colors = np.zeros_like(points)
            z_min, z_max = points[:, 2].min(), points[:, 2].max()
            if z_max != z_min:
                normalized_z = (points[:, 2] - z_min) / (z_max - z_min)
                colors[:, 0] = normalized_z        # R
                colors[:, 1] = 1.0 - normalized_z  # G
                colors[:, 2] = 0.5                  # B
            else:
                colors[:] = [1.0, 1.0, 1.0]

        # PointCloud2 メッセージの初期化
        self.pc2_msg = PointCloud2()
        self.pc2_msg.height = 1
        self.pc2_msg.width = len(points)
        self.pc2_msg.is_bigendian = False
        self.pc2_msg.is_dense = True

        # RViz2で確実にRGB色を出すため、型を PointField.FLOAT32 に指定（ROSの標準ルール）
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pc2_msg.fields = fields
        self.pc2_msg.point_step = 16
        self.pc2_msg.row_step = self.pc2_msg.point_step * self.pc2_msg.width

        # NumPyの構造化配列の型も float32 に合わせる
        buffer = np.zeros(len(points), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32)
        ])
        buffer['x'] = points[:, 0]
        buffer['y'] = points[:, 1]
        buffer['z'] = points[:, 2]

        # 0.0~1.0 の値を 0~255 に変換
        colors_uint8 = (colors * 255).astype(np.uint32)
        
        # ビットシフトで1つのuint32にパック (R << 16 | G << 8 | B)
        rgb_packed = (colors_uint8[:, 0] << 16) | (colors_uint8[:, 1] << 8) | colors_uint8[:, 2]
        
        # ★重要: ビット表現をそのまま float32 に再解釈(view)して代入
        buffer['rgb'] = rgb_packed.view(np.float32)

        self.pc2_msg.data = buffer.tobytes()
        self.get_logger().info('PointCloud2 message metadata prepared with RGB.')

    def timer_callback(self):
        if hasattr(self, 'pc2_msg'):
            self.pc2_msg.header.stamp = self.get_clock().now().to_msg()
            self.pc2_msg.header.frame_id = 'dock_body' 
            self.publisher_.publish(self.pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()