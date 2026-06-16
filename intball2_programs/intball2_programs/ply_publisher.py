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
        # ツリー構造(map/...)に合わせて変更しています。必要に応じて models 等に変えてください
        ply_path = os.path.join(package_share_dir, 'models', 'iss_10000.ply')
        
        self.get_logger().info(f'Loading PLY file from: {ply_path}')
        
        if not os.path.exists(ply_path):
            self.get_logger().error(f'PLY file NOT found at {ply_path}!')
            return

        # PLYファイルの読み込み
        pcd = o3d.io.read_point_cloud(ply_path)
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        self.get_logger().info(f'Loaded {len(points)} points.')

        # PointCloud2 メッセージの箱を初期化
        self.pc2_msg = PointCloud2()
        self.pc2_msg.height = 1
        self.pc2_msg.width = len(points)
        self.pc2_msg.is_bigendian = False
        self.pc2_msg.is_dense = True

        # XYZ + RGB のフィールド定義
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.pc2_msg.fields = fields
        self.pc2_msg.point_step = 16
        self.pc2_msg.row_step = self.pc2_msg.point_step * self.pc2_msg.width

        # NumPyの構造化配列を使ってバイトデータを高速に生成
        buffer = np.zeros(len(points), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
        buffer['x'] = points[:, 0]
        buffer['y'] = points[:, 1]
        buffer['z'] = points[:, 2]

        # PLYファイルに色情報がある場合
        if len(colors) == len(points):
            # Open3Dの0.0~1.0の値を0~255に変換
            colors_uint8 = (colors * 255).astype(np.uint32)
            # ビットシフトで1つのuint32にパック (RGB)
            buffer['rgb'] = (colors_uint8[:, 0] << 16) | (colors_uint8[:, 1] << 8) | colors_uint8[:, 2]
        else:
            # 色情報がない場合はデフォルトで白(255, 255, 255)にする
            buffer['rgb'] = (255 << 16) | (255 << 8) | 255

        self.pc2_msg.data = buffer.tobytes()
        self.get_logger().info('PointCloud2 message metadata prepared.')

    def timer_callback(self):
        if hasattr(self, 'pc2_msg'):
            self.pc2_msg.header.stamp = self.get_clock().now().to_msg()
            self.pc2_msg.header.frame_id = 'map'
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