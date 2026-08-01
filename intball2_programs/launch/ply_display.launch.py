import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'intball2_programs'

    # --- 追加: 起動引数(Launch Argument)の宣言 ---
    # ply_file: modelsディレクトリ内のPLYファイル名（ファイル名のみを指定。絶対パス不可）
    ply_file_arg = DeclareLaunchArgument(
        'ply_file',
        default_value='iss_30000.ply',
        description='models ディレクトリ内から読み込むPLYファイル名'
    )

    # processing_device: 点群の座標変換・色計算をCPUで行うかCUDA(CuPy)で行うか
    processing_device_arg = DeclareLaunchArgument(
        'processing_device',
        default_value='cuda',
        description=("点群処理に使うデバイス。'cpu' または 'cuda' を指定。"
                     "'cuda'指定時にCuPyまたはCUDA対応GPUが無い場合は自動的に'cpu'へ"
                     "フォールバックする。")
    )

    # 1. 点群パブリッシャーノード
    ply_publisher_node = Node(
        package=package_name,
        executable='ply_publisher',
        name='ply_publisher',
        output='screen',
        parameters=[{
            'ply_file': LaunchConfiguration('ply_file'),
            'processing_device': LaunchConfiguration('processing_device'),
        }]
    )
    
    # 2. RViz2の設定ファイルパスを取得
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'ply_config.rviz'
    )
    
    # 3. RViz2ノード（設定ファイルを引数に指定）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        ply_file_arg,
        processing_device_arg,
        ply_publisher_node,
        rviz_node
    ])