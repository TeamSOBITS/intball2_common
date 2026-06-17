import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'intball2_programs'
    
    # 1. 点群パブリッシャーノード
    ply_publisher_node = Node(
        package=package_name,
        executable='ply_publisher',
        name='ply_publisher',
        output='screen'
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
        ply_publisher_node,
        # rviz_node
    ])