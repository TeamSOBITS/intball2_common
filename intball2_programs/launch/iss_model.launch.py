import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('intball2_programs')

    # ISSのURDF
    urdf_path = os.path.join(pkg_dir, 'urdf', 'iss.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # RViz2の設定
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        # ISSの状態（TF）を配信するノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='iss_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 50.0
            }]
        ),
        # RViz2の起動
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_base_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'dock_body']
        # )
    ])