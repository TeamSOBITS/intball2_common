from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# Default stereo backend: 'cpp' or 'python' 
# Can be overridden at launch time with backend:=cpp|python.
STEREO_BACKEND = 'cpp'


def cpp_nodes(context):
    precision = IfCondition(LaunchConfiguration('precision')).evaluate(context)
    filename = 'stereo_precision_params.yaml' if precision else 'stereo_params.yaml'
    params_file = Path(get_package_share_directory('intball2_stereo')) / 'config' / filename
    return [Node(
        package='intball2_stereo',
        executable='stereo_node',
        name='stereo_node',
        output='screen',
        parameters=[
            str(params_file),
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)},
            # Override the YAML so downstream nodes get the same /stereo/points2 as the python backend
            {'points.publish_points': ParameterValue(
                LaunchConfiguration('publish_points'), value_type=bool)},
            {'topics.points': LaunchConfiguration('points_topic')},
        ],
    )]


def python_nodes(context):
    return [
        # stereo_image_proc の DisparityNode と PointCloudNode を
        # 同じコンテナで起動して通信コストを最小化する
        ComposableNodeContainer(
            name='stereo_container',
            namespace='stereo',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # 視差画像を計算するノード
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    namespace='stereo',
                    remappings=[
                        ('left/image_rect',   LaunchConfiguration('left_image_topic')),
                        ('right/image_rect',  LaunchConfiguration('right_image_topic')),
                        ('left/camera_info',  LaunchConfiguration('left_info_topic')),
                        ('right/camera_info', LaunchConfiguration('right_info_topic')),
                    ],
                    parameters=[{
                        'approximate_sync': True,
                        'stereo_algorithm': 1,
                        'prefilter_size': 9,
                        'prefilter_cap': 12,
                        'correlation_window_size': 15,
                        'min_disparity': -98,
                        'disparity_range': 128,
                        'uniqueness_ratio': 5.0,
                        'texture_threshold': 10,
                        'speckle_size': 375,
                        'speckle_range': 10,
                        'P1': 200.0,
                        'P2': 800.0,
                        'disp12_max_diff': 67,
                    }],
                ),
                # 視差画像から PointCloud2 を生成するノード
                # 出力: /stereo/points2
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    namespace='stereo',
                    remappings=[
                        ('left/image_rect_color', LaunchConfiguration('left_image_topic')),
                        ('left/camera_info',      LaunchConfiguration('left_info_topic')),
                        ('right/camera_info',     LaunchConfiguration('right_info_topic')),
                    ],
                    parameters=[{
                        'approximate_sync': True,
                    }],
                ),
            ],
            output='screen',
        ),
        Node(
            package='intball2_programs',
            executable='fix_camera_info',
            output='screen',
        ),
        Node(
            package='intball2_programs',
            executable='crop_pointcloud',
            output='screen',
        ),
    ]


def optical_frame_tf_nodes():
    # Both backends publish point clouds in camera*_optical_frame
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708',
                '--frame-id', f'camera{side}_link',
                '--child-frame-id', f'camera{side}_optical_frame',
            ],
            output='screen',
        )
        for side in ('L', 'R')
    ]


def launch_nodes(context):
    backend = LaunchConfiguration('backend').perform(context)
    if backend == 'cpp':
        return cpp_nodes(context) + optical_frame_tf_nodes()
    if backend == 'python':
        return python_nodes(context) + optical_frame_tf_nodes()
    raise RuntimeError(f"backend must be 'cpp' or 'python', got '{backend}'")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value=STEREO_BACKEND,
            choices=['cpp', 'python'],
            description='Stereo backend: cpp (stereo_node) or python (stereo_image_proc)'),
        # cpp backend
        DeclareLaunchArgument(
            'precision', default_value='false',
            description='[cpp] Use the precision parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='[cpp] Use the simulation clock'),
        DeclareLaunchArgument(
            'publish_points', default_value='true',
            description='[cpp] Publish the XYZ point cloud'),
        DeclareLaunchArgument(
            'points_topic', default_value='/stereo/points2',
            description='[cpp] Point cloud output topic (same as the python backend)'),
        # python backend
        DeclareLaunchArgument(
            'left_image_topic', default_value='/camera_left/image_raw',
            description='[python] Left camera image topic'),
        DeclareLaunchArgument(
            'right_image_topic', default_value='/camera_right/image_raw',
            description='[python] Right camera image topic'),
        DeclareLaunchArgument(
            'left_info_topic', default_value='/camera_left/camera_info_fixed',
            description='[python] Left camera camera_info topic'),
        DeclareLaunchArgument(
            'right_info_topic', default_value='/camera_right/camera_info_fixed',
            description='[python] Right camera camera_info topic'),
        OpaqueFunction(function=launch_nodes),
    ])
