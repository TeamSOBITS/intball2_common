#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'left_image_topic',
            default_value='/camera_left/image_raw',
        ),
        DeclareLaunchArgument(
            'right_image_topic',
            default_value='/camera_right/image_raw',
        ),
        DeclareLaunchArgument(
            'left_info_topic',
            default_value='/camera_left/camera_info',
        ),
        DeclareLaunchArgument(
            'right_info_topic',
            default_value='/camera_right/camera_info',
        ),

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
                        'min_disparity': -34,
                        'disparity_range': 64,
                        'uniqueness_ratio': 5.0,
                        'texture_threshold': 200,
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
    ])
