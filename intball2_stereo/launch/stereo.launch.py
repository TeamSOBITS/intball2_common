from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_node(context):
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
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'precision', default_value='false',
            description='Use the precision parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock'),
        OpaqueFunction(function=launch_node),
    ])
