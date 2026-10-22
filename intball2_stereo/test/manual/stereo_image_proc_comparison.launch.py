"""Reuse the common 800px pipeline without starting duplicate optical TF publishers.

Full pipeline keeps the common matcher, point cloud and XYZ crop unchanged.
Only simulation clock configuration and optional reuse of existing TF differ.
pointcloud:=false removes both point cloud generation and the downstream crop.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode


def common_entities():
    # The python backend of stereo.launch.py; its topic arguments plus nodes.
    path = Path(get_package_share_directory('intball2_stereo')) / 'launch/stereo.launch.py'
    spec = importlib.util.spec_from_file_location('common_stereo_comparison_source', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = [entity for entity in module.generate_launch_description().entities
                 if isinstance(entity, DeclareLaunchArgument) and entity.name.endswith('_topic')]
    return [*arguments, *module.python_nodes(None)]


def generate_launch_description():
    entities = [
        DeclareLaunchArgument('full', default_value='true',
                              description='Default value for pointcloud; original full pipeline.'),
        DeclareLaunchArgument('pointcloud', default_value=LaunchConfiguration('full'),
                              description='Keep original PointCloudNode and XYZ crop.'),
        DeclareLaunchArgument('reuse_existing_tf', default_value='true',
                              description='Skip only the two common optical static TF publishers.'),
        SetParameter(name='use_sim_time', value=True),
    ]
    containers = 0
    for entity in common_entities():
        if isinstance(entity, ComposableNodeContainer):
            # Humble exposes descriptors only on this private member; fail on API drift.
            attribute = '_ComposableNodeContainer__composable_node_descriptions'
            original = getattr(entity, attribute, None)
            if original is None or len(original) != 2:
                raise RuntimeError('unexpected common stereo component layout')
            components = []
            for component in original:
                plugin = ''.join(part.text for part in component.node_plugin)
                if plugin not in ('stereo_image_proc::DisparityNode', 'stereo_image_proc::PointCloudNode'):
                    raise RuntimeError('unexpected common stereo component plugin')
                components.append(ComposableNode(
                    package=component.package, plugin=component.node_plugin,
                    name=component.node_name, namespace=component.node_namespace,
                    parameters=[*(component.parameters or []), {'use_sim_time': True}],
                    remappings=component.remappings, extra_arguments=component.extra_arguments,
                    condition=IfCondition(LaunchConfiguration('pointcloud'))
                    if plugin.endswith('PointCloudNode') else component.condition()))
            setattr(entity, attribute, components)
            containers += 1
        if isinstance(entity, Node) and not isinstance(entity, ComposableNodeContainer):
            executable = entity.node_executable
            if not isinstance(executable, str):
                executable = ''.join(part.text for part in executable)
            if executable == 'static_transform_publisher':
                entity = GroupAction(actions=[entity],
                                     condition=UnlessCondition(LaunchConfiguration('reuse_existing_tf')))
            elif executable == 'crop_pointcloud':
                entity = GroupAction(actions=[entity],
                                     condition=IfCondition(LaunchConfiguration('pointcloud')))
        entities.append(entity)
    if containers != 1:
        raise RuntimeError('expected one common stereo container')
    return LaunchDescription(entities)
