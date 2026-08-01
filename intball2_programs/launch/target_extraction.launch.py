import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'intball2_programs'

    # --- ply_publisher.py 用の起動引数（ply_display.launch.pyと同一）---
    ply_file_arg = DeclareLaunchArgument(
        'ply_file',
        default_value='iss_30000.ply',
        description='models ディレクトリ内から読み込むPLYファイル名'
    )

    processing_device_arg = DeclareLaunchArgument(
        'processing_device',
        default_value='cuda',
        description=("点群処理に使うデバイス。'cpu' または 'cuda' を指定。"
                     "'cuda'指定時にCuPyまたはCUDA対応GPUが無い場合は自動的に'cpu'へ"
                     "フォールバックする。")
    )

    # --- ply_target_extractor.py 用の3D ROI起動引数 ---
    # デフォルト値はclaude.md Step1の例（x:1.0〜2.0, y:-0.5〜0.5, z:0.3〜1.5）
    roi_x_min_arg = DeclareLaunchArgument('roi_x_min', default_value='5.0', description='ROI x最小値[m]')
    roi_x_max_arg = DeclareLaunchArgument('roi_x_max', default_value='7.0', description='ROI x最大値[m]')
    roi_y_min_arg = DeclareLaunchArgument('roi_y_min', default_value='-0.8', description='ROI y最小値[m]')
    roi_y_max_arg = DeclareLaunchArgument('roi_y_max', default_value='0.8', description='ROI y最大値[m]')
    roi_z_min_arg = DeclareLaunchArgument('roi_z_min', default_value='-1.5', description='ROI z最小値[m]')
    roi_z_max_arg = DeclareLaunchArgument('roi_z_max', default_value='0.3', description='ROI z最大値[m]')

    # --- ply_candidate_generator.py 用の起動引数 ---
    # candidate_theta_deg（極角のリスト）は配列型のためlaunch引数化が煩雑になるので、
    # ノード側のデフォルト値[45.0, 90.0, 135.0]をそのまま使用する（変更したい場合は
    # ply_candidate_generator.py のdeclare_parameterのデフォルト値を直接編集するか、
    # 起動後に `ros2 param set` で変更してください）。
    candidate_radius_arg = DeclareLaunchArgument(
        'candidate_radius', default_value='0.7', description='撮影候補地点の半径r[m]')
    candidate_phi_step_deg_arg = DeclareLaunchArgument(
        'candidate_phi_step_deg', default_value='45.0', description='方位角phiのステップ[deg]')

    # 1. 点群パブリッシャーノード（既存のply_publisher.pyをそのまま起動）
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

    # 2. 対象領域抽出ノード（新規）
    #    ROIパラメータはLaunchConfiguration経由だと文字列型になってしまうため、
    #    ParameterValue(..., value_type=float)で明示的にdouble型としてノードへ渡す。
    ply_target_extractor_node = Node(
        package=package_name,
        executable='ply_target_extractor',
        name='ply_target_extractor',
        output='screen',
        parameters=[{
            'roi_x_min': ParameterValue(LaunchConfiguration('roi_x_min'), value_type=float),
            'roi_x_max': ParameterValue(LaunchConfiguration('roi_x_max'), value_type=float),
            'roi_y_min': ParameterValue(LaunchConfiguration('roi_y_min'), value_type=float),
            'roi_y_max': ParameterValue(LaunchConfiguration('roi_y_max'), value_type=float),
            'roi_z_min': ParameterValue(LaunchConfiguration('roi_z_min'), value_type=float),
            'roi_z_max': ParameterValue(LaunchConfiguration('roi_z_max'), value_type=float),
        }]
    )

    # 3. 候補地点生成ノード（新規）
    #    Bounding Boxの周囲に球面状の候補地点を生成し、TFフレームとしてbroadcastする
    ply_candidate_generator_node = Node(
        package=package_name,
        executable='ply_candidate_generator',
        name='ply_candidate_generator',
        output='screen',
        parameters=[{
            'candidate_radius': ParameterValue(LaunchConfiguration('candidate_radius'), value_type=float),
            'candidate_phi_step_deg': ParameterValue(LaunchConfiguration('candidate_phi_step_deg'), value_type=float),
            # candidate_theta_deg はノード側のデフォルト値[45.0, 90.0, 135.0]を使用
        }]
    )

    # 4. RViz2（既存のply_config.rvizを流用。/target_pointcloud, /target_roi_marker,
    #    TFのDisplayは未登録のため、初回起動時はRViz2のGUIから手動で追加してください）
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'ply_config.rviz'
    )

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
        roi_x_min_arg,
        roi_x_max_arg,
        roi_y_min_arg,
        roi_y_max_arg,
        roi_z_min_arg,
        roi_z_max_arg,
        candidate_radius_arg,
        candidate_phi_step_deg_arg,
        ply_publisher_node,
        ply_target_extractor_node,
        ply_candidate_generator_node,
        rviz_node,
    ])