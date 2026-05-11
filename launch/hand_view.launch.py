import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from atlas_hand_core.hand_configs import CONFIG_REGISTRY


def _launch_setup(context):
    model = context.launch_configurations['model']
    side  = context.launch_configurations['side']
    pkg_share = get_package_share_directory('atlas_hand')

    config    = CONFIG_REGISTRY[model]()
    urdf_path = config._get_urdf_path(side)

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
    ]

    rviz_file = config._RVIZ_FILENAME.get(side, '')
    if rviz_file:
        rviz_path = os.path.join(pkg_share, 'models', config._MODEL_SUBDIR, 'rviz', rviz_file)
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='base_hand',
            choices=list(CONFIG_REGISTRY.keys()),
            description='Hand model: ' + ' | '.join(CONFIG_REGISTRY.keys()),
        ),
        DeclareLaunchArgument(
            'side',
            default_value='left',
            choices=['left', 'right'],
            description='Hand side (left / right)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
