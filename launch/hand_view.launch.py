import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_RVIZ = {
    'base':    {'left': 'base_left.rviz',  'right': 'base_right.rviz'},
    'orca':    {'left': 'orca.rviz',        'right': 'orca.rviz'},
    'robotis': {'left': 'robotis.rviz',     'right': 'robotis.rviz'},
}

_URDF = {
    'base':    {'left': 'left.urdf',           'right': 'right.urdf'},
    'orca':    {'left': 'left.urdf',            'right': 'right.urdf'},
    'robotis': {'left': 'hx5_d20_left.urdf',   'right': 'hx5_d20_right.urdf'},
}

def _launch_setup(context, *args, **kwargs):
    model = context.launch_configurations['model']
    side  = context.launch_configurations['side']
    pkg_share = get_package_share_directory('atlas_hand')

    urdf_path = os.path.join(pkg_share, 'models', model, 'urdf', _URDF[model][side])
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_desc = robot_desc.replace('package://atlas_hand/', f'file://{pkg_share}/')

    

    rviz_config = _RVIZ.get(model, {}).get(side, 'config.rviz')
    rviz_path = os.path.join(pkg_share, 'models', model, 'rviz', rviz_config)

    return [
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
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='base',
            choices=['base', 'orca', 'robotis'],
            description='Hand model (base / orca / robotis)',
        ),
        DeclareLaunchArgument(
            'side',
            default_value='left',
            choices=['left', 'right'],
            description='Hand side (left / right)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
