import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('atlas_hand')

    base_dir = os.path.join(pkg_share, 'urdf')

    urdf_path = os.path.join(base_dir, 'left_hand', 'urdf', 'left_hand_rerun.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'left_hand_view.rviz')

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    mesh_base = f"file://{os.path.join(base_dir)}/"
    robot_desc = robot_desc.replace('package://', mesh_base)
    robot_desc = robot_desc.replace('//meshes', '/meshes')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '-1.57', '0', 'world', 'wrist_left_link'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
