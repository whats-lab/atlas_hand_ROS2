from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    hand_type_arg = DeclareLaunchArgument(
        'hand_type',
        default_value='both',
        description="실행할 손: 'left' | 'right' | 'both'",
    )

    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='hand_rerun',
        description='로봇 설정',
    )

    hand_type   = LaunchConfiguration('hand_type')
    launch_left  = PythonExpression(["'", hand_type, "' in ['left',  'both']"])
    launch_right = PythonExpression(["'", hand_type, "' in ['right', 'both']"])
    
    robot_type = LaunchConfiguration('robot_config')
    
    return LaunchDescription([
        hand_type_arg,
        robot_config_arg,

        Node(
            package='atlas_hand',
            executable='osc_receiver',
            name='osc_receiver',
            output='screen',
            parameters=[{
                'listen_ip':   '0.0.0.0',
                'server_port': 4040,
                'client_port': 4042,
                'target_ip':   '127.0.0.1',
                'verbose':     False,
            }],
        ),

        Node(
            package='atlas_hand',
            executable='retarget',
            name='retarget_left',
            namespace='left',
            output='screen',
            parameters=[{
                'hand_type': 'left',
                'robot_config': robot_type,
            }],
            condition=IfCondition(launch_left),
        ),

        Node(
            package='atlas_hand',
            executable='retarget',
            name='retarget_right',
            namespace='right',
            output='screen',
            parameters=[{
                'hand_type': 'right',
                'robot_config': robot_type,
            }],
            condition=IfCondition(launch_right),
        ),
    ])
