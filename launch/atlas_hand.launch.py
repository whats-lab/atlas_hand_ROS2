from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    hand_type_arg = DeclareLaunchArgument(
        'hand_type',
        default_value='both',
        description="실행할 손: 'left' | 'right' | 'both'",
    )

    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='base_hand',
        description='로봇 설정',
    )

    hand_type    = LaunchConfiguration('hand_type')
    launch_left  = OrSubstitution(EqualsSubstitution(hand_type, 'left'),  EqualsSubstitution(hand_type, 'both'))
    launch_right = OrSubstitution(EqualsSubstitution(hand_type, 'right'), EqualsSubstitution(hand_type, 'both'))
    
    robot_type = LaunchConfiguration('robot_config')
    
    input_source_arg = DeclareLaunchArgument(
        'input_source',
        default_value='atlas',
        description="입력 소스: 'atlas' | 'meta_quest'",
    )

    return LaunchDescription([
        hand_type_arg,
        robot_config_arg,
        input_source_arg,

        Node(
            package='atlas_hand',
            executable='input_receiver',
            name='input_receiver',
            output='screen',
            parameters=[{
                'input_source': LaunchConfiguration('input_source'),
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
