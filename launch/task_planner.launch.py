from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Shell script — not a ROS node, must use ExecuteProcess
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ur_client_library', 'start_ursim.sh',
                '-m', 'ur10e', '-v', '5.17.3',
            ],
            output='screen',
            name='ursim',
        ),
        Node(
            package='task_planner_fsm',
            executable='mock_server',
            name='mock_server',
            output='screen',
        ),
        Node(
            package='arm_control',
            executable='optimal_base_service',
            name='optimal_base_service',
            output='screen',
            parameters=[{'map_relpath': 'resource/rmap.npy'}],
        ),
        Node(
            package='arm_control',
            executable='wall_discretization_node',
            name='wall_discretization_node',
            output='screen',
        ),
        Node(
            package='arm_control',
            executable='script_command_service_node',
            name='script_command_service_node',
            output='screen',
        ),
    ])
