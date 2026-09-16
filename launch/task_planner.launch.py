from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Shell script — not a ROS node, must use ExecuteProcess
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'ur_client_library', 'start_ursim.sh',
        #         '-m', 'ur10e', '-v', '5.17.3',
        #     ],
        #     output='screen',
        #     name='ursim',
        # ),
        Node(
            package='task_planner_fsm',
            executable='mock_server',
            name='mock_server',
            output='screen',
        ),
        # GPR fake-encoder bridge: /gpr/trigger -> UDP -> ESP32 (ESP32/GPR_RX_FINALE.ino).
        # Real robot only; set receiver_ip to the board's address on Oliwall_2G and
        # run the FSM with gpr_trigger_bridge_required:=true so a sweep never
        # starts on a dead link. Defaults reproduce the calibrated handheld
        # (ESP32/GPR_TX_01): fake wheel encoder, 16 quadrature cycles per cm,
        # 800 us half period; cycles per trigger follow trigger_distance_m,
        # which must equal the FSM's gpr_trigger_distance_m.
        # Node(
        #     package='task_planner_fsm',
        #     executable='gpr_trigger_bridge',
        #     name='gpr_trigger_bridge',
        #     output='screen',
        #     parameters=[{
        #         'receiver_ip': '192.168.1.50',
        #         'receiver_port': 5005,
        #         'mode': 1,
        #         'encoder_cycles_per_cm': 16.0,
        #         'trigger_distance_m': 0.005,
        #         'half_period_us': 800,
        #     }],
        # ),
        # Node(
        #     package='arm_control',
        #     executable='optimal_base_service',
        #     name='optimal_base_service',
        #     output='screen',
        #     parameters=[{'map_relpath': 'resource/rmap.npy'}],
        # ),
        # Node(
        #     package='arm_control',
        #     executable='wall_discretization_node',
        #     name='wall_discretization_node',
        #     output='screen',
        # ),
        # Node(
        #     package='arm_control',
        #     executable='script_command_service_node',
        #     name='script_command_service_node',
        #     output='screen',
        # ),
    ])
