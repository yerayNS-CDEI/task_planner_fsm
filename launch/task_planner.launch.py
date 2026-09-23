from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # GPR fake-encoder bridge (see the Node below). Off by default so sim
        # runs and bench tests don't pay for a node that has nothing to talk to;
        # the FSM tab of the UI sets both when its "GPR trigger bridge" box is
        # ticked, and the terminal equivalent is
        #   ros2 launch task_planner_fsm task_planner.launch.py \
        #       gpr_trigger_bridge:=true gpr_receiver_ip:=<ESP32 IP>
        DeclareLaunchArgument(
            'gpr_trigger_bridge', default_value='false',
            description='Start the gpr_trigger_bridge node (/gpr/trigger -> UDP -> ESP32).',
        ),
        DeclareLaunchArgument(
            'gpr_receiver_ip', default_value='192.168.1.166',
            description='ESP32 address on Oliwall_2G (DHCP reservation); required when '
                        'gpr_trigger_bridge is true.',
        ),
        DeclareLaunchArgument(
            'gpr_receiver_port', default_value='5005',
            description='UDP port the ESP32 sketch listens on.',
        ),
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
        # Real robot only, hence gated on gpr_trigger_bridge:=true above; run the
        # FSM with -p gpr_trigger_bridge_required:=true alongside it so a sweep
        # never starts on a dead link. Defaults reproduce the calibrated handheld
        # (ESP32/GPR_TX_01): fake wheel encoder, 16 quadrature cycles per cm,
        # 800 us half period; cycles per trigger follow trigger_distance_m,
        # which must equal the FSM's gpr_trigger_distance_m.
        Node(
            package='task_planner_fsm',
            executable='gpr_trigger_bridge',
            name='gpr_trigger_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gpr_trigger_bridge')),
            parameters=[{
                'receiver_ip': ParameterValue(
                    LaunchConfiguration('gpr_receiver_ip'), value_type=str),
                'receiver_port': ParameterValue(
                    LaunchConfiguration('gpr_receiver_port'), value_type=int),
                'mode': 1,
                'encoder_cycles_per_cm': 16.0,
                'trigger_distance_m': 0.005,
                'half_period_us': 800,
            }],
        ),
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
