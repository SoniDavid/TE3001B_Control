from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_type',       default_value='PD',
                              description='CTC | PD | PID'),
        DeclareLaunchArgument('perturbation_enabled',  default_value='false',
                              description='true | false'),
        DeclareLaunchArgument('csv_dir',               default_value='data',
                              description='Directory for CSV/metadata output'),
        DeclareLaunchArgument('loop_trajectory',       default_value='false',
                              description='Loop trajectory after completion'),
        DeclareLaunchArgument('trajectory_mode',       default_value='pcb',
                      description='pcb | step'),
        DeclareLaunchArgument('step_axis',             default_value='x',
                      description='Step axis for trajectory_mode=step: x | y | z'),
        DeclareLaunchArgument('step_size_m',           default_value='0.010',
                      description='Step amplitude in meters for trajectory_mode=step'),
        DeclareLaunchArgument('step_hold_before_s',    default_value='2.0',
                      description='Initial hold before the step (seconds)'),
        DeclareLaunchArgument('step_hold_after_s',     default_value='6.0',
                      description='Final hold after the step (seconds)'),
        DeclareLaunchArgument('joint_states_topic',    default_value='/joint_states',
                              description='Joint states topic (/joint_states ~10Hz or /ufactory/joint_states ~150Hz)'),

        Node(
            package='xarm_ctc_challenge',
            executable='challenge_controller',
            name='challenge_controller',
            output='screen',
            parameters=[{
                'controller_type':      LaunchConfiguration('controller_type'),
                'perturbation_enabled': LaunchConfiguration('perturbation_enabled'),
                'csv_dir':              LaunchConfiguration('csv_dir'),
                'loop_trajectory':      LaunchConfiguration('loop_trajectory'),
                'trajectory_mode':      LaunchConfiguration('trajectory_mode'),
                'step_axis':            ParameterValue(LaunchConfiguration('step_axis'), value_type=str),
                'step_size_m':          LaunchConfiguration('step_size_m'),
                'step_hold_before_s':   LaunchConfiguration('step_hold_before_s'),
                'step_hold_after_s':    LaunchConfiguration('step_hold_after_s'),
                'joint_states_topic':   LaunchConfiguration('joint_states_topic'),
            }],
        ),
    ])
