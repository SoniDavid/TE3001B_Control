from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            }],
        ),
    ])
