import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('osd_bridge')
    default_params = os.path.join(pkg_share, 'parameters', 'osd_bridge_parameters.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sim_boxes', default_value='false',
                              description='two-target box simulator (no detector needed)'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='enable MIPI camera video (false = gray bench frame)'),
        DeclareLaunchArgument('snapframe', default_value='-1',
                              description='dump snapshots at frame N (-1 off)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='osd_bridge',
            executable='osd_bridge_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                # LaunchConfiguration yields strings - coerce to the declared types
                'boxes.sim': ParameterValue(LaunchConfiguration('sim_boxes'), value_type=bool),
                'camera.enabled': ParameterValue(LaunchConfiguration('camera'), value_type=bool),
                'snapshot.frame': ParameterValue(LaunchConfiguration('snapframe'), value_type=int),
            }],
        ),
    ])
