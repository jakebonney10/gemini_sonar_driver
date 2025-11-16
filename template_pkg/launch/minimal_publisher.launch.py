import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    config = os.path.join(
        get_package_share_directory('template_pkg'),
        'config',
        'minimal_publisher.yaml'
    )

    node = Node(
        package='template_pkg',
        name='minimal_publisher',
        executable='minimal_publisher',
        parameters=[
            config,
            {'log_level': LaunchConfiguration('log_level')}
        ]
    )
    
    ld.add_action(log_level_arg)
    ld.add_action(node)
    return ld
