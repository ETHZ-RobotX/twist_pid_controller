from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_dir = os.path.join(os.getenv('COLCON_PREFIX_PATH'), 'twist_pid_controller')
    config = os.path.join(package_dir, 'share', 'twist_pid_controller', 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='twist_pid_controller',
            executable='twist_pid_controller',
            name='twist_pid_controller',
            parameters=[config],
            output='screen'
        )
    ])
