from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'twist_pid_controller'
    pkg_share = get_package_share_directory(package_name)
    config = os.path.join(pkg_share, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='twist_pid_controller_node',  
            name='twist_pid_controller',
            parameters=[
                {'use_sim_time': True},
                config
            ],
            output='screen'
        )
    ])
