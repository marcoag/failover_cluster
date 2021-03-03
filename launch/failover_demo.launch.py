

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('autostart', default_value=True, description='Auto start for the node and heartbeat'),
        launch_ros.actions.Node(
            package='failover_cluster', 
            executable='linktime_composition', 
            output='screen',
            name='linktime_composition',
            parameters=[{'heartbeat_period': 200,
                            'watchdog_period': 220,
                            'acitive_node': launch.substitutions.LaunchConfiguration('autostart')}])])
