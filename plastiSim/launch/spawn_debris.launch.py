from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'num_objects',
            default_value='15',
            description='Number of debris objects to spawn'
        ),
        DeclareLaunchArgument(
            'spawn_interval',
            default_value='1.5',
            description='Time interval between spawns (seconds)'
        ),
        DeclareLaunchArgument(
            'max_objects',
            default_value='100',
            description='Maximum total objects in simulation'
        ),
        
        # Debris spawner node
        Node(
            package='plastiSim',
            executable='debris_spawner',
            name='debris_spawner',
            output='screen',
            parameters=[{
                'num_objects': LaunchConfiguration('num_objects'),
                'spawn_interval': LaunchConfiguration('spawn_interval'),
                'max_objects': LaunchConfiguration('max_objects'),
            }]
        )
    ])
