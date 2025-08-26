from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'max_thrust',
            default_value='250.0',
            description='Maximum thrust for each thruster (Newtons)'
        ),
        DeclareLaunchArgument(
            'boat_length',
            default_value='4.9',
            description='Distance between thrusters (meters)'
        ),
        
        # Velocity controller node
        Node(
            package='usv_control',
            executable='usv_velocity_controller',
            name='usv_velocity_controller',
            output='screen',
            parameters=[{
                'max_thrust': LaunchConfiguration('max_thrust'),
                'boat_length': LaunchConfiguration('boat_length'),
                'thrust_deadband': 0.1,
            }]
        )
    ])
