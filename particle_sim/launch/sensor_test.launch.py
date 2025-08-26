#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='wamv_simple',
        description='Robot TF frame name'
    )
    
    detection_radius_arg = DeclareLaunchArgument(
        'detection_radius',
        default_value='5.0',
        description='Particle detection radius in meters'
    )

    # Microparticle sensor node
    sensor_node = Node(
        package='particle_sim',
        executable='microparticle_sensor_node.py',
        name='microparticle_sensor_node',
        parameters=[{
            'robot_frame': LaunchConfiguration('robot_frame'),
            'detection_radius': LaunchConfiguration('detection_radius'),
            'frame_id': 'map',
        }],
        output='screen'
    )

    # Static transform publisher for wamv_simple frame (for testing)
    wamv_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wamv_simple_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'wamv_simple']
    )

    return LaunchDescription([
        robot_frame_arg,
        detection_radius_arg,
        wamv_tf_node,
        sensor_node
    ])
