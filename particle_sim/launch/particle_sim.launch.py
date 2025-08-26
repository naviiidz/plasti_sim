#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('particle_sim')
    
    # Launch arguments
    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path',
        default_value=os.path.join(pkg_dir, 'config', 'particles.csv'),
        description='Path to CSV file containing particle data'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for particle visualization'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    particle_scale_arg = DeclareLaunchArgument(
        'particle_scale',
        default_value='0.2',
        description='Scale/size of particle markers'
    )
    
    trail_length_arg = DeclareLaunchArgument(
        'trail_length',
        default_value='50',
        description='Number of trail points to display'
    )
    
    auto_play_arg = DeclareLaunchArgument(
        'auto_play',
        default_value='true',
        description='Automatically start playback'
    )
    
    # Node
    particle_sim_node = Node(
        package='particle_sim',
        executable='particle_sim_node.py',
        name='particle_sim_node',
        parameters=[{
            'csv_file_path': LaunchConfiguration('csv_file_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'particle_scale': LaunchConfiguration('particle_scale'),
            'trail_length': LaunchConfiguration('trail_length'),
            'auto_play': LaunchConfiguration('auto_play'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        csv_file_arg,
        frame_id_arg,
        publish_rate_arg,
        particle_scale_arg,
        trail_length_arg,
        auto_play_arg,
        particle_sim_node
    ])
