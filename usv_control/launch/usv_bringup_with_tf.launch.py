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
            'map_frame',
            default_value='map',
            description='Map frame ID for particle simulation'
        ),
        
        DeclareLaunchArgument(
            'robot_frame',
            default_value='wamv/wamv/base_link',
            description='Robot base frame ID from VRX simulation'
        ),
        
        # ROS-Gazebo Bridge for left thruster
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='left_thruster_bridge',
            arguments=[
                '/wamv/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen'
        ),
        
        # ROS-Gazebo Bridge for right thruster
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='right_thruster_bridge',
            arguments=[
                '/wamv/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen'
        ),
        
        # ROS-Gazebo Bridge for cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=[
                '/wamv/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),

        # ROS-Gazebo Bridge for world pose info (contains robot position)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='world_pose_bridge',
            arguments=[
                '/world/sydney_regatta/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
            ],
            remappings=[('/world/sydney_regatta/pose/info', '/world/poses')],
            output='screen'
        ),

        # ROS-Gazebo Bridge for dynamic pose info (for debugging)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='dynamic_pose_bridge',
            arguments=[
                '/world/sydney_regatta/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
            ],
            output='screen'
        ),

        # Simple WAMV Pose Extractor (extracts WAMV pose from world poses)
        Node(
            package='usv_control',
            executable='simple_wamv_pose_extractor.py',
            name='simple_wamv_pose_extractor',
            output='screen'
        ),
        
        # ROS-Gazebo Bridge for cameras
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/middle_right_camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/middle_right_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image', '/wamv/cameras/front_left/image'),
                ('/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/camera_info', '/wamv/cameras/front_left/camera_info'),
                ('/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/image', '/wamv/cameras/front_right/image'),
                ('/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/camera_info', '/wamv/cameras/front_right/camera_info'),
                ('/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/middle_right_camera_sensor/image', '/wamv/cameras/middle_right/image'),
                ('/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/middle_right_camera_sensor/camera_info', '/wamv/cameras/middle_right/camera_info'),
            ],
            output='screen'
        ),
        
        # Velocity controller node
        Node(
            package='usv_control',
            executable='usv_velocity_controller',
            name='usv_velocity_controller',
            output='screen',
            parameters=[{
                'max_thrust': LaunchConfiguration('max_thrust'),
                'boat_length': 4.9,
                'thrust_deadband': 0.1,
            }]
        )
    ])
