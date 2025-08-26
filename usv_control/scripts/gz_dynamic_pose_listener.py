#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import threading
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

class GzDynamicPoseListener(Node):
    def __init__(self):
        super().__init__('gz_dynamic_pose_listener')
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info("GZ Dynamic Pose Listener started")
        self.get_logger().info("Listening to raw Gazebo topic: /world/sydney_regatta/dynamic_pose/info")
        self.get_logger().info("Filtering for WAMV entities only")
        self.get_logger().info("Publishing WAMV pose as TF: map -> wamv_base_link")
        
        # Start a thread to continuously listen to the Gazebo topic
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_to_gazebo_topic)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        # Counter for message numbering
        self.msg_count = 0
    
    def _listen_to_gazebo_topic(self):
        """Listen to the raw Gazebo topic in a separate thread"""
        try:
            # Use gz topic command to stream the data
            process = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', '/world/sydney_regatta/dynamic_pose/info'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            current_message = []
            in_message = False
            
            while self.running and process.poll() is None:
                line = process.stdout.readline()
                if not line:
                    break
                    
                line = line.strip()
                
                # Detect start of new message (header)
                if line.startswith('header {'):
                    if current_message:  # Process previous message
                        self._process_gazebo_message('\n'.join(current_message))
                    current_message = [line]
                    in_message = True
                elif in_message:
                    current_message.append(line)
                    
            # Process last message if exists
            if current_message:
                self._process_gazebo_message('\n'.join(current_message))
                
        except Exception as e:
            self.get_logger().error(f"Error listening to Gazebo topic: {e}")
    
    def _process_gazebo_message(self, message_text):
        """Process a complete Gazebo message and filter for WAMV entities"""
        self.msg_count += 1
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Dynamic Pose Message #{self.msg_count}")
        self.get_logger().info(f"{'='*60}")
        
        # Parse poses from the message
        poses = self._parse_poses_from_message(message_text)
        
        # Filter for WAMV entities only - exact match for "wamv"
        wamv_poses = []
        for pose_info in poses:
            name = pose_info.get('name', 'unknown')
            if name == 'wamv':
                wamv_poses.append(pose_info)
        
        self.get_logger().info(f"Found {len(wamv_poses)} WAMV entities (name='wamv' exactly) out of {len(poses)} total entities:")
        
        for i, pose_info in enumerate(wamv_poses):
            name = pose_info.get('name', 'unknown')
            entity_id = pose_info.get('id', 'unknown')
            pos = pose_info.get('position', {})
            ori = pose_info.get('orientation', {})
            
            self.get_logger().info(f"  🚢 WAMV Pose {i}: '{name}' (ID: {entity_id}) - *** MAIN WAMV ***")
            self.get_logger().info(f"    Position: x={pos.get('x', 0):.4f}, y={pos.get('y', 0):.4f}, z={pos.get('z', 0):.4f}")
            if ori:
                self.get_logger().info(f"    Orientation: x={ori.get('x', 0):.4f}, y={ori.get('y', 0):.4f}, z={ori.get('z', 0):.4f}, w={ori.get('w', 1):.4f}")
            else:
                self.get_logger().info("    Orientation: Not available, using identity quaternion")
            
            # Publish TF transform for the main WAMV
            self._publish_wamv_tf(pos, ori)
        
        if len(wamv_poses) == 0:
            self.get_logger().warn("⚠️  No WAMV entities with exact name 'wamv' found in this message!")
    
    def _parse_poses_from_message(self, message_text):
        """Simple parser for Gazebo pose messages"""
        poses = []
        lines = message_text.split('\n')
        
        current_pose = {}
        in_pose = False
        current_section = None  # 'position' or 'orientation' or None
        brace_depth = 0
        
        for line in lines:
            line = line.strip()
            
            # Track brace depth to handle nested structures
            if '{' in line:
                brace_depth += line.count('{')
            if '}' in line:
                brace_depth -= line.count('}')
            
            if line.startswith('pose {'):
                current_pose = {}
                in_pose = True
                current_section = None
                brace_depth = 1
            elif line == '}' and in_pose and brace_depth == 0:
                # End of pose block
                if current_pose:
                    poses.append(current_pose)
                current_pose = {}
                in_pose = False
                current_section = None
            elif in_pose:
                if line.startswith('name: '):
                    current_pose['name'] = line.split('"')[1] if '"' in line else line.split(': ')[1]
                elif line.startswith('id: '):
                    current_pose['id'] = line.split(': ')[1]
                elif line.startswith('position {'):
                    current_pose['position'] = {}
                    current_section = 'position'
                elif line.startswith('orientation {'):
                    current_pose['orientation'] = {}
                    current_section = 'orientation'
                elif current_section == 'position' and line.startswith(('x: ', 'y: ', 'z: ')):
                    key = line.split(': ')[0]
                    value = float(line.split(': ')[1])
                    current_pose['position'][key] = value
                elif current_section == 'orientation' and line.startswith(('x: ', 'y: ', 'z: ', 'w: ')):
                    key = line.split(': ')[0]
                    value = float(line.split(': ')[1])
                    current_pose['orientation'][key] = value
                elif line == '}':
                    # End of nested section
                    current_section = None
        
        return poses
    
    def _publish_wamv_tf(self, position, orientation):
        """Publish TF transform for WAMV"""
        try:
            transform = TransformStamped()
            
            # Header
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'  # Parent frame
            transform.child_frame_id = 'wamv_simple'  # Child frame
            
            # Translation
            transform.transform.translation.x = float(position.get('x', 0.0))
            transform.transform.translation.y = float(position.get('y', 0.0))
            transform.transform.translation.z = float(position.get('z', 0.0))
            
            # Rotation (quaternion)
            transform.transform.rotation.x = float(orientation.get('x', 0.0))
            transform.transform.rotation.y = float(orientation.get('y', 0.0))
            transform.transform.rotation.z = float(orientation.get('z', 0.0))
            transform.transform.rotation.w = float(orientation.get('w', 1.0))  # Default to identity
            
            # Send the transform
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish TF: {e}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'listener_thread'):
            self.listener_thread.join(timeout=2.0)

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = GzDynamicPoseListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
