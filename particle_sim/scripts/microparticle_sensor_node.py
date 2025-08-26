#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import math
import json
import numpy as np
import collections


class MicroparticleSensorNode(Node):
    def __init__(self):
        super().__init__('microparticle_sensor_node')
        
        # Declare parameters
        self.declare_parameter('robot_frame', 'wamv_simple')  # Robot TF frame
        self.declare_parameter('detection_radius', 10.0)       # Detection radius around robot
        self.declare_parameter('frame_id', 'map')             # Reference frame ID
        
        # Get parameters
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.detection_radius = self.get_parameter('detection_radius').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Pre-calculate squared detection radius for faster comparisons
        self.detection_radius_sq = self.detection_radius ** 2
        
        # Initialize robot position (will be updated from TF)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        
        # TF caching variables for performance
        self.robot_position_valid = False
        self.last_tf_update = 0.0
        self.tf_cache_duration = 0.05  # Cache TF for 50ms
        
        # Material type mapping (same as particle_sim_node)
        self.material_types = {
            'pe_pp_pellet': 'PE/PP Pellet',
            'eps_bead': 'EPS Bead',
            'microbead_pe_pp': 'Microbead PE/PP',
            'cenosphere': 'Cenosphere',
            'bubble_foam': 'Bubble Foam'
        }
        
        # Detection tracking
        self.detected_particles = set()  # Track already detected particles
        self.detection_history = collections.deque(maxlen=100)  # Fixed size queue for efficiency
        
        # Throttled logging variables
        self.detection_count = 0
        self.last_log_time = 0.0
        self.log_interval = 1.0  # Log summary every 1 second
        self.recent_detections = []
        
        # Pre-allocate material type list for faster lookup
        self.material_list = list(self.material_types.values())
        
        # TF buffer and listener for robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Optimized QoS profile for speed
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Faster than RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest message
        )
        
        # Subscriber to particle markers
        self.marker_sub = self.create_subscription(
            MarkerArray,
            'particle_markers',
            self.particle_callback,
            qos_profile
        )
        
        # Publisher for detection events
        self.detection_pub = self.create_publisher(
            String,
            'particle_detections',
            qos_profile
        )
        
        self.get_logger().info(f'Microparticle Sensor Node started')
        self.get_logger().info(f'Robot frame: {self.robot_frame}')
        self.get_logger().info(f'Detection radius: {self.detection_radius:.2f} meters')
        self.get_logger().info('Waiting for particle markers and TF transforms...')
        
        # Create timer for periodic TF updates instead of per-callback
        self.tf_timer = self.create_timer(0.05, self.update_robot_position_cached)  # 20Hz
    
    def update_robot_position_cached(self):
        """Update robot position periodically (20Hz) instead of per-message"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, rclpy.time.Time()
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            self.robot_z = transform.transform.translation.z
            self.robot_position_valid = True
            self.last_tf_update = self.get_clock().now().nanoseconds / 1e9
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.robot_position_valid = False
    
    def get_robot_position(self):
        """Get robot position from TF transform"""
        try:
            # Get transform from map frame to robot frame
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,      # target frame (map)
                self.robot_frame,   # source frame (wamv_simple)
                rclpy.time.Time()   # latest available
            )
            
            # Update robot position from transform
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            self.robot_z = transform.transform.translation.z
            
            return True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # TF transform not available yet
            return False
    
    def particle_callback(self, msg):
        """Process incoming particle markers and detect collisions using vectorized operations"""
        # Use cached robot position instead of TF lookup every time
        if not self.robot_position_valid:
            return
            
        # Extract all particle data at once for vectorized processing
        particle_data = []
        for marker in msg.markers:
            if marker.ns == "particles":
                particle_data.append([
                    marker.id,
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ])
        
        if len(particle_data) == 0:
            return
            
        # Convert to NumPy arrays for vectorized operations
        particles = np.array(particle_data)
        particle_ids = particles[:, 0].astype(int)
        positions = particles[:, 1:4]  # x, y, z coordinates
        
        # Robot position as array
        robot_pos = np.array([self.robot_x, self.robot_y, self.robot_z])
        
        # Vectorized distance calculation (ALL particles at once!)
        diff = positions - robot_pos  # Broadcasting: (N, 3) - (3,) = (N, 3)
        distances_sq = np.sum(diff**2, axis=1)  # Sum along xyz axis
        
        # Find all particles within detection radius (vectorized comparison)
        in_range_mask = distances_sq <= self.detection_radius_sq
        detected_indices = np.where(in_range_mask)[0]
        
        # Track current particles for cleanup
        current_particles = set(particle_ids)
        
        # Process detected particles
        for idx in detected_indices:
            particle_id = int(particle_ids[idx])  # Convert numpy int64 to Python int
            if particle_id not in self.detected_particles:
                self.detected_particles.add(particle_id)
                
                # Only calculate actual distance when needed for logging
                actual_distance = float(np.sqrt(distances_sq[idx]))  # Convert to Python float
                
                # Extract material type
                material_type = self.extract_material_type_fast(particle_id)
                
                # Get particle position
                particle_x, particle_y, particle_z = float(positions[idx][0]), float(positions[idx][1]), float(positions[idx][2])
                
                # Log detection (throttled)
                self.log_detection_throttled(particle_id, material_type, actual_distance, 
                                           particle_x, particle_y, particle_z)
                
                # Publish detection event
                self.publish_detection(particle_id, material_type, actual_distance)
        
        # Remove particles that are no longer in range or no longer exist
        particles_to_remove = []
        for detected_id in self.detected_particles:
            if detected_id not in current_particles:
                particles_to_remove.append(detected_id)
        
        for particle_id in particles_to_remove:
            self.detected_particles.remove(particle_id)
    
    def calculate_distance_squared(self, x1, y1, z1, x2, y2, z2):
        """Calculate squared distance (faster - no sqrt needed for comparison)"""
        return (x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2
    
    def calculate_distance(self, x1, y1, z1, x2, y2, z2):
        """Calculate 3D Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    
    def extract_material_type_fast(self, particle_id):
        """Optimized material type extraction using pre-allocated list"""
        return self.material_list[particle_id % len(self.material_list)]
    
    def extract_material_type(self, particle_id, marker):
        """Extract material type from particle marker"""
        # For now, we'll use a simple assignment based on particle ID
        # This matches the round_robin assignment in particle_sim_node
        material_keys = list(self.material_types.keys())
        material_key = material_keys[particle_id % len(material_keys)]
        return self.material_types[material_key]
    
    def log_detection_throttled(self, particle_id, material_type, distance, x, y, z):
        """Efficient throttled logging to reduce I/O overhead"""
        self.detection_count += 1
        
        # Store detection data (fast in-memory operation)
        detection = {
            'id': particle_id,
            'material': material_type,
            'distance': distance,
            'pos': (x, y, z)
        }
        self.recent_detections.append(detection)
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Only log summary periodically (not every detection)
        if current_time - self.last_log_time >= self.log_interval:
            if self.recent_detections:
                # Log summary of recent detections
                latest = self.recent_detections[-1]
                summary_msg = (
                    f"🔍 PARTICLE DETECTIONS: {len(self.recent_detections)} new particles | "
                    f"Latest: ID:{latest['id']}, {latest['material']}, "
                    f"{latest['distance']:.3f}m | Total: {self.detection_count}"
                )
                self.get_logger().info(summary_msg)
                
                # Clear recent detections
                self.recent_detections.clear()
                self.last_log_time = current_time
        
        # Store in detection history
        detection_record = {
            'timestamp': self.get_clock().now().to_msg(),
            'particle_id': particle_id,
            'material_type': material_type,
            'distance': distance,
            'position': {'x': x, 'y': y, 'z': z},
            'robot_position': {'x': self.robot_x, 'y': self.robot_y, 'z': self.robot_z}
        }
        
        self.detection_history.append(detection_record)
    
    def log_detection(self, particle_id, material_type, distance, x, y, z):
        """Log particle detection with details"""
        detection_msg = (
            f"🔍 PARTICLE DETECTED! "
            f"ID: {particle_id}, "
            f"Material: {material_type}, "
            f"Distance: {distance:.3f}m, "
            f"Position: ({x:.2f}, {y:.2f}, {z:.2f})"
        )
        
        self.get_logger().info(detection_msg)
        
        # Store in detection history
        detection_record = {
            'timestamp': self.get_clock().now().to_msg(),
            'particle_id': particle_id,
            'material_type': material_type,
            'distance': distance,
            'position': {'x': x, 'y': y, 'z': z},
            'robot_position': {'x': self.robot_x, 'y': self.robot_y, 'z': self.robot_z}
        }
        
        self.detection_history.append(detection_record)
        
        # Keep only last 100 detections
        if len(self.detection_history) > 100:
            self.detection_history.pop(0)
    
    def publish_detection(self, particle_id, material_type, distance):
        """Publish detection event as JSON message"""
        detection_data = {
            'particle_id': particle_id,
            'material_type': material_type,
            'distance': distance,
            'robot_position': {
                'x': self.robot_x,
                'y': self.robot_y,
                'z': self.robot_z
            },
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        msg = String()
        msg.data = json.dumps(detection_data)
        self.detection_pub.publish(msg)
    
    def get_detection_stats(self):
        """Get detection statistics"""
        total_detections = len(self.detection_history)
        
        if total_detections == 0:
            return "No particles detected yet."
        
        # Count by material type
        material_counts = {}
        for detection in self.detection_history:
            material = detection['material_type']
            material_counts[material] = material_counts.get(material, 0) + 1
        
        stats_msg = f"Total detections: {total_detections}\n"
        stats_msg += "Material breakdown:\n"
        for material, count in material_counts.items():
            stats_msg += f"  - {material}: {count}\n"
        
        return stats_msg


def main(args=None):
    rclpy.init(args=args)
    
    node = MicroparticleSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down microparticle sensor...")
        node.get_logger().info("Final detection statistics:")
        node.get_logger().info(node.get_detection_stats())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
