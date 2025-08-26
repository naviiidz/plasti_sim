#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from particle_sim.action import SampleParticles
from particle_sim.msg import DetectedParticle
from visualization_msgs.msg import MarkerArray
import tf2_ros
import numpy as np
import time
from collections import defaultdict


class ParticleSamplingActionServer(Node):
    def __init__(self):
        super().__init__('particle_sampling_action_server')
        
        # Declare parameters
        self.declare_parameter('robot_frame', 'wamv_simple')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('default_detection_radius', 10.0)
        
        # Get parameters
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.default_detection_radius = self.get_parameter('default_detection_radius').get_parameter_value().double_value
        
        # Material type mapping (same as sensor node)
        self.material_types = {
            'pe_pp_pellet': 'PE/PP Pellet',
            'eps_bead': 'EPS Bead',
            'microbead_pe_pp': 'Microbead PE/PP',
            'cenosphere': 'Cenosphere',
            'bubble_foam': 'Bubble Foam'
        }
        self.material_list = list(self.material_types.values())
        
        # Robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_position_valid = False
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber to particle markers
        self.marker_sub = self.create_subscription(
            MarkerArray,
            'particle_markers',
            self.particle_callback,
            qos_profile
        )
        
        # Current particle data for sampling
        self.current_particles = {}
        
        # Action server
        self._action_server = ActionServer(
            self,
            SampleParticles,
            'sample_particles',
            self.sample_particles_callback
        )
        
        # Timer for robot position updates
        self.tf_timer = self.create_timer(0.05, self.update_robot_position)
        
        self.get_logger().info('Particle Sampling Action Server started')
        self.get_logger().info(f'Robot frame: {self.robot_frame}')
        self.get_logger().info(f'Default detection radius: {self.default_detection_radius:.2f} meters')

    def update_robot_position(self):
        """Update robot position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, rclpy.time.Time()
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            self.robot_z = transform.transform.translation.z
            self.robot_position_valid = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.robot_position_valid = False

    def particle_callback(self, msg):
        """Store current particle data"""
        if not self.robot_position_valid:
            return
            
        # Extract particle data
        self.current_particles.clear()
        for marker in msg.markers:
            if marker.ns == "particles":
                self.current_particles[marker.id] = {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z
                }

    def detect_particles_in_range(self, detection_radius):
        """Detect particles within range using vectorized operations (same logic as sensor)"""
        if not self.robot_position_valid or not self.current_particles:
            return []
            
        # Convert to arrays for vectorized processing
        particle_data = []
        for pid, pos in self.current_particles.items():
            particle_data.append([pid, pos['x'], pos['y'], pos['z']])
            
        if len(particle_data) == 0:
            return []
            
        particles = np.array(particle_data)
        particle_ids = particles[:, 0].astype(int)
        positions = particles[:, 1:4]
        
        # Robot position as array
        robot_pos = np.array([self.robot_x, self.robot_y, self.robot_z])
        
        # Vectorized distance calculation
        diff = positions - robot_pos
        distances_sq = np.sum(diff**2, axis=1)
        detection_radius_sq = detection_radius ** 2
        
        # Find particles in range
        in_range_mask = distances_sq <= detection_radius_sq
        detected_indices = np.where(in_range_mask)[0]
        
        # Create detected particle list
        detected = []
        current_time = self.get_clock().now().to_msg()
        
        for idx in detected_indices:
            particle_id = int(particle_ids[idx])
            distance = float(np.sqrt(distances_sq[idx]))
            material_type = self.material_list[particle_id % len(self.material_list)]
            
            detected_particle = DetectedParticle()
            detected_particle.particle_id = particle_id
            detected_particle.material_type = material_type
            detected_particle.distance = distance
            detected_particle.particle_x = float(positions[idx][0])
            detected_particle.particle_y = float(positions[idx][1])
            detected_particle.particle_z = float(positions[idx][2])
            detected_particle.robot_x = self.robot_x
            detected_particle.robot_y = self.robot_y
            detected_particle.robot_z = self.robot_z
            detected_particle.detection_timestamp = float(current_time.sec) + float(current_time.nanosec) / 1e9
            
            detected.append(detected_particle)
            
        return detected

    def sample_particles_callback(self, goal_handle):
        """Action callback for sampling particles"""
        self.get_logger().info('Sampling particles action started')
        
        # Get goal parameters
        sampling_duration = goal_handle.request.sampling_duration
        detection_radius = goal_handle.request.detection_radius
        
        # Use default radius if not specified
        if detection_radius <= 0.0:
            detection_radius = self.default_detection_radius
            
        self.get_logger().info(f'Sampling for {sampling_duration:.1f}s with radius {detection_radius:.1f}m')
        
        # Initialize sampling
        start_time = time.time()
        all_detections = []
        unique_particles = set()
        material_counts = defaultdict(int)
        
        # Feedback message
        feedback_msg = SampleParticles.Feedback()
        
        # Sampling loop
        while True:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Check if sampling duration reached
            if elapsed >= sampling_duration:
                break
                
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Sampling canceled')
                return SampleParticles.Result()
            
            # Detect particles in current frame
            detected = self.detect_particles_in_range(detection_radius)
            
            # Process detections
            for particle in detected:
                if particle.particle_id not in unique_particles:
                    unique_particles.add(particle.particle_id)
                    all_detections.append(particle)
                    material_counts[particle.material_type] += 1
            
            # Send feedback
            feedback_msg.elapsed_time = elapsed
            feedback_msg.current_detections = len(all_detections)
            feedback_msg.progress = elapsed / sampling_duration
            goal_handle.publish_feedback(feedback_msg)
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.01)
        
        # Create result
        result = SampleParticles.Result()
        result.success = True
        result.message = 'Sampling completed successfully'
        result.total_detections = len(all_detections)
        result.unique_particles = len(unique_particles)
        result.actual_duration = elapsed
        result.particles = all_detections
        
        # Log results by material type
        if material_counts:
            self.get_logger().info(f'Sampling completed: {len(all_detections)} particles detected')
            for material, count in material_counts.items():
                self.get_logger().info(f'  - {material}: {count}')
        else:
            self.get_logger().info('Sampling completed: No particles detected')
        
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    
    action_server = ParticleSamplingActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info("Shutting down particle sampling action server...")
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
