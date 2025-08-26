#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import csv
import os
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import time


class ParticleSimNode(Node):
    def __init__(self):
        super().__init__('particle_sim_node')
        
        # Declare parameters
        self.declare_parameter('csv_file', 'particles.csv')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('particle_scale', 5.0)  # Size of particles
        self.declare_parameter('trail_length', 50)     # Number of trail points
        self.declare_parameter('auto_play', True)      # Automatically start playback
        self.declare_parameter('show_labels', True)    # Show particle labels
        self.declare_parameter('show_material_colors', True)  # Use material-specific colors
        self.declare_parameter('show_material_types', True)   # Show material types in labels
        self.declare_parameter('use_csv_time', True)  # Use CSV timestamps for ROS messages
        
        # Get parameters
        self.csv_file_path = self.get_parameter('csv_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.particle_scale = self.get_parameter('particle_scale').get_parameter_value().double_value
        self.trail_length = self.get_parameter('trail_length').get_parameter_value().integer_value
        self.auto_play = self.get_parameter('auto_play').get_parameter_value().bool_value
        self.show_labels = self.get_parameter('show_labels').get_parameter_value().bool_value
        self.show_material_colors = self.get_parameter('show_material_colors').get_parameter_value().bool_value
        self.show_material_types = self.get_parameter('show_material_types').get_parameter_value().bool_value
        self.use_csv_time = self.get_parameter('use_csv_time').get_parameter_value().bool_value
        
        # Material type definitions
        self.material_types = {
            'pe_pp_pellet': {
                'name': 'PE/PP Pellet',
                'color': (0.8, 0.6, 0.4),  # Tan/beige
                'scale_factor': 1.0,
                'density': 0.95  # g/cm³
            },
            'eps_bead': {
                'name': 'EPS Bead',
                'color': (1.0, 1.0, 1.0),  # White
                'scale_factor': 0.8,
                'density': 0.05  # g/cm³
            },
            'microbead_pe_pp': {
                'name': 'Microbead PE/PP',
                'color': (0.2, 0.4, 0.8),  # Blue
                'scale_factor': 0.6,
                'density': 0.92  # g/cm³
            },
            'cenosphere': {
                'name': 'Cenosphere',
                'color': (0.5, 0.5, 0.5),  # Gray
                'scale_factor': 1.2,
                'density': 0.7  # g/cm³
            },
            'bubble_foam': {
                'name': 'Bubble Foam',
                'color': (0.9, 0.9, 0.6),  # Light yellow
                'scale_factor': 1.5,
                'density': 0.03  # g/cm³
            }
        }
        
        # Default material assignment method
        self.default_material_assignment = 'round_robin'  # or 'random', 'by_id'
        
        # QoS profile for visualization
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'particle_markers',
            qos_profile
        )
        
        self.trail_pub = self.create_publisher(
            MarkerArray,
            'particle_trails',
            qos_profile
        )
        
        self.shore_pub = self.create_publisher(
            MarkerArray,
            'shore_mesh',
            qos_profile
        )
        
        # Data storage
        self.particle_data = {}  # {particle_id: [(x, y, z, timestamp), ...]}
        self.particle_materials = {}  # {particle_id: material_type}
        self.current_time_step = 0
        self.time_steps = []
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Load CSV data
        self.load_csv_data()
        
        self.get_logger().info(f'Particle Sim Node started')
        self.get_logger().info(f'CSV file: {self.csv_file_path}')
        self.get_logger().info(f'Loaded {len(self.particle_data)} particles with {len(self.time_steps)} time steps')
        
        # Publish the shore mesh once
        self.publish_shore_mesh()
        
    def load_csv_data(self):
        """Load particle position data from CSV file"""
        try:
            if not os.path.exists(self.csv_file_path):
                self.get_logger().error(f'CSV file not found: {self.csv_file_path}')
                self.create_sample_csv()
                return
                
            with open(self.csv_file_path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                
                # Check for required columns - support both formats
                required_columns_format1 = {'particle_id', 'x', 'y', 'timestamp'}
                required_columns_format2 = {'idx', 'x', 'y', 't'}
                
                fieldnames_set = set(reader.fieldnames)
                
                if required_columns_format1.issubset(fieldnames_set):
                    # Standard format
                    id_col, timestamp_col = 'particle_id', 'timestamp'
                elif required_columns_format2.issubset(fieldnames_set):
                    # Alternative format (like sydney_regatta.csv)
                    id_col, timestamp_col = 'idx', 't'
                else:
                    self.get_logger().error(f'CSV must contain either columns: {required_columns_format1} or {required_columns_format2}')
                    self.get_logger().error(f'Found columns: {reader.fieldnames}')
                    return
                
                # Check if material column exists
                has_material_col = 'material' in fieldnames_set or 'material_type' in fieldnames_set
                material_col = 'material' if 'material' in fieldnames_set else 'material_type'
                
                time_steps_set = set()
                
                for row in reader:
                    try:
                        particle_id = int(row[id_col])
                        x = float(row['x'])
                        y = float(row['y'])
                        z = float(row.get('z', 0.0))  # Default z=0 if not provided
                        timestamp = float(row[timestamp_col])
                        
                        if particle_id not in self.particle_data:
                            self.particle_data[particle_id] = []
                        
                        self.particle_data[particle_id].append((x, y, z, timestamp))
                        time_steps_set.add(timestamp)
                        
                        # Assign material if specified in CSV
                        if has_material_col and row.get(material_col):
                            material = row[material_col].strip()
                            if material in self.material_types:
                                self.particle_materials[particle_id] = material
                        
                    except (ValueError, KeyError) as e:
                        self.get_logger().warn(f'Skipping invalid row: {e}')
                
                # Sort time steps and particle data by timestamp
                self.time_steps = sorted(list(time_steps_set))
                for particle_id in self.particle_data:
                    self.particle_data[particle_id].sort(key=lambda x: x[3])
                
                # Assign materials to particles that don't have them
                for particle_id in self.particle_data.keys():
                    if particle_id not in self.particle_materials:
                        self.assign_particle_material(particle_id)
                    
        except Exception as e:
            self.get_logger().error(f'Error loading CSV: {e}')
            self.create_sample_csv()
    
    def create_sample_csv(self):
        """Create a sample CSV file for demonstration"""
        self.get_logger().info('Creating sample CSV file...')
        
        sample_data = []
        num_particles = 5
        num_time_steps = 100
        material_keys = list(self.material_types.keys())
        
        for t in range(num_time_steps):
            timestamp = t * 0.1  # 0.1 second intervals
            
            for particle_id in range(num_particles):
                # Create circular motion for each particle
                radius = (particle_id + 1) * 2.0
                angular_freq = 0.5 + particle_id * 0.1
                
                x = radius * math.cos(angular_freq * timestamp)
                y = radius * math.sin(angular_freq * timestamp)
                z = math.sin(timestamp + particle_id) * 0.5
                
                # Assign material type
                material_type = material_keys[particle_id % len(material_keys)]
                
                sample_data.append({
                    'particle_id': particle_id,
                    'x': x,
                    'y': y,
                    'z': z,
                    'timestamp': timestamp,
                    'material': material_type
                })
        
        # Write sample data
        sample_file = os.path.join(os.path.dirname(self.csv_file_path), 'sample_particles.csv')
        try:
            with open(sample_file, 'w', newline='') as csvfile:
                fieldnames = ['particle_id', 'x', 'y', 'z', 'timestamp', 'material']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(sample_data)
            
            self.get_logger().info(f'Sample CSV created: {sample_file}')
            self.csv_file_path = sample_file
            self.load_csv_data()
            
        except Exception as e:
            self.get_logger().error(f'Failed to create sample CSV: {e}')
    
    def get_particle_positions_at_time(self, timestamp):
        """Get particle positions at a specific timestamp"""
        positions = {}
        
        for particle_id, data_points in self.particle_data.items():
            # Find the closest timestamp
            closest_point = min(data_points, key=lambda x: abs(x[3] - timestamp))
            positions[particle_id] = closest_point[:3]  # (x, y, z)
        
        return positions
    
    def get_particle_trail(self, particle_id, current_timestamp):
        """Get trail points for a particle up to current timestamp"""
        if particle_id not in self.particle_data:
            return []
        
        trail_points = []
        data_points = self.particle_data[particle_id]
        
        # Get points up to current timestamp
        valid_points = [p for p in data_points if p[3] <= current_timestamp]
        
        # Limit to trail_length most recent points
        if len(valid_points) > self.trail_length:
            valid_points = valid_points[-self.trail_length:]
        
        return [p[:3] for p in valid_points]  # Return (x, y, z) tuples
    
    def timer_callback(self):
        """Main timer callback for publishing visualization data"""
        if not self.time_steps or not self.particle_data:
            return
        
        if not self.auto_play:
            return
        
        # Get current timestamp
        if self.current_time_step >= len(self.time_steps):
            self.current_time_step = 0  # Loop back to beginning
        
        current_timestamp = self.time_steps[self.current_time_step]
        
        # Publish current particle positions
        self.publish_particles(current_timestamp)
        
        # Publish particle trails
        self.publish_trails(current_timestamp)
        
        # Republish shore mesh every 100 cycles to ensure it stays visible
        if self.current_time_step % 100 == 0:
            self.publish_shore_mesh()
        
        self.current_time_step += 1
    
    def publish_particles(self, timestamp):
        """Publish current particle positions as markers"""
        marker_array = MarkerArray()
        positions = self.get_particle_positions_at_time(timestamp)
        
        for particle_id, (x, y, z) in positions.items():
            material_type = self.get_particle_material(particle_id)
            material_props = self.get_material_properties(material_type)
            
            # Create sphere marker for particle
            marker = Marker()
            marker.header.frame_id = self.frame_id
            if self.use_csv_time:
                # Use CSV timestamp as authoritative ROS time
                marker.header.stamp = rclpy.time.Time(seconds=timestamp).to_msg()
            else:
                # Use current ROS time
                marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "particles"
            marker.id = particle_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            
            # Scale based on material type
            scale = self.particle_scale * material_props['scale_factor']
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # Color based on material or particle ID
            if self.show_material_colors:
                r, g, b = material_props['color']
            else:
                hue = (particle_id * 60) % 360  # Different hue for each particle
                r, g, b = self.hsv_to_rgb(hue, 1.0, 1.0)
            
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            
            marker.lifetime.sec = 0  # Persistent
            
            marker_array.markers.append(marker)
            
            # Create text marker for label
            if self.show_labels:
                text_marker = Marker()
                text_marker.header.frame_id = self.frame_id
                if self.use_csv_time:
                    # Use CSV timestamp as authoritative ROS time
                    text_marker.header.stamp = rclpy.time.Time(seconds=timestamp).to_msg()
                else:
                    # Use current ROS time
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "particle_labels"
                text_marker.id = particle_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Position (slightly above the particle)
                text_marker.pose.position.x = x
                text_marker.pose.position.y = y
                text_marker.pose.position.z = z + scale * 0.8
                text_marker.pose.orientation.w = 1.0
                
                # Scale (text height)
                text_marker.scale.z = scale * 0.4
                
                # Text content - show particle ID and optionally material type
                if self.show_material_types:
                    text_marker.text = f"P{particle_id}\n{material_type}"
                else:
                    text_marker.text = f"P{particle_id}"
                
                # Color (white text)
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                text_marker.lifetime.sec = 0  # Persistent
                
                marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_trails(self, timestamp):
        """Publish particle trails as line strips"""
        marker_array = MarkerArray()
        
        for particle_id in self.particle_data.keys():
            trail_points = self.get_particle_trail(particle_id, timestamp)
            
            if len(trail_points) < 2:
                continue
            
            marker = Marker()
            marker.header.frame_id = self.frame_id
            if self.use_csv_time:
                # Use CSV timestamp as authoritative ROS time
                marker.header.stamp = rclpy.time.Time(seconds=timestamp).to_msg()
            else:
                # Use current ROS time
                marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trails"
            marker.id = particle_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Position and orientation
            marker.pose.orientation.w = 1.0
            
            # Scale (line width)
            marker.scale.x = self.particle_scale * 0.3
            
            # Color (same as particle but more transparent)
            hue = (particle_id * 60) % 360
            r, g, b = self.hsv_to_rgb(hue, 1.0, 0.8)
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.6
            
            # Points
            for i, (x, y, z) in enumerate(trail_points):
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                marker.points.append(point)
                
                # Gradient color for trail (fade towards the back)
                color = ColorRGBA()
                alpha = (i + 1) / len(trail_points) * 0.6
                color.r = r
                color.g = g
                color.b = b
                color.a = alpha
                marker.colors.append(color)
            
            marker.lifetime.sec = 0  # Persistent
            
            marker_array.markers.append(marker)
        
        self.trail_pub.publish(marker_array)
    
    def publish_shore_mesh(self):
        """Publish the Sydney regatta shore DAE mesh"""
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "shore_mesh"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # Position and orientation
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color
        marker.color.r = 0.6
        marker.color.g = 0.4
        marker.color.b = 0.2
        marker.color.a = 0.8
        
        # Mesh resource - use file:// URI for the DAE file from package share directory
        from ament_index_python.packages import get_package_share_directory
        try:
            package_share_dir = get_package_share_directory('particle_sim')
            dae_path = os.path.join(package_share_dir, 'sydney_regatta_shore.dae')
            marker.mesh_resource = f"file://{dae_path}"
            self.get_logger().info(f"Using mesh file: {marker.mesh_resource}")
        except Exception as e:
            self.get_logger().error(f"Failed to find package share directory: {e}")
            marker.mesh_resource = ""
        
        marker.lifetime.sec = 0  # Persistent
        
        marker_array.markers.append(marker)
        self.shore_pub.publish(marker_array)
        
        self.get_logger().info(f'Published DAE shore mesh: {marker.mesh_resource}')
    
    def hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB color space"""
        h = h / 360.0
        i = int(h * 6.0)
        f = (h * 6.0) - i
        p, q, t = v * (1.0 - s), v * (1.0 - s * f), v * (1.0 - s * (1.0 - f))
        
        i %= 6
        if i == 0:
            return v, t, p
        if i == 1:
            return q, v, p
        if i == 2:
            return p, v, t
        if i == 3:
            return p, q, v
        if i == 4:
            return t, p, v
        if i == 5:
            return v, p, q
    
    def assign_particle_material(self, particle_id, material_type=None):
        """Assign material type to a particle"""
        if material_type and material_type in self.material_types:
            self.particle_materials[particle_id] = material_type
        else:
            # Auto-assign based on default method
            if self.default_material_assignment == 'round_robin':
                material_keys = list(self.material_types.keys())
                self.particle_materials[particle_id] = material_keys[particle_id % len(material_keys)]
            elif self.default_material_assignment == 'random':
                import random
                self.particle_materials[particle_id] = random.choice(list(self.material_types.keys()))
            else:  # by_id
                material_keys = list(self.material_types.keys())
                self.particle_materials[particle_id] = material_keys[min(particle_id, len(material_keys) - 1)]
    
    def get_particle_material(self, particle_id):
        """Get material type for a particle"""
        if particle_id not in self.particle_materials:
            self.assign_particle_material(particle_id)
        return self.particle_materials[particle_id]
    
    def get_material_properties(self, material_type):
        """Get properties for a material type"""
        return self.material_types.get(material_type, self.material_types['pe_pp_pellet'])


def main(args=None):
    rclpy.init(args=args)
    
    node = ParticleSimNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
