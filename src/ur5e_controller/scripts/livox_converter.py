#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import struct
import yaml
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

class LivoxPointCloudConverter(Node):
    def __init__(self):
        super().__init__('livox_converter_node')
        
        # Get package directories
        try:
            self.package_share_dir = get_package_share_directory('ur5e_controller')
        except Exception as e:
            self.package_share_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '..')
            self.get_logger().warning(f'Package not found in ament index, using relative path: {self.package_share_dir}')
        
        # Default config path relative to the package
        default_config_path = os.path.join(self.package_share_dir, 'config', 'lidar_transform.yaml')
        
        # Declare parameters
        self.declare_parameter('livox_custom_topic', '/livox/lidar')
        self.declare_parameter('point_cloud_topic', '/livox/point_cloud')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'livox_frame')
        self.declare_parameter('config_path', default_config_path)
        
        # Get parameters
        self.livox_custom_topic = self.get_parameter('livox_custom_topic').get_parameter_value().string_value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        # Load transformation parameters (no longer used for direct transformation)
        self.transform_params = self.load_transform_config()
        
        # Set up publishers and subscribers
        self.pc_publisher = self.create_publisher(PointCloud2, self.point_cloud_topic, 10)
        self.livox_subscriber = self.create_subscription(
            CustomMsg,
            self.livox_custom_topic,
            self.livox_callback,
            10
        )
        
        self.get_logger().info(f'Livox converter initialized. Converting from {self.livox_custom_topic} to {self.point_cloud_topic}')
        self.get_logger().info(f'Publishing in the {self.source_frame} frame - TF system will handle transformations')
    
    def load_transform_config(self):
        """Load transformation parameters from YAML config file"""
        try:
            if not os.path.exists(self.config_path):
                self.get_logger().error(f'Config file not found: {self.config_path}')
                return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            if 'lidar_transform' not in config:
                self.get_logger().error('No lidar_transform section found in config file')
                return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                
            return config['lidar_transform']
        except Exception as e:
            self.get_logger().error(f'Error loading config file: {e}')
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    
    def livox_callback(self, msg):
        """Process incoming Livox custom messages and convert to PointCloud2"""
        # Convert to standard PointCloud2
        pc2_msg = self.convert_to_pointcloud2(msg)
        
        # Publish the converted message
        if pc2_msg:
            self.pc_publisher.publish(pc2_msg)
    
    def convert_to_pointcloud2(self, custom_msg):
        """Convert Livox CustomMsg to PointCloud2"""
        num_points = custom_msg.point_num
        
        if num_points == 0:
            self.get_logger().warning('Received empty point cloud, skipping')
            return None
        
        # Create point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Create point cloud data
        points_data = bytearray(num_points * 16)  # x,y,z,intensity each 4 bytes (float32)
        
        # Process each point - no transformation applied here, just copy the points
        for i in range(num_points):
            point = custom_msg.points[i]
            
            # Get coordinates directly from Livox custom point
            x = point.x
            y = point.y
            z = point.z
            
            # Convert reflectivity to intensity (0-1 float range)
            intensity = float(point.reflectivity) / 255.0
            
            # Pack data into binary format
            offset = i * 16
            struct.pack_into('ffff', points_data, offset, x, y, z, intensity)
        
        # Create and return PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = custom_msg.header
        pc2_msg.header.frame_id = self.source_frame  # Use source_frame instead of target_frame
        pc2_msg.height = 1
        pc2_msg.width = num_points
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16  # 4 fields * 4 bytes
        pc2_msg.row_step = pc2_msg.point_step * num_points
        pc2_msg.data = points_data
        pc2_msg.is_dense = True
        
        return pc2_msg

def main(args=None):
    rclpy.init(args=args)
    converter = LivoxPointCloudConverter()
    
    rclpy.spin(converter)
    
    # Clean up
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()