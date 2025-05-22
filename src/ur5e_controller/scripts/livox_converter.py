#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import struct
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from collections import deque
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LivoxPointCloudConverter(Node):
    def __init__(self):
        super().__init__('livox_converter_node')
        
        try:
            self.package_share_dir = get_package_share_directory('ur5e_controller')
        except Exception as e:
            self.package_share_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '..')
            self.get_logger().warning(f'Package not found in ament index, using relative path: {self.package_share_dir}')
        
        # Default config path and override with parameter if provided
        default_config_path = os.path.join(self.package_share_dir, 'config', 'lidar_config.yaml')
        self.declare_parameter('config_path', default_config_path)
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        # Load configuration from YAML
        config = self.load_config()
        converter_config = config.get('converter', {})
        transform_config = config.get('transform', {})
        
        # Set parameters from the configuration
        self.buffer_duration = float(converter_config.get('buffer_duration', 0.5))
        self.publish_rate = float(converter_config.get('publish_rate', 10.0))
        
        # Fixed topic names, not from config
        self.livox_custom_topic = "/livox/lidar"
        self.point_cloud_topic = "/livox/point_cloud"
        
        self.target_frame = transform_config.get('parent_frame', 'base')
        self.source_frame = transform_config.get('child_frame', 'livox_frame')
        
        self.point_buffer = deque()
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pc_publisher = self.create_publisher(PointCloud2, self.point_cloud_topic, 10)
        self.livox_subscriber = self.create_subscription(
            CustomMsg,
            self.livox_custom_topic,
            self.livox_callback,
            qos
        )
        
        self.publish_timer = self.create_timer(1.0/self.publish_rate, self.publish_accumulated_points)
        
        self.get_logger().info(f'Livox converter initialized from config: {self.config_path}')
        self.get_logger().info(f'Converting from {self.livox_custom_topic} to {self.point_cloud_topic}')
        self.get_logger().info(f'Publishing in the {self.source_frame} frame with {self.buffer_duration}s buffer')
        self.get_logger().info(f'Target frame: {self.target_frame}')
    
    def load_config(self):
        try:
            if not os.path.exists(self.config_path):
                self.get_logger().error(f'Config file not found: {self.config_path}')
                return {}
            
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.get_logger().info(f'Loaded configuration from {self.config_path}')
            return config
        except Exception as e:
            self.get_logger().error(f'Error loading config file: {e}')
            return {}
    
    def livox_callback(self, msg):
        if msg.point_num > 0:
            current_time = time.time()
            self.point_buffer.append((current_time, msg))
            self.clean_buffer(current_time)
    
    def clean_buffer(self, current_time):
        cutoff_time = current_time - self.buffer_duration
        
        while self.point_buffer and self.point_buffer[0][0] < cutoff_time:
            self.point_buffer.popleft()
    
    def publish_accumulated_points(self):
        if not self.point_buffer:
            return
        
        total_points = sum(msg.point_num for _, msg in self.point_buffer)
        
        if total_points == 0:
            return
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        points_data = bytearray(total_points * 16)
        reference_header = self.point_buffer[-1][1].header
        
        point_index = 0
        for _, msg in self.point_buffer:
            for i in range(msg.point_num):
                point = msg.points[i]
                
                x = point.x
                y = point.y
                z = point.z
                intensity = float(point.reflectivity) / 255.0
                
                offset = point_index * 16
                struct.pack_into('ffff', points_data, offset, x, y, z, intensity)
                point_index += 1
        
        pc2_msg = PointCloud2()
        pc2_msg.header = reference_header
        pc2_msg.header.frame_id = self.source_frame
        pc2_msg.height = 1
        pc2_msg.width = total_points
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16
        pc2_msg.row_step = pc2_msg.point_step * total_points
        pc2_msg.data = points_data
        pc2_msg.is_dense = True
        
        self.pc_publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    converter = LivoxPointCloudConverter()
    
    rclpy.spin(converter)
    
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()