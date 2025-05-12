#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time

class CollisionEnvironmentNode(Node):
    def __init__(self):
        super().__init__('collision_environment_node')
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )
        
        time.sleep(2.0)
        self.setup_environment()
        
    def setup_environment(self):
        self.get_logger().info('Setting up collision environment')
        self.add_floor()
        self.add_safety_wall()
        self.get_logger().info('Collision environment setup complete')
        
    def add_floor(self):
        self.get_logger().info('Adding floor collision object')
        
        floor_id = 'floor'
        dimensions = [3.0, 3.0, 0.02]
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.01
        pose.orientation.w = 1.0
        
        # Grey floor
        color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)
        
        self.add_box_collision(floor_id, dimensions, pose, "base_link", color)
        
    def add_safety_wall(self):
        self.get_logger().info('Adding safety wall behind the robot')
        
        wall_id = 'safety_wall'
        dimensions = [1.5, 0.05, 1.8]
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = -0.25
        pose.position.z = 0.9
        pose.orientation.w = 1.0
        
        # Red safety wall
        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
        
        self.add_box_collision(wall_id, dimensions, pose, "base_link", color)
        
    def add_box_collision(self, object_id, dimensions, pose, frame_id="base_link", color=None):
        # Add collision object to planning scene
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_model_name = "ur5e"  # ensure scene is applied to the ur5e model

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        
        collision_object.primitives = [box]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        
        planning_scene.world.collision_objects = [collision_object]
        
        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info(f'Collision object {object_id} added to planning scene')
        
        # Add visualization marker if color is provided
        if color:
            self.publish_marker(object_id, dimensions, pose, frame_id, color)
    
    def publish_marker(self, object_id, dimensions, pose, frame_id, color):
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "collision_objects"
        marker.id = hash(object_id) % 10000  # Simple hash for unique ID
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set pose
        marker.pose = pose
        
        # Set scale from dimensions
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]
        
        # Set color
        marker.color = color
        
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Visualization marker for {object_id} published')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionEnvironmentNode()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
