#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
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
        self.attach_gripper()
        # self.add_safety_wall()
        # self.add_left_wall()
        # self.add_right_wall()
        self.get_logger().info('Collision environment setup complete')
        
    def add_floor(self):
        self.get_logger().info('Adding floor collision object')
        
        floor_id = 'floor'
        # Adjusted dimensions to match the U-shape formed by the walls
        # Width matches the back wall (1.3m) + some margin
        # Length is the depth of the side walls (0.9m) + back wall thickness + margin
        dimensions = [1.5, 1.2, 0.05]
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.2  # Centered with the U-shape structure
        pose.position.z = -0.05
        pose.orientation.w = 1.0
        
        color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)
        
        self.add_box_collision(floor_id, dimensions, pose, "base_link", color)
        
    def attach_gripper(self):
        self.get_logger().info('Attaching gripper box to robot')
        
        gripper_id = 'gripper_box'
        dimensions = [0.08, 0.08, 0.148]
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.13
        pose.orientation.w = 1.0
        
        color = ColorRGBA(r=0.0, g=0.2, b=0.8, a=0.7)
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = "tool0"
        collision_object.id = gripper_id
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        
        collision_object.primitives = [box]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "tool0"
        attached_object.object = collision_object
        
        attached_object.touch_links = [
            "tool0", "wrist_3_link", "wrist_2_link", "wrist_1_link", 
            "forearm_link", "upper_arm_link", "ee_link", "tool0_controller", "flange"
        ]
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_model_name = "ur5e"
        planning_scene.robot_state.attached_collision_objects = [attached_object]
        planning_scene.world.collision_objects = []
        
        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info(f'Gripper {gripper_id} attached to robot')
        
        self.publish_marker(gripper_id, dimensions, pose, "tool0", color)

    def add_safety_wall(self):
        self.get_logger().info('Adding safety wall behind the robot')
        
        wall_id = 'safety_wall'
        dimensions = [1.3, 0.05, 1.3]
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = -0.25
        pose.position.z = 0.65
        pose.orientation.w = 1.0
        
        color = ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.6)
        
        self.add_box_collision(wall_id, dimensions, pose, "base_link", color)
        self.get_logger().info('Safety wall added behind the robot')

    def add_left_wall(self):
        self.get_logger().info('Adding left wall to form U-shape')
        
        wall_id = 'left_wall'
        dimensions = [0.05, 0.9, 1.3]
        
        pose = Pose()
        pose.position.x = -0.65
        pose.position.y = 0.2
        pose.position.z = 0.65
        pose.orientation.w = 1.0
        
        color = ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.6)
        
        self.add_box_collision(wall_id, dimensions, pose, "base_link", color)
        self.get_logger().info('Left wall added to form U-shape')

    def add_right_wall(self):
        self.get_logger().info('Adding right wall to form U-shape')
        
        wall_id = 'right_wall'
        dimensions = [0.05, 0.9, 1.3]
        
        pose = Pose()
        pose.position.x = 0.65
        pose.position.y = 0.2
        pose.position.z = 0.65
        pose.orientation.w = 1.0
        
        color = ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.6)
        
        self.add_box_collision(wall_id, dimensions, pose, "base_link", color)
        self.get_logger().info('Right wall added to form U-shape')

    def add_box_collision(self, object_id, dimensions, pose, frame_id="base_link", color=None):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_model_name = "ur5e"

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
        
        if color:
            self.publish_marker(object_id, dimensions, pose, frame_id, color)
    
    def publish_marker(self, object_id, dimensions, pose, frame_id, color):
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "collision_objects"
        marker.id = hash(object_id) % 10000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose = pose
        
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]
        
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
