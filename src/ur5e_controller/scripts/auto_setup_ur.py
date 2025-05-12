#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from builtin_interfaces.msg import Duration
import socket
import time

class URRobotSetup(Node):
    def __init__(self):
        super().__init__('ur_robot_setup')
        
        self.robot_ip = self.declare_parameter('robot_ip', '172.31.1.200').get_parameter_value().string_value
        self.max_retries = 3
        self.dashboard_port = 29999  # UR Dashboard Server port

        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.switch_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        
    def setup_robot(self):
        """Complete robot setup process"""
        if self.check_robot_state():
            success = self.load_and_run_program()
            
            if success:
                self.get_logger().info("External control program successfully started. Setting up controllers...")
                
                time_waiting = 0
                while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for controller_manager/list_controllers service...')
                    time_waiting += 1
                    if time_waiting > 10:
                        self.get_logger().error('Controller manager services not available, continuing without controller setup')
                        return
                
                while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for controller_manager/switch_controller service...')
                    time_waiting += 1
                    if time_waiting > 10:
                        self.get_logger().error('Controller manager services not available, continuing without controller setup')
                        return
                
                self.setup_controllers()
            else:
                self.get_logger().error("Failed to start external control program. Skipping controller setup.")
        else:
            self.get_logger().error("Failed to prepare robot (power on/brake release). Setup aborted.")

    def check_robot_state(self):
        """Check robot state and power on/release brakes if needed"""
        self.get_logger().info(f"Checking robot state on {self.robot_ip}")
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                s.connect((self.robot_ip, self.dashboard_port))
                
                welcome_msg = s.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Dashboard server: {welcome_msg}")
                
                s.send(b"robotmode\n")
                robot_mode = s.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Robot mode: {robot_mode}")
                
                powered_on = self.ensure_robot_powered_on(s)
                if not powered_on:
                    return False
                
                brakes_released = self.ensure_brakes_released(s)
                if not brakes_released:
                    return False
                
                return True
                
        except socket.timeout:
            self.get_logger().error("Connection to the robot's dashboard server timed out.")
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to check robot state: {str(e)}")
            return False
    
    def ensure_robot_powered_on(self, socket_conn):
        """Ensure robot is powered on"""
        socket_conn.send(b"robotmode\n")
        robot_mode = socket_conn.recv(1024).decode('utf-8').strip()
        
        if "POWER_OFF" in robot_mode:
            self.get_logger().info("Robot is powered off. Powering on...")
            
            socket_conn.send(b"power on\n")
            response = socket_conn.recv(1024).decode('utf-8').strip()
            self.get_logger().info(f"Power on response: {response}")
            
            if not ("Powering on" in response or "Robot is already powered on" in response):
                self.get_logger().error("Failed to power on the robot.")
                return False
            
            self.get_logger().info("Waiting for robot to power up (10s)...")
            time.sleep(10)
            
            for _ in range(3):
                socket_conn.send(b"robotmode\n")
                robot_mode = socket_conn.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Robot mode after power on: {robot_mode}")
                
                if "IDLE" in robot_mode or "RUNNING" in robot_mode or "POWER_ON" in robot_mode:
                    self.get_logger().info("Robot successfully powered on.")
                    return True
                    
                time.sleep(2)
            
            self.get_logger().error("Robot failed to power on within the expected time.")
            return False
        elif "IDLE" in robot_mode or "RUNNING" in robot_mode or "POWER_ON" in robot_mode:
            self.get_logger().info("Robot is already powered on.")
            return True
        else:
            self.get_logger().info(f"Robot is in mode: {robot_mode}, continuing setup...")
            return True
    
    def ensure_brakes_released(self, socket_conn):
        """Ensure robot brakes are released"""
        socket_conn.send(b"robotmode\n")
        robot_mode = socket_conn.recv(1024).decode('utf-8').strip()
        
        if "IDLE" in robot_mode:
            self.get_logger().info("Robot brakes may be engaged. Attempting to release...")
            
            socket_conn.send(b"brake release\n")
            response = socket_conn.recv(1024).decode('utf-8').strip()
            self.get_logger().info(f"Brake release response: {response}")
            
            if "Brake releasing" in response:
                self.get_logger().info("Waiting for brakes to release (10s)...")
                time.sleep(10)
                
                socket_conn.send(b"robotmode\n")
                robot_mode = socket_conn.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Robot mode after brake release: {robot_mode}")
                
                if "RUNNING" in robot_mode:
                    self.get_logger().info("Brakes successfully released.")
                    return True
                else:
                    self.get_logger().error("Failed to release brakes, robot still in IDLE mode.")
                    return False
            elif "Brake release already requested" in response:
                self.get_logger().info("Brake release already in progress, waiting...")
                time.sleep(5)
                return True
            elif "Brakes cannot be released" in response:
                self.get_logger().error("Cannot release brakes. Check robot for safety issues.")
                return False
            else:
                self.get_logger().error(f"Unexpected response to brake release: {response}")
                return False
        elif "RUNNING" in robot_mode:
            self.get_logger().info("Robot brakes are already released.")
            return True
        else:
            self.get_logger().info(f"Robot is in mode: {robot_mode}, continuing setup...")
            return True

    def load_and_run_program(self):
        """Load and run external_control.urp program"""
        self.get_logger().info(f"Attempting to load and run program on robot {self.robot_ip}")
        
        for attempt in range(self.max_retries):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                s.connect((self.robot_ip, self.dashboard_port))
                
                s.recv(1024)
                
                s.send(b"programState\n")
                prog_state_response = s.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Program state: {prog_state_response}")
                
                if "PLAYING" in prog_state_response or "PAUSED" in prog_state_response:
                    self.get_logger().info("Stopping current program...")
                    s.send(b"stop\n")
                    time.sleep(1)
                    s.recv(1024)
                
                self.get_logger().info("Loading external_control.urp program...")
                s.send(b"load external_control.urp\n")
                response = s.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Load command response: {response}")
                
                if "File not found" in response:
                    self.get_logger().error("The external_control.urp program was not found on the robot.")
                    self.get_logger().info("Please ensure external_control.urp exists in the robot programs folder.")
                    s.close()
                    return False
                
                time.sleep(2)
                
                self.get_logger().info("Running external_control.urp program...")
                s.send(b"play\n")
                response = s.recv(1024).decode('utf-8').strip()
                self.get_logger().info(f"Play command response: {response}")
                
                if "Starting program" in response or "Loading program" in response:
                    self.get_logger().info("Successfully initiated the external_control.urp program.")
                    
                    time.sleep(2)
                    s.send(b"programState\n")
                    prog_state_response = s.recv(1024).decode('utf-8').strip()
                    
                    if "PLAYING" in prog_state_response:
                        self.get_logger().info("Program is running successfully.")
                        s.close()
                        return True
                    elif "Starting program" in prog_state_response:
                        self.get_logger().info("Program is starting, please wait...")
                        time.sleep(2)
                        s.send(b"programState\n")
                        prog_state_response = s.recv(1024).decode('utf-8').strip()
                        
                        if "PLAYING" in prog_state_response:
                            self.get_logger().info("Program started successfully.")
                            s.close()
                            return True
                        else:
                            self.get_logger().warning(f"Unexpected program state: {prog_state_response}, retrying...")
                    else:
                        self.get_logger().warning(f"Program state after play: {prog_state_response}, retrying...")
                        
                s.close()
                
                self.get_logger().info(f"Attempt {attempt+1}/{self.max_retries} failed. Waiting before retry...")
                time.sleep(2)
                
            except socket.timeout:
                self.get_logger().error("Connection to the robot's dashboard server timed out.")
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"Failed to load and run program: {str(e)}")
                time.sleep(2)
        
        self.get_logger().error(f"Failed to start program after {self.max_retries} attempts.")
        return False

    def setup_controllers(self):
        """Handle controller setup"""
        try:
            self.get_logger().info("Stopping conflicting controllers...")
            self.stop_conflicting_controllers()
            
            self.get_logger().info("Waiting for controllers to stop...")
            time.sleep(2.0)
            
            self.get_logger().info("Activating scaled_joint_trajectory_controller...")
            success = self.activate_controller('scaled_joint_trajectory_controller')
            
            if success:
                self.get_logger().info("Successfully activated scaled_joint_trajectory_controller")
            else:
                self.get_logger().error("Failed to activate scaled_joint_trajectory_controller")
                
        except Exception as e:
            self.get_logger().error(f"Error during controller setup: {str(e)}")

    def list_controllers(self):
        """Get list of all controllers and states"""
        req = ListControllers.Request()
        future = self.list_controllers_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().controller
        else:
            self.get_logger().error('Failed to call list_controllers service')
            return []
    
    def stop_conflicting_controllers(self):
        """Stop controllers that might conflict"""
        controllers = self.list_controllers()
        
        controllers_to_stop = []
        
        safe_controllers = [
            'joint_state_broadcaster',
            'speed_scaling_state_broadcaster',
            'force_torque_sensor_broadcaster',
            'tcp_pose_broadcaster',
            'io_and_status_controller',
            'ur_configuration_controller',
            'scaled_joint_trajectory_controller'
        ]
        
        for controller in controllers:
            if controller.name in safe_controllers:
                continue
                
            if controller.state == 'active':
                self.get_logger().info(f"Found active controller: {controller.name}")
                controllers_to_stop.append(controller.name)
        
        if controllers_to_stop:
            self.get_logger().info(f"Stopping conflicting controllers: {controllers_to_stop}")
            self.switch_controllers(stop_controllers=controllers_to_stop)
            time.sleep(1.0)
            return True
        
        return False
    
    def activate_controller(self, controller_name):
        """Activate a specific controller"""
        controllers = self.list_controllers()
        for controller in controllers:
            if controller.name == controller_name:
                if controller.state == 'active':
                    self.get_logger().info(f"Controller {controller_name} is already active")
                    return True
                elif controller.state == 'inactive':
                    self.get_logger().info(f"Activating controller: {controller_name}")
                    return self.switch_controllers(start_controllers=[controller_name])
                else:
                    self.get_logger().warn(f"Controller {controller_name} is in state {controller.state}, cannot activate")
                    return False
        
        self.get_logger().error(f"Controller {controller_name} not found")
        return False
    
    def switch_controllers(self, start_controllers=None, stop_controllers=None):
        """Switch controllers using service"""
        if start_controllers is None:
            start_controllers = []
        if stop_controllers is None:
            stop_controllers = []
            
        req = SwitchController.Request()
        # Use the new field names instead of deprecated ones
        req.activate_controllers = start_controllers
        req.deactivate_controllers = stop_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        
        timeout = Duration()
        timeout.sec = 5
        timeout.nanosec = 0
        req.timeout = timeout
        
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info(f"Successfully switched controllers: Start={start_controllers}, Stop={stop_controllers}")
                return True
            else:
                self.get_logger().error(f"Failed to switch controllers: Start={start_controllers}, Stop={stop_controllers}")
                return False
        else:
            self.get_logger().error('Failed to call switch_controller service')
            return False

def main(args=None):
    rclpy.init(args=args)
    setup = URRobotSetup()
    
    try:
        setup.setup_robot()
        setup.get_logger().info("Robot setup completed.")
    except KeyboardInterrupt:
        setup.get_logger().info("Keyboard interrupt, shutting down...")
    except Exception as e:
        setup.get_logger().error(f"Setup failed with error: {str(e)}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
