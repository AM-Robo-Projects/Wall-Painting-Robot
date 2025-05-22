from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    env_var = SetEnvironmentVariable('ROBOT_MODEL_NAME', 'ur5e')
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.31.1.200',
        description='IP address of the UR robot'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Update lidar config file path
    lidar_config_file = os.path.join(
        get_package_share_directory('ur5e_controller'),
        'config',
        'lidar_config.yaml'
    )
    
    # Load the config file only once
    config_data = {}
    transform_config = {}
    gui_enabled = False
    try:
        with open(lidar_config_file, 'r') as f:
            config_data = yaml.safe_load(f)
            
        # Extract all required configuration values
        transform_config = config_data.get('transform', {})
        gui_enabled = config_data.get('wall_detection', {}).get('enable_crop_box_gui', False)
        
        print(f"Loaded lidar configuration from {lidar_config_file}")
        print(f"GUI enabled: {gui_enabled}")
    except Exception as e:
        print(f"Error loading config file: {e}")
    
    kinematics_file = os.path.join(
        get_package_share_directory('ur5e_controller'),
        'config',
        'kinematics.yaml'
    )
    if not os.path.exists(kinematics_file):
        os.makedirs(os.path.dirname(kinematics_file), exist_ok=True)
        with open(kinematics_file, 'w') as f:
            f.write('''ur_manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 1.0
  kinematics_solver_attempts: 5
  position_only_ik: false
''')
    kinematics_exists_msg = LogInfo(
        msg=['Kinematics file status at: ', kinematics_file, 
             ' Exists: ', str(os.path.exists(kinematics_file))]
    )
    common_launch_args = {
        'robot_ip': LaunchConfiguration('robot_ip'),
        'ur_type': 'ur5e',
        'use_fake_hardware': 'false',
    }
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            **common_launch_args,
            'launch_rviz': 'false',
            'initial_joint_controller': 'joint_trajectory_controller',
            'kinematics_config': kinematics_file,
            'load_kinematics': 'true',
        }.items()
    )
    ur5e_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            **common_launch_args,
            'launch_rviz': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'kinematics_config': kinematics_file,
            'load_kinematics': 'true',
            'launch_servo': 'false',
            'disable_octomap': 'true',
        }.items()
    )
    moveit_launch = LogInfo(msg='Launching MoveIt 2 with UR5e robot...')
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_base_static_transform',
        arguments=[
            str(transform_config.get('x', 0.0)), 
            str(transform_config.get('y', 0.0)), 
            str(transform_config.get('z', 0.0)),
            str(transform_config.get('yaw', 0.0)), 
            str(transform_config.get('pitch', 0.0)), 
            str(transform_config.get('roll', 0.0)),
            transform_config.get('parent_frame', 'base'), 
            transform_config.get('child_frame', 'livox_frame')
        ]
    )
    livox_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ])
    )
    livox_converter = Node(
        package='ur5e_controller',
        executable='livox_converter.py',
        name='livox_converter',
        output='screen',
        parameters=[{
            'config_path': lidar_config_file
        }]
    )
    auto_setup = Node(
        package='ur5e_controller',
        executable='auto_setup_ur.py',
        name='auto_setup_ur',
        output='screen',
        parameters=[{'robot_ip': LaunchConfiguration('robot_ip')}]
    )
    collision_environment = Node(
        package='ur5e_controller',
        executable='collision_environment.py',
        name='collision_environment',
        output='screen',
    )
    
    wall_detector = Node(
        package='ur5e_controller',
        executable='wall_detector',
        name='wall_detector',
        output='screen',
        parameters=[{
            'config_path': lidar_config_file
        }]
    )
    
    load_and_setup = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Powering on robot, releasing brakes, loading program and setting up controllers...'),
            auto_setup
        ]
    )
    add_collision_env = TimerAction(
        period=3.0, 
        actions=[
            LogInfo(msg='Setting up collision environment...'),
            collision_environment
        ]
    )
    
    # Define all nodes for the LaunchDescription
    nodes = [
        env_var,
        robot_ip_arg,
        use_sim_time_arg,
        kinematics_exists_msg,
        ur_robot_driver_launch,
        moveit_launch,
        static_transform,
        livox_lidar_launch,
        ur5e_moveit_launch,
        load_and_setup,
        add_collision_env,
        livox_converter,
        wall_detector,
    ]
    
    # Conditionally add the crop box GUI
    if gui_enabled:
        crop_box_gui = Node(
            package='ur5e_controller',
            executable='crop_box_gui.py',
            name='crop_box_gui',
            output='screen',
            parameters=[{
                'config_path': lidar_config_file
            }]
        )
        nodes.append(crop_box_gui)
    
    return LaunchDescription(nodes)
