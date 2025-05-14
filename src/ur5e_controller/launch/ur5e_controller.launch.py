from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, SetEnvironmentVariable, ExecuteProcess
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
    
    # Load lidar transform config
    lidar_transform_file = os.path.join(
        get_package_share_directory('ur5e_controller'),
        'config',
        'lidar_transform.yaml'
    )
    
    with open(lidar_transform_file, 'r') as f:
        lidar_config = yaml.safe_load(f)['lidar_transform']
    
    # Kinematics setup
    kinematics_file = os.path.join(
        get_package_share_directory('ur5e_controller'),
        'config',
        'kinematics.yaml'
    )
    
    # Create the kinematics file if it doesn't exist
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

    # Use values from config file for static transform
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_base_static_transform',
        arguments=[
            str(lidar_config['x']), 
            str(lidar_config['y']), 
            str(lidar_config['z']),
            str(lidar_config['roll']), 
            str(lidar_config['pitch']), 
            str(lidar_config['yaw']),
            lidar_config['parent_frame'], 
            lidar_config['child_frame']
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
            'livox_custom_topic': '/livox/lidar',
            'point_cloud_topic': '/livox/point_cloud',
            'target_frame': lidar_config['parent_frame'],
            'source_frame': lidar_config['child_frame']
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

    return LaunchDescription([
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
    ])
