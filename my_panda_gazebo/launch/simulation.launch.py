#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1
    load_gripper = LaunchConfiguration("load_gripper")
    declare_load_gripper_arg = DeclareLaunchArgument(
        "load_gripper",
        default_value="False",
        description="Use Franka Gripper as end-effector if True",
    )
    
    # 2 In simulation, this flag must be se to true
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
        description="Whether simulation is used",
    )
    
    # 3 RVIZ config file
    rviz_config_file_path = 'visualize_franka.rviz'
    package_name_viz = 'my_panda_viz'
    pkg_share_viz = FindPackageShare(package=package_name_viz).find(package_name_viz)
    default_rviz_config_path = os.path.join(pkg_share_viz, rviz_config_file_path)  
    
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_arg = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
    
    # 4 Use RVIZ
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_arg = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
    # Start arm controller
    start_arm_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'arm_controller'],
        output='screen')
    
    ## Spawn empty world in ignition-gazebo.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    # Launch joint state broadcaster
    start_joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'joint_state_broadcaster'],
        output='screen'
    )
    
    controller_config_name = PythonExpression(
        [
            "'my_panda_controllers.yaml'"
        ]
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_panda_gazebo"),
            "config",
            controller_config_name,
        ]
    )
    
    # Load robot description content that will be used by 'gz_spawn_entity' node below.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_panda_description"),
                    "robots",
                    "panda_arm.urdf.xacro",
                ]
            ),
            " hand:=",
            load_gripper,
            " use_sim:=",
            use_sim,
            " simulation_controllers_config_file:=",
            robot_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # Robot State Publisher
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Launch RViz
    start_rviz_cmd = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       output='screen',
       arguments=['-d', rviz_config_file],
       condition=IfCondition(use_rviz)
    )  
    
    # Spawn the Franka Arm 

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "franka_arm",
            "-allow_renaming",
            "true",
            "-string",
            robot_description_content
        ],
        output="screen",
    )
    
    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
       event_handler=OnProcessExit(
       target_action=gz_spawn_entity ,
       on_exit=[start_joint_state_broadcaster])
    )

    # Launch the arm controller after launching the joint state broadcaster
    load_arm_controller_cmd = RegisterEventHandler(
       event_handler=OnProcessExit(
       target_action=start_joint_state_broadcaster,
       on_exit=[start_arm_controller])
    )

    return LaunchDescription(
        [   declare_load_gripper_arg,
            declare_rviz_config_file_arg,
            declare_use_rviz_arg,
            declare_use_sim_arg,
            SetParameter(name="use_sim", value=True),
            gz_sim, ## Spawn empty world in ignition-gazebo.
            gz_spawn_entity, ## Spawn Franka Emika Robot on the table.
            load_joint_state_broadcaster_cmd,
            robot_state_pub_node,
            start_rviz_cmd,
            load_arm_controller_cmd
        ]
    )
