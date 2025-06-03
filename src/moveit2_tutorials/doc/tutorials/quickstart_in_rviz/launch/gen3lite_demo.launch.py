import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "192.168.1.10",
        "use_fake_hardware": "false",  # Set to 'true' if using fake hardware
        "gripper": "gen3_lite_2f",  # Gripper model
        "gripper_joint_name": "right_finger_bottom_joint",  # Gripper joint name
        "dof": "6",  # Degrees of Freedom
    }

    # Load the robot configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3_lite_gen3_lite_2f",
            package_name="kinova_gen3_lite_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Node to run the move_group action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )


    # Declare the RViz configuration argument
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="kinova_moveit_config_demo.rviz",
        description="RViz configuration file",
    )

    # RViz node for visualization
    rviz_config_file = os.path.join(
        get_package_share_directory("kinova_gen3_lite_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static transform for TF (World to base link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish robot state information
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using mock hardware for trajectory execution
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_lite_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    # Spawn joint state broadcaster and controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gen3_lite_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gen3_lite_2f_gripper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
        ]
    )
