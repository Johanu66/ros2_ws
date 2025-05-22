import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = True
    world = LaunchConfiguration("world")

    # Get package paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_kinova_sim = get_package_share_directory("arms_sim")

    # Robot description (Xacro → URDF)
    xacro_file = os.path.join(pkg_kinova_sim, "urdf", "gen3_lite.urdf.xacro")
    robot_description_content = Command(["xacro ", xacro_file, " sim_gazebo:=true"])
    robot_description = {"robot_description": robot_description_content}

    # Gazebo world
    world_path = PathJoinSubstitution([pkg_kinova_sim, "worlds", world])
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": [world_path, " -r -v 4"]}.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_gen3_lite",
        output="screen",
        arguments=["-name", "gen3_lite", "-string", robot_description_content],
    )

    # Launch ros2_control_node (controller manager)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            os.path.join(pkg_kinova_sim, "config", "ros2_controllers.yaml"),
        ],
    )

    # Controllers to spawn
    controllers = [
        "joint_state_broadcaster",
        "gen3_lite_joint_trajectory_controller",
        "gen3_lite_2f_gripper_controller",
        "twist_controller",
        "fault_controller",
    ]

    spawn_controller_processes = [
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", controller],
            output="screen",
        ) for controller in controllers
    ]

    # Bridge for camera
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/camera/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image"
        ]
    )

    # Optional image_saver node (disabled by default)
    image_saver = Node(
        package="arms_sim",
        executable="image_saver",
        name="image_saver",
        output="screen"
    )

    auto_motion_explorer = Node(
        package="arms_sim",
        executable="auto_motion_explorer",
        name="auto_motion_explorer",
        output="screen"
    )

    # LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("world", default_value="empty_world.sdf", description="World to load"),

        # Step 1: Launch Gazebo
        gz_sim,

        # Step 2: Robot State Publisher
        robot_state_pub,

        # Step 3: Spawn robot into Gazebo
        spawn_robot,

        # Step 4: Start ros2_control only after robot is spawned
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=spawn_robot,
        #         on_exit=[controller_manager],
        #     )
        # ),

        # Step 5: Spawn controllers after ros2_control is up
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=spawn_controller_processes,
            )
        ),

        # Step 6: Camera bridge
        camera_bridge,

        # Step 7: Image saver
        image_saver,

        # Step 8: Auto Motion Explorer
        auto_motion_explorer
    ])
