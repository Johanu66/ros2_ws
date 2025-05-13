import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler



def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Package paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_kinova_sim = get_package_share_directory("kinova_gen3_lite_sim")

    # Xacro file for the robot description
    xacro_file = os.path.join(pkg_kinova_sim, "urdf", "gen3_lite_gen3_lite_2f.xacro")
    robot_description_content = Command(["xacro ", xacro_file, " sim_gazebo:=true"])
    robot_description = {"robot_description": robot_description_content}


    # World file path
    world_path = PathJoinSubstitution([
        pkg_kinova_sim,
        "worlds",
        LaunchConfiguration("world")
    ])

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])

    # Gazebo launch (gz sim)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": [world_path, " -r -v 4"]}.items(),
    )

    # State publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Robot spawn
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_gen3_lite",
        output="screen",
        arguments=["-name", "gen3_lite", "-string", robot_description_content],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            os.path.join(pkg_kinova_sim, "config", "ros2_controllers.yaml")
        ],
        output="screen"
    )


    # Bridge for logical camera
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/camera/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image"
        ]
    )

    # Image saver node
    # image_saver = Node(
    #     package="kinova_gen3_lite_sim",
    #     executable="image_saver",
    #     name="image_saver",
    #     output="screen"
    # )

    # Return LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("world", default_value="empty_world.sdf", description="Name of the world SDF file"),
        gz_sim,
        robot_state_pub,
        TimerAction(period=3.0, actions=[spawn_robot]),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[controller_manager],
            )
        ),

        # Delay before spawning controllers to ensure robot is spawned
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager"
                    ],
                    shell=True
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2 run controller_manager spawner gen3_lite_joint_trajectory_controller --controller-manager /controller_manager"
                    ],
                    shell=True
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2 run controller_manager spawner gen3_lite_2f_gripper_controller --controller-manager /controller_manager"
                    ],
                    shell=True
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2 run controller_manager spawner twist_controller --controller-manager /controller_manager"
                    ],
                    shell=True
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2 run controller_manager spawner fault_controller --controller-manager /controller_manager"
                    ],
                    shell=True
                ),
            ]
        ),

        camera_bridge,
        # image_saver
    ])
