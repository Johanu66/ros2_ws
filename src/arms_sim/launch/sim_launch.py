import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from arms_sim.robot_info_extractor import extract_robot_info_with_auto_install, extract_yaml_paths, inject_controller_yaml_to_xacro
from arms_sim.tools import add_description_packages_to_gz_path, extract_controller_names
from launch.logging import get_logger

from arms_sim.tools import (
    generate_universal_controller_config,
    replace_hardware_plugin_for_simulation
)

def manage_controller_config(controllers_default_path, robot_info, xacro_path):
    yaml_paths_from_xacro_file = extract_yaml_paths(xacro_path)

    if yaml_paths_from_xacro_file and len(yaml_paths_from_xacro_file) > 0:
        return yaml_paths_from_xacro_file[0]["resolved_path"]
    else:
        generate_universal_controller_config(robot_info, controllers_default_path)
        if inject_controller_yaml_to_xacro(xacro_path=xacro_path, yaml_path=controllers_default_path, logger=get_logger()):
            return controllers_default_path
        else:
            get_logger().error("Can't generate a controller config file for the robot arm")
            exit()


def launch_setup(context, *args, **kwargs):
    add_description_packages_to_gz_path()

    # Get launch configurations
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() == "true"
    
    world = LaunchConfiguration("world").perform(context)
    
    # Get package paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_kinova_sim = get_package_share_directory("arms_sim")

    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    urdf_path = os.path.join(pkg_kinova_sim, "urdf", urdf_file)

    robot_info = extract_robot_info_with_auto_install(xacro_path=urdf_path, logger=get_logger("arms_sim"))

    # controllers_file = robot_info["gazebo_config_files"][0]["resolved_path"]
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    controllers_path = os.path.join(pkg_kinova_sim, "config", controllers_file)

    controllers_path = manage_controller_config(controllers_default_path=controllers_path, robot_info=robot_info, xacro_path=urdf_path)
    
    # Extract robot name
    robot_name = robot_info["robot_name"]
    print(f"Using robot name: {robot_name}")
    
    # Get controller names from YAML
    controller_names = extract_controller_names(controllers_path)
    print(f"Found controllers: {controller_names}")

    robot_info = extract_robot_info_with_auto_install(xacro_path=urdf_path, logger=get_logger("arms_sim"))
    print(robot_info)
    
    # Robot description (Xacro → URDF)
    xacro_file = os.path.join(pkg_kinova_sim, "urdf", urdf_file)
    # robot_description_content = Command(["xacro ", xacro_file, " sim_gazebo:=true"])
    # robot_description = {"robot_description": robot_description_content}
    
    # Execute xacro command to get robot description
    import subprocess
    xacro_cmd = ["xacro", xacro_file, "sim_gazebo:=true"]
    robot_description_content = subprocess.check_output(xacro_cmd).decode('utf-8')
    
    # Replace hardware plugin
    robot_description_content = replace_hardware_plugin_for_simulation(robot_description_content)
    
    # Use the modified content
    robot_description = {"robot_description": robot_description_content}

    # Gazebo world
    world_path = os.path.join(pkg_kinova_sim, "worlds", world)
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": f"{world_path} -r -v 4"}.items(),
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
        name=f"spawn_{robot_name}",
        output="screen",
        arguments=["-name", robot_name, "-string", robot_description_content],
    )

    # Controller Manager with loaded configuration
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_path,  # Load directly from the YAML file
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen"
        ) for controller in controller_names
    ]

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/camera/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image"
        ]
    )

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
        output="screen",
        parameters=[
            {"urdf_path": urdf_path}
        ]
    )

    return [
        gz_sim,
        robot_state_pub,
        spawn_robot,
        
        # Start controller manager after robot is spawned
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[controller_manager]
            )
        ),
        
        # Start all controllers after controller manager is up
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=controller_spawners
            )
        ),
        
        camera_bridge,
        image_saver,
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_spawners[-1],
                on_exit=auto_motion_explorer
            )
        )
    ]


def generate_launch_description():
    # Common parameters
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time")
    world_arg = DeclareLaunchArgument("world", default_value="empty_world.sdf", description="World file to load")
    
    # Robot-specific parameters
    controllers_file_arg = DeclareLaunchArgument("controllers_file", default_value="universal_arms_ros2_controllers.yaml", description="Controller configuration file")
    urdf_file_arg = DeclareLaunchArgument("urdf_file", default_value="gen3_lite.urdf.xacro", description="URDF/XACRO file name")
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        controllers_file_arg,
        urdf_file_arg,
        OpaqueFunction(function=launch_setup)
    ])