import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from arms_sim.robot_info_extractor import extract_robot_info_with_auto_install
import yaml


def extract_controller_names(yaml_path):
    """
    Extract controller names from a ROS 2 controller configuration YAML file.
    """
    try:
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
            
        controller_names = []
        
        # Check for controller_manager configuration
        if 'controller_manager' in config and 'ros__parameters' in config['controller_manager']:
            controller_params = config['controller_manager']['ros__parameters']
            
            for key, value in controller_params.items():
                if isinstance(value, dict) and 'type' in value:
                    controller_names.append(key)
                    
        # If nothing found, look for top-level controller configurations
        if not controller_names:
            for key in config:
                if key != 'controller_manager' and isinstance(config[key], dict):
                    if 'ros__parameters' in config[key]:
                        controller_names.append(key)
            
        return controller_names
    except Exception as e:
        print(f"Error reading controllers from {yaml_path}: {e}")
        return ["joint_state_broadcaster"]


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() == "true"
    
    world = LaunchConfiguration("world").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    
    # Get package paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_kinova_sim = get_package_share_directory("arms_sim")
    
    # Calculate full paths
    urdf_path = os.path.join(pkg_kinova_sim, "urdf", urdf_file)
    controllers_path = os.path.join(pkg_kinova_sim, "config", controllers_file)

    from launch.logging import get_logger
    logger = get_logger("arms_sim")
    robot_info = extract_robot_info_with_auto_install(xacro_path=urdf_path, logger=logger)
    
    # Extract robot name
    robot_name = robot_info["robot_name"]
    print(f"Using robot name: {robot_name}")
    
    # Get controller names from YAML
    controller_names = extract_controller_names(controllers_path)
    print(f"Found controllers: {controller_names}")
    
    # Robot description (Xacro → URDF)
    xacro_file = os.path.join(pkg_kinova_sim, "urdf", urdf_file)
    robot_description_content = Command(["xacro ", xacro_file, " sim_gazebo:=true"])
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

    # Contrôleurs avec les deux approches combinées
    controller_launcher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_path,
            *controller_names  # Utilise la liste extraite des contrôleurs
        ],
        output="screen"
    )

    # Rest of your nodes
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
            {"urdf_path": urdf_path},
            {"use_sim_time": use_sim_time}
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
                on_exit=[controller_launcher]
            )
        ),
        
        camera_bridge,
        image_saver,
        auto_motion_explorer
    ]


def generate_launch_description():
    # Common parameters
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time")
    world_arg = DeclareLaunchArgument("world", default_value="empty_world.sdf", description="World file to load")
    
    # Robot-specific parameters
    controllers_file_arg = DeclareLaunchArgument("controllers_file", default_value="ros2_controllers.yaml", description="Controller configuration file")
    urdf_file_arg = DeclareLaunchArgument("urdf_file", default_value="gen3_lite.urdf.xacro", description="URDF/XACRO file name")
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        controllers_file_arg,
        urdf_file_arg,
        OpaqueFunction(function=launch_setup)
    ])