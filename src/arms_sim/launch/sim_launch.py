import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from arms_sim.robot_info_extractor import RobotInfoExtractor

def extract_controller_names(controllers_file_path):
    """
    Extract controller names from ROS2 controller configuration file.
    
    Handles the typical structure where controllers are defined under
    controller_manager/ros__parameters and have their own configuration
    sections at the top level.
    """
    import yaml # type: ignore
    
    try:
        with open(controllers_file_path, 'r') as file:
            config = yaml.safe_load(file)
        
        if not config:
            print(f"Warning: Empty config file: {controllers_file_path}")
            return default_controllers()
            
        controller_names = []
        
        # Check for the standard controller_manager structure
        if 'controller_manager' in config and 'ros__parameters' in config['controller_manager']:
            # Extract all keys that are not 'update_rate' or other parameters
            controller_params = config['controller_manager']['ros__parameters']
            
            for key, value in controller_params.items():
                # Skip non-controller parameters like update_rate
                if isinstance(value, dict) and 'type' in value:
                    controller_names.append(key)
                    print(f"Found controller: {key} of type {value['type']}")
        
        # If no controllers found, try looking for top-level controller configurations
        if not controller_names:
            for key in config:
                # Skip the controller_manager itself
                if key == 'controller_manager':
                    continue
                    
                # If it's a section with ros__parameters, it's likely a controller
                if isinstance(config[key], dict) and 'ros__parameters' in config[key]:
                    controller_names.append(key)
                    print(f"Found controller from top-level config: {key}")
        
        # If still no controllers found, return defaults
        if not controller_names:
            print(f"Warning: No controllers found in {controllers_file_path}")
            return default_controllers()
            
        # Prioritize controllers:
        # 1. joint_state_broadcaster always first
        # 2. trajectory controllers
        # 3. gripper controllers
        # 4. others
        
        # First ensure joint_state_broadcaster is first
        if 'joint_state_broadcaster' in controller_names:
            controller_names.remove('joint_state_broadcaster')
            final_controllers = ['joint_state_broadcaster']
        else:
            # Add it if it doesn't exist
            final_controllers = ['joint_state_broadcaster']
            print("Added missing joint_state_broadcaster")
            
        # Then prioritize trajectory controllers
        trajectory_controllers = [c for c in controller_names 
                                 if any(term in c.lower() for term in 
                                       ['trajectory', 'position', 'velocity', 'effort', 'joint'])]
        final_controllers.extend(trajectory_controllers)
        
        # Then gripper controllers
        gripper_controllers = [c for c in controller_names 
                              if 'gripper' in c.lower() and c not in final_controllers]
        final_controllers.extend(gripper_controllers)
        
        # Then any remaining controllers
        remaining_controllers = [c for c in controller_names 
                               if c not in final_controllers]
        final_controllers.extend(remaining_controllers)
        
        print(f"Final ordered controller list: {final_controllers}")
        return final_controllers
        
    except Exception as e:
        print(f"Error extracting controller names: {e}")
        return default_controllers()


def default_controllers():
    """Return default controller list if extraction fails."""
    return ["joint_state_broadcaster"]


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    # Convert string to boolean
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
    robot_info = RobotInfoExtractor.extract_robot_info_with_auto_install(xacro_path=urdf_path,logger=logger)
    
    # Extract robot name
    robot_name = robot_info["robot_name"]
    print(f"Using robot name: {robot_name}")
    
    # Extract controller list
    controller_list = extract_controller_names(controllers_path)
    print(f"Using controllers: {controller_list}")

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
        parameters=[robot_description, {"use_sim_time": use_sim_time}],  # Boolean value
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{robot_name}",
        output="screen",
        arguments=["-name", robot_name, "-string", robot_description_content],
    )

    # Spawn controller processes based on extracted controller list
    spawn_controller_processes = []
    for controller in controller_list:
        spawn_controller_processes.append(
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner", controller],
                output="screen",
            )
        )

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

    # Optional image_saver node
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

    return [
        gz_sim,

        # Step 2: Robot State Publisher
        robot_state_pub,

        # Step 3: Spawn robot into Gazebo
        spawn_robot,
        
        # RegisterEventHandler to spawn controllers after robot is spawned
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
