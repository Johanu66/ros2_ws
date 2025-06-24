import os
import subprocess
import yaml # type: ignore
import xml.etree.ElementTree as ET

def generate_universal_controller_config(robot_data, output_path=None):
    """
    Generates a universal controller configuration for a robot arm
    based on its joint structure, following the correct ROS 2 controller structure.
    
    Args:
        robot_data: Dictionary containing robot information
        output_path: Path to save the generated YAML file
        
    Returns:
        dict: Controller configuration
    """
    # Find the main arm joints (usually revolute or prismatic joints)
    arm_joints = [j["name"] for j in robot_data["joints"] 
                 if j["type"] in ["revolute", "prismatic", "continuous"] 
                 and not ("gripper" in j["name"].lower() or "finger" in j["name"].lower())]
    
    # Find gripper joints
    gripper_joints = [j["name"] for j in robot_data["joints"] 
                     if j["type"] in ["prismatic", "revolute"] 
                     and ("gripper" in j["name"].lower() or "finger" in j["name"].lower())]
    
    # Create base controller config with correct structure
    config = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 1000,  # Hz
                "use_sim_time": True,
                
                # List the controllers in controller_manager section
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                
                "arm_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        }
    }
    
    # Add gripper controller if gripper joints exist
    if gripper_joints:
        config["controller_manager"]["ros__parameters"]["gripper_controller"] = {
            "type": "position_controllers/GripperActionController"
        }
    
    # Define arm controller parameters as a top-level entry
    config["arm_controller"] = {
        "ros__parameters": {
            "joints": arm_joints,
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"],
            "state_publish_rate": 100.0,
            "action_monitor_rate": 20.0,
            "allow_partial_joints_goal": False,
            "constraints": {
                "stopped_velocity_tolerance": 0.05,
                "goal_time": 0.5
            }
        }
    }
    
    # Add gripper controller parameters if needed
    if gripper_joints:
        if len(gripper_joints) == 1:
            # Single joint gripper
            config["gripper_controller"] = {
                "ros__parameters": {
                    "joint": gripper_joints[0],
                    "allow_stalling": True,
                    "default": True
                }
            }
        else:
            # Multi-joint gripper - select the first one as main joint
            main_gripper_joint = gripper_joints[0]
            config["gripper_controller"] = {
                "ros__parameters": {
                    "joint": main_gripper_joint,
                    "allow_stalling": True,
                    "default": True
                }
            }
    
    # Save to file if output path is specified
    if output_path:
        with open(output_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    
    return config

def add_ros2_control_to_urdf(urdf_str, hardware_plugin="gazebo_ros2_control/GazeboSystem"):
    """
    Add ros2_control tags to URDF based on robot joints.
    
    Args:
        urdf_str: URDF as string
        hardware_plugin: Hardware plugin to use
        
    Returns:
        str: Modified URDF string with ros2_control tags
    """
    root = ET.fromstring(urdf_str)
    robot_name = root.get('name', 'robot')
    
    # Find all joints
    joints = []
    for joint in root.findall('.//joint'):
        joint_type = joint.get('type')
        joint_name = joint.get('name')
        if joint_type in ['revolute', 'prismatic', 'continuous'] and joint_name:
            joints.append(joint_name)
    
    # Create ros2_control element
    ros2_control = ET.Element('ros2_control', {
        'name': f"{robot_name}_control",
        'type': "system"
    })
    
    # Add hardware plugin
    hardware = ET.SubElement(ros2_control, 'hardware')
    plugin = ET.SubElement(hardware, 'plugin')
    plugin.text = hardware_plugin
    
    # Add joint interfaces
    for joint_name in joints:
        joint_elem = ET.SubElement(ros2_control, 'joint', {'name': joint_name})
        
        # Command interface
        cmd_interface = ET.SubElement(joint_elem, 'command_interface', {'name': 'position'})
        
        # State interfaces
        state_pos = ET.SubElement(joint_elem, 'state_interface', {'name': 'position'})
        state_vel = ET.SubElement(joint_elem, 'state_interface', {'name': 'velocity'})
    
    # Add ros2_control to robot
    root.append(ros2_control)
    
    # Add gazebo plugin for ros2_control
    gazebo = ET.Element('gazebo')
    gz_plugin = ET.SubElement(gazebo, 'plugin', {
        'name': 'gazebo_ros2_control',
        'filename': 'libgazebo_ros2_control.so'
    })
    
    # Create temporary file path for the controller config
    config_path = f"/tmp/{robot_name}_controllers.yaml"
    parameters = ET.SubElement(gz_plugin, 'parameters')
    parameters.text = config_path
    
    root.append(gazebo)
    
    # Return modified URDF
    return ET.tostring(root, encoding='unicode')




def add_description_packages_to_gz_path():
    """
    Finds all installed ROS packages that include 'description' in their name
    and adds their paths to the GZ_SIM_RESOURCE_PATH environment variable.
    """
    # Get the list of all ROS packages
    try:
        # This works for ROS 2
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True, check=True)
        packages = result.stdout.strip().split('\n')
    except (subprocess.SubprocessError, FileNotFoundError):
        try:
            # Fallback for ROS 1
            result = subprocess.run(['rospack', 'list'], capture_output=True, text=True, check=True)
            packages = [line.split()[0] for line in result.stdout.strip().split('\n')]
        except (subprocess.SubprocessError, FileNotFoundError):
            print("Error: Unable to list ROS packages. Make sure ROS is sourced properly.")
            return False
    
    # Filter packages that include 'description' in their name
    description_packages = [pkg for pkg in packages if 'description' in pkg.lower()]
    
    if not description_packages:
        print("No description packages found.")
        return False
    
    # Get the paths for these packages
    package_paths = []
    for pkg in description_packages:
        try:
            # For ROS 2
            result = subprocess.run(['ros2', 'pkg', 'prefix', pkg], 
                                   capture_output=True, text=True, check=True)
            # Add the share directory to the path
            pkg_path = os.path.join(result.stdout.strip(), 'share')
            if os.path.exists(pkg_path):
                package_paths.append(pkg_path)
        except (subprocess.SubprocessError, FileNotFoundError):
            try:
                # Fallback for ROS 1
                result = subprocess.run(['rospack', 'find', pkg], 
                                       capture_output=True, text=True, check=True)
                pkg_path = result.stdout.strip()
                if os.path.exists(pkg_path):
                    package_paths.append(pkg_path)
            except subprocess.SubprocessError:
                print(f"Could not find path for package: {pkg}")
    
    # Get existing GZ_SIM_RESOURCE_PATH
    gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Split the path into components
    gz_paths = gz_path.split(':') if gz_path else []
    
    # Add new paths if not already present
    added_paths = []
    for path in package_paths:
        if path not in gz_paths:
            gz_paths.append(path)
            added_paths.append(path)
    
    # Create the new path string
    new_gz_path = ':'.join(filter(None, gz_paths))
    
    # Update the environment variable
    os.environ['GZ_SIM_RESOURCE_PATH'] = new_gz_path
    
    # Print feedback
    if added_paths:
        print(f"Paths added to GZ_SIM_RESOURCE_PATH.")
    else:
        print("No new paths needed to be added.")
    
    return True


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
    

def replace_hardware_plugin_for_simulation(robot_description_content):
    """
    Replaces any hardware plugin in the ros2_control section with the Gazebo simulation plugin.
    
    Args:
        robot_description_content (str): URDF/XML content
        
    Returns:
        str: Modified URDF/XML content
    """
    import re
    
    # Define regex pattern to find any plugin tag within ros2_control > hardware section
    pattern = r'(<ros2_control[^>]*>.*?<hardware>.*?)<plugin>.*?</plugin>(.*?</hardware>.*?</ros2_control>)'
    
    # Replace with the Gazebo simulation plugin
    replacement = r'\1<plugin>gz_ros2_control/GazeboSimSystem</plugin>\2'
    
    # Apply the replacement
    modified_content = re.sub(pattern, replacement, robot_description_content, flags=re.DOTALL)
    
    return modified_content