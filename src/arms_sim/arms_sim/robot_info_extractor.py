# main.py
import rclpy
from rclpy.node import Node
import subprocess
import tempfile
import os
import re
import xml.etree.ElementTree as ET
from urdf_parser_py.urdf import URDF
from glob import glob
from arms_sim.package_installer import (
    install_rosdep_package,
    clone_and_build_package,
    install_ros_apt_package
)

from ament_index_python.packages import get_package_prefix
workspace_root = os.path.dirname(os.path.dirname(get_package_prefix('arms_sim')))


def extract_yaml_paths(xacro_path, logger=None, auto_install=True):
    """
    Converts a Xacro file to URDF, parses the model with urdf_parser_py,
    and returns useful information as a dictionary.
    
    Args:
        xacro_path (str): Path to the xacro file
        logger: Logger object for messages
        auto_install (bool): Whether to auto-install missing packages
        
    Returns:
        dict: Robot information extracted from URDF or None if an error occurred
    """
    # 1. Convert Xacro to URDF XML
    try:
        # Check if the file exists
        if not os.path.isfile(xacro_path):
            if logger:
                logger.error(f"File not found: {xacro_path}")
            else:
                print(f"File not found: {xacro_path}")
            return None
            
        # Convert xacro to URDF
        urdf_str = subprocess.check_output(['xacro', xacro_path]).decode('utf-8')
        
        # Add Gazebo plugin parameters extraction for YAML files
        gazebo_yaml_files = []
        root = ET.fromstring(urdf_str)
        
        # Find all gazebo tags
        for gazebo in root.findall('.//gazebo'):
            # Find all plugins within gazebo tags
            for plugin in gazebo.findall('.//plugin'):
                # Find parameter tags
                for param in plugin.findall('.//parameters'):
                    if param.text and '.yaml' in param.text:
                        # Extract the parameter file path
                        param_path = param.text.strip()
                        
                        # Extract package name if the path uses $(find) format
                        match = re.search(r'\$\(find\s+([^\)]+)\)', param_path)
                        if match:
                            package_name = match.group(1)
                            # Try to find/install the package
                            try:
                                # Check if package exists using ros2 pkg
                                subprocess.check_output(['ros2', 'pkg', 'prefix', package_name], stderr=subprocess.PIPE)
                                package_installed = True
                            except subprocess.CalledProcessError:
                                package_installed = False
                                
                            if not package_installed and auto_install:
                                if logger:
                                    logger.info(f"Attempting to install missing package: {package_name}")
                                # Try installing with rosdep
                                try:
                                    # Try installing with rosdep first
                                    if install_rosdep_package(package_name, logger=logger):
                                        logger.info(f"Successfully installed {package_name} from rosdep")
                                    # Try cloning from GitHub
                                    elif clone_and_build_package(package_name, logger=logger):
                                        logger.info(f"Successfully built {package_name} from source")
                                    # Try installing with apt
                                    elif install_ros_apt_package(package_name, logger=logger):
                                        logger.info(f"Successfully installed {package_name} from ros apt")
                                except Exception as e:
                                    if logger:
                                        logger.error(f"Failed to install {package_name}: {str(e)}")
                            
                            # Resolve the full path
                            try:
                                pkg_path = subprocess.check_output(['ros2', 'pkg', 'prefix', package_name]).decode().strip()
                                resolved_path = param_path.replace(f'$(find {package_name})', f'{pkg_path}/share/{package_name}')
                                gazebo_yaml_files.append({
                                    'package': package_name,
                                    'original_path': param_path,
                                    'resolved_path': resolved_path,
                                    'file_type': 'yaml'
                                })
                            except subprocess.CalledProcessError:
                                if logger:
                                    logger.warning(f"Could not resolve path for package: {package_name}")
                                else:
                                    print(f"Could not resolve path for package: {package_name}")
                        else:
                            # Handle absolute or relative paths that don't use $(find)
                            gazebo_yaml_files.append({
                                'package': None,
                                'original_path': param_path,
                                'resolved_path': os.path.abspath(param_path) if not os.path.isabs(param_path) else param_path,
                                'file_type': 'yaml'
                            })
        
        return gazebo_yaml_files
        
    except Exception as e:
        if logger:
            logger.error(f"Failed to extract robot information: {str(e)}")
        else:
            print(f"Failed to extract robot information: {str(e)}")
        return None
    
def inject_controller_yaml_to_xacro(xacro_path, yaml_path, output_path=None, logger=None):
    """
    Injects a controller YAML configuration path into a Xacro file with the proper 
    gz_ros2_control plugin format.
    
    Args:
        xacro_path (str): Path to the original xacro file
        yaml_path (str): Path to the controller YAML file to inject
        output_path (str): Path where to save the modified xacro file (default: overwrite original)
        logger: Logger object for messages
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        if not os.path.isfile(xacro_path):
            if logger:
                logger.error(f"Xacro file not found: {xacro_path}")
            else:
                print(f"Xacro file not found: {xacro_path}")
            return False
        
        # Read the xacro file
        with open(xacro_path, 'r') as file:
            content = file.read()
        
        # Check if the closing </robot> tag exists
        if '</robot>' not in content:
            if logger:
                logger.error(f"No closing </robot> tag found in {xacro_path}")
            else:
                print(f"No closing </robot> tag found in {xacro_path}")
            return False
            
        # Check if a similar gazebo plugin already exists
        gazebo_plugin_pattern = r'<gazebo>\s*<plugin\s+filename="gz_ros2_control-system"\s+name="gz_ros2_control::GazeboSimROS2ControlPlugin">\s*<parameters>[^<]+</parameters>\s*</plugin>\s*</gazebo>'
        
        gazebo_plugin_match = re.search(gazebo_plugin_pattern, content)
        
        # Format the new gazebo plugin element with the provided yaml_path
        new_gazebo_plugin = f'''
            <gazebo>
                <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
                <parameters>{yaml_path}</parameters>
                </plugin>
            </gazebo>
            '''
        
        if gazebo_plugin_match:
            # Replace existing plugin
            content = re.sub(gazebo_plugin_pattern, new_gazebo_plugin.strip(), content)
            action = "Updated"
        else:
            # Insert before closing </robot> tag
            content = content.replace('</robot>', f'{new_gazebo_plugin}</robot>')
            action = "Added"
        
        # Save the modified content
        if output_path is None:
            output_path = xacro_path
            
        with open(output_path, 'w') as file:
            file.write(content)
            
        if logger:
            logger.info(f"{action} controller YAML path '{yaml_path}' in {output_path}")
        else:
            print(f"{action} controller YAML path '{yaml_path}' in {output_path}")
            
        return True
        
    except Exception as e:
        if logger:
            logger.error(f"Failed to inject controller YAML into xacro file: {str(e)}")
        else:
            print(f"Failed to inject controller YAML into xacro file: {str(e)}")
        return False


def extract_robot_info(xacro_path, logger=None):
    """
    Converts a Xacro file to URDF, parses the model with urdf_parser_py,
    and returns useful information as a dictionary.
    
    Args:
        xacro_path (str): Path to the xacro file
        
    Returns:
        dict: Robot information extracted from URDF or None if an error occurred
    """
    # 1. Convert Xacro to URDF XML
    try:
        # Check if the file exists
        if not os.path.isfile(xacro_path):
            print(f"Error: File not found: {xacro_path}")
            return None
            
        # Check if the path ends with .xacro (not required but good to verify)
        if not xacro_path.endswith(('.xacro', '.urdf')):
            print(f"Warning: File might not be a xacro or urdf file: {xacro_path}")
        
        # Use bash to source both the ROS 2 Jazzy setup and workspace setup
        setup_cmd = f"bash -c 'source /opt/ros/jazzy/setup.bash && source {workspace_root}/install/setup.bash && xacro {xacro_path}'"
        process = subprocess.run(setup_cmd, shell=True, capture_output=True, text=True)
        
        if process.returncode != 0:
            print(f"Error running xacro: {process.stderr}")
            return None
            
        urdf_xml = process.stdout
        
    except Exception as e:
        print(f"Failed to process xacro file: {str(e)}")
        return None

    # 2. Parse URDF (from string)
    try:
        robot = URDF.from_xml_string(urdf_xml)
    except Exception as e:
        print(f"Failed to parse URDF: {str(e)}")
        return None

    # 3. Extract information
    try:
        data = {}

        # 3.1 Joints
        joints = []
        for j in robot.joints:
            joint_info = {
                "name": j.name,
                "type": j.type,
                "parent": j.parent,
                "child": j.child,
                "limit": vars(j.limit) if j.limit else None,
                "axis": j.axis,
                "origin": {
                    "xyz": j.origin.xyz if j.origin else None,
                    "rpy": getattr(j.origin, 'rpy', None) if j.origin else None,
                },
            }
            joints.append(joint_info)

        # 3.2 Links
        links = []
        for l in robot.links:
            link_info = {
                "name": l.name,
                "collisions": len(l.collisions or []),
                "visuals": len(l.visuals or []),
                "inertial": vars(l.inertial) if l.inertial else None,
            }
            links.append(link_info)

        # 3.3 End-effectors candidates (children with no further child)
        child_links = set(j['child'] for j in joints)
        parent_links = set(j['parent'] for j in joints)
        terminal_links = list(child_links - parent_links)

        # 4. Assemble final data
        data['robot_name'] = robot.name.replace(' ', '_')
        data['joints'] = joints
        data['links'] = links
        data['joint_names'] = [j['name'] for j in joints]
        data['link_names'] = [l['name'] for l in links]
        data['ee_link_candidates'] = terminal_links
        data['root_link'] = robot.get_root()

        # Add the gazebo yaml files to the returned data
        data["gazebo_config_files"] = extract_yaml_paths(xacro_path, logger=logger)

        return data
    except Exception as e:
        print(f"Failed to extract robot information: {str(e)}")
        return None
    
def extract_robot_info_with_auto_install(xacro_path, logger, auto_install=True):
    """Extract robot info with automatic package installation."""
    try:
        # Try normal extraction first
        urdf_xml = subprocess.check_output(['xacro', xacro_path], stderr=subprocess.PIPE).decode("utf-8")
        if urdf_xml: return extract_robot_info(xacro_path, logger=logger)
    except subprocess.CalledProcessError as e:
        error_output = e.stderr.decode('utf-8') if e.stderr else str(e)
        
        if auto_install and "package" in error_output and "not found" in error_output:
            logger.warning(f"Package error: {error_output}")
            
            # Extract package name from error message
            match = re.search(r"package '([^']+)' not found", error_output)
            
            if match:
                package_name = match.group(1)
                
                logger.info(f"Attempting to install missing package: {package_name}")
                
                # Try installing with rosdep first
                if install_rosdep_package(package_name, logger=logger):
                    logger.info(f"Successfully installed {package_name} from rosdep")
                    # Try again with the package installed
                    return extract_robot_info(xacro_path, logger=logger)
                
                # Try cloning from GitHub
                if clone_and_build_package(package_name, logger=logger):
                    logger.info(f"Successfully built {package_name} from source")
                    # Try again with the package installed
                    return extract_robot_info(xacro_path, logger=logger)
                
                # Try installing with apt
                if install_ros_apt_package(package_name, logger=logger):
                    logger.info(f"Successfully installed {package_name} from ros apt")
                    # Try again with the package installed
                    return extract_robot_info(xacro_path, logger=logger)
                
                logger.error(f"Failed to install {package_name} using any method")
        
        # If auto-install failed or was not attempted, re-raise with better logging
        logger.error(f"Failed to process {xacro_path}: {error_output}")
        raise

class RobotInfoExtractor(Node):
    def __init__(self):
        super().__init__('extractor')

        logger = self.get_logger()
        
        urdf_dir = f"{workspace_root}/src/arms_sim/urdf/"
        # Cherche tous les fichiers xacro et urdf dans le dossier
        files = glob(os.path.join(urdf_dir, '*.xacro')) + glob(os.path.join(urdf_dir, '*.urdf'))

        for file_path in files:
            logger.info(f"Extraction pour le fichier: {file_path}")
            try:
                info = extract_robot_info_with_auto_install(xacro_path=file_path, logger=logger)
                logger.info(f"Successfully extracted info from {file_path}")
                logger.info(str(info))
            except Exception as e:
                logger.error(f"Erreur avec le fichier {file_path}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoExtractor()

    # If you want to spin:
    rclpy.spin(node)

    # After spin returns, shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()