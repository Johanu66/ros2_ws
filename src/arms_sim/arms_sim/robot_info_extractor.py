# main.py
import rclpy
from rclpy.node import Node
import subprocess
import tempfile
import os
import re
from urdf_parser_py.urdf import URDF
from glob import glob
from arms_sim.package_installer import (
    install_rosdep_package,
    clone_and_build_package,
    install_ros_apt_package
)

from ament_index_python.packages import get_package_prefix
workspace_root = os.path.dirname(os.path.dirname(get_package_prefix('arms_sim')))

def extract_robot_info(xacro_path):
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
    
    print(robot)

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

        return data
    except Exception as e:
        print(f"Failed to extract robot information: {str(e)}")
        return None
    
def extract_robot_info_with_auto_install(xacro_path, logger, auto_install=True):
    """Extract robot info with automatic package installation."""
    try:
        # Try normal extraction first
        urdf_xml = subprocess.check_output(['xacro', xacro_path], stderr=subprocess.PIPE).decode("utf-8")
        if urdf_xml: return extract_robot_info(xacro_path)
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
                    return extract_robot_info(xacro_path)
                
                # Try cloning from GitHub
                if clone_and_build_package(package_name, logger=logger):
                    logger.info(f"Successfully built {package_name} from source")
                    # Try again with the package installed
                    return extract_robot_info(xacro_path)
                
                # Try installing with apt
                if install_ros_apt_package(package_name, logger=logger):
                    logger.info(f"Successfully installed {package_name} from ros apt")
                    # Try again with the package installed
                    return extract_robot_info(xacro_path)
                
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