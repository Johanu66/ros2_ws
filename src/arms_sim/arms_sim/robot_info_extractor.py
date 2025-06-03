import rclpy
from rclpy.node import Node
import subprocess
import tempfile
import os
import re
from urdf_parser_py.urdf import URDF
from glob import glob


def install_rosdep_package(package_name, logger=None):
    """Install a missing ROS package using rosdep."""
    try:
        if logger:
            logger.info(f"Trying to install {package_name} using rosdep...")
        
        # First try to resolve and install the package
        result = subprocess.check_call(['rosdep', 'install', '--from-paths', '.', 
                              '--ignore-src', '-r', '-y', package_name],
                              stderr=subprocess.PIPE)
        
        if logger:
            logger.info(f"Successfully installed {package_name} using rosdep")
        return True
    except subprocess.CalledProcessError as e:
        if logger:
            logger.error(f"Failed to install {package_name} using rosdep: {e.stderr.decode('utf-8') if e.stderr else str(e)}")
        return False
    

def clone_and_build_package(package_name, ros_distro="jazzy", logger=None):
    """Clone a ROS package from GitHub and build it by searching for package.xml."""
    try:
        if logger:
            logger.info(f"Trying to clone and build {package_name} from GitHub...")
            
        # Create a directory for source packages if needed
        src_dir = os.path.expanduser("~/ros2_ws/src")
        os.makedirs(src_dir, exist_ok=True)
        os.chdir(src_dir)
        
        # Define known repositories for common packages
        # known_repos = {
        #     'iiwa_description': 'https://github.com/kuka-isir/iiwa_description.git',
        #     'ur_description': 'https://github.com/UniversalRobots/ur_description.git', 
        #     'kortex_description': 'https://github.com/Kinovarobotics/ros2_kortex.git',
        #     'robotiq_description': 'https://github.com/ros-industrial/robotiq.git'
        # }
        known_repos = {}
        
        # Check if we have a known repository for this package
        if package_name in known_repos:
            repo_url = known_repos[package_name]
            if logger:
                logger.info(f"Using known repository for {package_name}: {repo_url}")
        else:
            # Search for package.xml with the right package name using GitHub search API
            import requests
            import base64
            import re
            
            # First search for repositories that might contain our package
            search_queries = [
                f"{package_name} filename:package.xml",
                f"{package_name} ros2 filename:package.xml", 
                f"{package_name} description filename:package.xml",
                f"{package_name} ros filename:package.xml"
            ]
            
            potential_repos = []
            
            for query in search_queries:
                if logger:
                    logger.info(f"Searching GitHub for: {query}")
                
                try:
                    # Search for code containing package.xml
                    code_search_url = f"https://api.github.com/search/code?q={query}"
                    response = requests.get(code_search_url)
                    data = response.json()
                    
                    if 'items' in data:
                        for item in data['items']:
                            if item['name'] == 'package.xml':
                                repo_name = item['repository']['full_name']
                                repo_url = item['repository']['html_url']
                                
                                # Check if this package.xml contains our package name
                                file_url = item['url']
                                file_response = requests.get(file_url)
                                file_data = file_response.json()
                                
                                if 'content' in file_data:
                                    content = base64.b64decode(file_data['content']).decode('utf-8')
                                    # Look for <name>package_name</name> in the content
                                    name_match = re.search(r'<name>\s*(.*?)\s*</name>', content)
                                    
                                    if name_match and name_match.group(1) == package_name:
                                        if logger:
                                            logger.info(f"Found exact package match in {repo_url}")
                                        # We found an exact match, use this repo
                                        return clone_and_build_specific_repo(repo_url, package_name, logger=logger)
                                    
                                    # If not exact match, add to potential repos
                                    potential_repos.append((repo_name, repo_url))
                                
                except Exception as e:
                    if logger:
                        logger.warn(f"GitHub API search failed: {e}")
                    continue
            
            # If we didn't find an exact match but have potential repos
            if potential_repos:
                # Use the first potential repo
                repo_name, repo_url = potential_repos[0]
                if logger:
                    logger.info(f"Using potential repository: {repo_url}")
                return clone_and_build_specific_repo(repo_url, package_name, logger=logger)
            
            # If nothing found, search by repo name as fallback
            if logger:
                logger.warn(f"No package.xml with name={package_name} found, searching by repo")
            
            repo_search_url = f"https://api.github.com/search/repositories?q={package_name}"
            try:
                response = requests.get(repo_search_url)
                data = response.json()
                
                if 'items' in data and len(data['items']) > 0:
                    repo_url = data['items'][0]['clone_url']
                    if logger:
                        logger.info(f"Using repository from name search: {repo_url}")
                    return clone_and_build_specific_repo(repo_url, package_name, logger=logger)
                else:
                    if logger:
                        logger.error(f"No repository found for {package_name}")
                    return False
            except Exception as e:
                if logger:
                    logger.error(f"GitHub API search failed: {e}")
                return False
        
        # If we have a known repo, use it
        return clone_and_build_specific_repo(repo_url, package_name, logger=logger)
            
    except Exception as e:
        if logger:
            logger.error(f"Error building package {package_name}: {str(e)}")
        return False

def clone_specific_repo(repo_url, package_name, logger=None):
    """Clone a specific repo for a ROS package."""
    try:
        # Get source directory
        src_dir = os.path.expanduser("~/ros2_ws/src")
        
        # Choose appropriate directory name for cloning
        target_dir = os.path.join(src_dir, package_name)
        
        if not os.path.exists(target_dir):
            # Clone the repository
            if logger:
                logger.info(f"Cloning {repo_url} to {target_dir}")
            
            subprocess.check_call(['git', 'clone', repo_url, target_dir], 
                                stderr=subprocess.PIPE)
            return True
        else:
            if logger:
                logger.info(f"Directory {target_dir} already exists, skipping clone")
            return True
            
    except Exception as e:
        if logger:
            logger.error(f"Error cloning repo for {package_name}: {str(e)}")
        return False


def build_package(package_name, logger=None, ros_distro="jazzy", attempts=2):
    """
    Build a ROS package with improved error handling and automatic fixes.
    
    Args:
        package_name: Name of the package to build
        logger: ROS2 logger to display messages
        ros_distro: ROS distribution (jazzy, humble, etc.)
        attempts: Number of build attempts
    
    Returns:
        bool: True if build succeeded, False otherwise
    """
    try:
        # Remember current directory to return later
        original_dir = os.getcwd()
        
        try:
            # Change to workspace directory
            workspace_dir = os.path.expanduser("~/ros2_ws")
            os.chdir(workspace_dir)
            
            if logger:
                logger.info(f"Building package {package_name} in {workspace_dir}...")
            
            # Try to resolve dependencies
            try:
                install_rosdep_package(package_name, logger=logger)
            except Exception as e:
                if logger:
                    logger.warn(f"Failed to resolve dependencies with rosdep: {e}")
            
            # Path to the package
            package_path = os.path.join(workspace_dir, "src", package_name)
            
            for attempt in range(1, attempts + 1):
                try:
                    if logger:
                        logger.info(f"Build attempt {attempt}/{attempts} for {package_name}...")
                    
                    # Try to build
                    process = subprocess.Popen(
                        ['colcon', 'build', '--packages-select', package_name], 
                        stderr=subprocess.PIPE,
                        stdout=subprocess.PIPE
                    )
                    stdout, stderr = process.communicate()
                    stdout = stdout.decode('utf-8')
                    stderr = stderr.decode('utf-8')
                    
                    # If build succeeded
                    if process.returncode == 0:
                        if logger:
                            logger.info(f"Successfully built {package_name}")
                        return True
                    
                    # Analyze errors for potential automatic fixes
                    if "catkin" in stderr or "catkin" in stdout:
                        if logger:
                            logger.warn(f"Package {package_name} is designed for ROS1 (catkin). Attempting conversion...")
                        
                        if not convert_ros1_to_ros2(package_path, package_name, logger):
                            if logger:
                                logger.error(f"Failed to convert ROS1->ROS2 for {package_name}")
                            return False
                    
                    elif "Could not find a package configuration file provided by" in stderr:
                        # Extract missing package name
                        import re
                        match = re.search(r'Could not find a package configuration file provided by "([^"]+)"', stderr)
                        if match:
                            missing_pkg = match.group(1)
                            if logger:
                                logger.warn(f"Missing package: {missing_pkg}, attempting to install...")
                            
                            # Try to install missing package
                            if install_ros_apt_package(missing_pkg, ros_distro=ros_distro, logger=logger) or \
                               install_rosdep_package(missing_pkg, logger=logger):
                                continue  # Try building again
                    
                    elif "No such file or directory" in stderr:
                        # Extract missing directories or files from error messages
                        import re
                        
                        # Look for patterns like "cannot find "/path/to/something": No such file or directory"
                        missing_paths = re.findall(r'cannot find "([^"]+)": No such file or directory', stderr)
                        
                        # Also look for other common error patterns
                        if not missing_paths:
                            missing_paths = re.findall(r'No such file or directory:?\s+([^\s\n]+)', stderr)
                        
                        for missing_path in missing_paths:
                            # Check if this is a directory path
                            if os.path.dirname(missing_path) == os.path.join(workspace_dir, "src", package_name):
                                # It's a directory directly under the package
                                dir_name = os.path.basename(missing_path)
                                if logger:
                                    logger.warn(f"Creating missing directory from error: {dir_name}")
                                os.makedirs(missing_path, exist_ok=True)
                            elif missing_path.startswith(os.path.join(workspace_dir, "src", package_name)):
                                # It's a path within the package
                                if logger:
                                    logger.warn(f"Creating missing path: {missing_path}")
                                os.makedirs(os.path.dirname(missing_path), exist_ok=True)
                                
                                # If it seems like a directory that was missing
                                if not os.path.splitext(missing_path)[1]:  # No file extension
                                    os.makedirs(missing_path, exist_ok=True)
                        
                        # If no specific paths were found, check for standard directories
                        if not missing_paths:
                            # Check common directories that might be referenced in error messages
                            for dir_name in ["include", "meshes", "urdf", "launch", "config", "description"]:
                                if dir_name in stderr and not os.path.exists(os.path.join(package_path, dir_name)):
                                    if logger:
                                        logger.warn(f"Creating common missing directory: {dir_name}")
                                    os.makedirs(os.path.join(package_path, dir_name), exist_ok=True)
                    
                    elif "CMake Error at CMakeLists.txt" in stderr:
                        # If problem with CMakeLists.txt, try to simplify it
                        if logger:
                            logger.warn("Problem with CMakeLists.txt, attempting simplification...")
                        
                        # Backup original file
                        cmake_path = os.path.join(package_path, "CMakeLists.txt")
                        if os.path.exists(cmake_path):
                            import shutil
                            shutil.copy(cmake_path, f"{cmake_path}.bak")
                            
                            # If it's a description package, create minimal CMakeLists.txt
                            if "description" in package_name.lower():
                                create_minimal_description_cmakelists(cmake_path, package_name)
                    
                    else:
                        # If we've exhausted all attempts, show full error
                        if attempt == attempts:
                            if logger:
                                logger.error(f"Build failed after {attempts} attempts.")
                                logger.error(f"Error: {stderr}")
                            return False
                
                except Exception as e:
                    if logger:
                        logger.error(f"Exception during build: {str(e)}")
                    if attempt == attempts:
                        return False
            
            return False
            
        finally:
            # Return to original directory
            os.chdir(original_dir)
            
    except Exception as e:
        if logger:
            logger.error(f"Error building package {package_name}: {str(e)}")
        return False


def convert_ros1_to_ros2(package_path, package_name, logger=None):
    """Convert a ROS1 package to ROS2."""
    try:
        # Check if this is a description package
        is_description = "description" in package_name.lower()
        
        # Paths to important files
        cmake_path = os.path.join(package_path, "CMakeLists.txt")
        package_xml_path = os.path.join(package_path, "package.xml")
        
        # Backup original files
        if os.path.exists(cmake_path):
            import shutil
            shutil.copy(cmake_path, f"{cmake_path}.ros1")
        
        if os.path.exists(package_xml_path):
            import shutil
            shutil.copy(package_xml_path, f"{package_xml_path}.ros1")
        
        # If it's a description package, create simplified files
        if is_description:
            create_minimal_description_cmakelists(cmake_path, package_name)
            create_minimal_description_package_xml(package_xml_path, package_name)
            
            if logger:
                logger.info(f"Converted {package_name} to a ROS2 description package")
            return True
        
        # Otherwise, do a more elaborate conversion
        if os.path.exists(cmake_path):
            with open(cmake_path, 'r') as f:
                content = f.read()
            
            # Replace catkin elements with ament_cmake elements
            content = content.replace("find_package(catkin REQUIRED", "find_package(ament_cmake REQUIRED")
            content = content.replace("catkin_package(", "ament_package(")
            
            # Add ament_package() if it's not there
            if "ament_package()" not in content:
                content += "\nament_package()\n"
            
            with open(cmake_path, 'w') as f:
                f.write(content)
        
        if os.path.exists(package_xml_path):
            with open(package_xml_path, 'r') as f:
                content = f.read()
            
            # Update to format="3"
            import re
            content = re.sub(r'<package[^>]*>', '<package format="3">', content)
            
            # Replace dependencies
            content = content.replace("<buildtool_depend>catkin</buildtool_depend>", 
                                     "<buildtool_depend>ament_cmake</buildtool_depend>")
            
            # Add export if needed
            if "<export>" not in content:
                end_tag = "</package>"
                if end_tag in content:
                    content = content.replace(end_tag, "  <export>\n    <build_type>ament_cmake</build_type>\n  </export>\n</package>")
            elif "<build_type>" not in content:
                content = content.replace("</export>", "    <build_type>ament_cmake</build_type>\n  </export>")
            
            with open(package_xml_path, 'w') as f:
                f.write(content)
        
        if logger:
            logger.info(f"Converted {package_name} from ROS1 to ROS2")
        return True
        
    except Exception as e:
        if logger:
            logger.error(f"Error converting ROS1->ROS2: {str(e)}")
        return False


def create_minimal_description_cmakelists(cmake_path, package_name):
    """Create a minimal CMakeLists.txt for a description package."""
    minimal_cmake = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

find_package(ament_cmake REQUIRED)

# Install directories with meshes and URDF files
foreach(dir meshes urdf launch config)
  if(EXISTS "${{CMAKE_CURRENT_SOURCE_DIR}}/${{dir}}")
    install(DIRECTORY ${{dir}}
      DESTINATION share/${{PROJECT_NAME}}
    )
  endif()
endforeach()

ament_package()
"""
    with open(cmake_path, 'w') as f:
        f.write(minimal_cmake)
    return True


def create_minimal_description_package_xml(package_xml_path, package_name):
    """Create a minimal package.xml for a description package."""
    # If package.xml already exists, get some information from it
    description = f"ROS2 description package for {package_name}"
    version = "0.0.0"
    maintainer = "user@todo.todo"
    license_tag = "TODO"
    
    if os.path.exists(package_xml_path):
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(package_xml_path)
            root = tree.getroot()
            
            if root.find("description") is not None:
                description = root.find("description").text
            
            if root.find("version") is not None:
                version = root.find("version").text
                
            if root.find("maintainer") is not None:
                maintainer_elem = root.find("maintainer")
                maintainer = maintainer_elem.get("email", "user@todo.todo")
                
            if root.find("license") is not None:
                license_tag = root.find("license").text
        except Exception:
            pass
    
    minimal_package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>{version}</version>
  <description>{description}</description>
  <maintainer email="{maintainer}">{maintainer.split('@')[0]}</maintainer>
  <license>{license_tag}</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    with open(package_xml_path, 'w') as f:
        f.write(minimal_package_xml)
    return True


def clone_and_build_specific_repo(repo_url, package_name, logger=None):
    """Clone a specific repo and build the package."""
    # First clone the repository
    if not clone_specific_repo(repo_url, package_name, logger):
        if logger:
            logger.error(f"Failed to clone repository {repo_url}")
        return False
    
    # Then build the package
    if not build_package(package_name, logger):
        if logger:
            logger.error(f"Failed to build package {package_name}")
        return False
    
    return True


def install_ros_apt_package(package_name, ros_distro="jazzy", logger=None):
    """Try to install a ROS package using apt."""
    try:
        # Format for most ROS package names in apt
        apt_package = f"ros-{ros_distro}-{package_name.replace('_', '-')}"
        
        if logger:
            logger.info(f"Attempting to install {apt_package} via apt...")
            
        subprocess.check_call(['sudo', 'apt-get', 'update'], 
                             stderr=subprocess.PIPE)
        subprocess.check_call(['sudo', 'apt-get', 'install', '-y', apt_package], 
                             stderr=subprocess.PIPE)
                             
        if logger:
            logger.info(f"Successfully installed {apt_package} via apt")
        return True
    except subprocess.CalledProcessError as e:
        if logger:
            logger.error(f"Failed to install {apt_package} via apt: {e.stderr.decode('utf-8') if e.stderr else str(e)}")
        return False


def extract_robot_info(xacro_path):
    """
    Convertit un fichier Xacro en URDF, parse le modèle avec urdf_parser_py,
    et retourne les infos utiles sous forme de dictionnaire.
    """
    # 1. Conversion Xacro ➜ URDF XML

    # Get the current environment
    env = os.environ.copy()
    
    # Add the workspace lib paths
    workspace_path = os.path.expanduser("~/ros2_ws")
    install_path = os.path.join(workspace_path, "install")
    
    # Set AMENT_PREFIX_PATH to include your workspace
    if 'AMENT_PREFIX_PATH' in env:
        env['AMENT_PREFIX_PATH'] = f"{install_path}:{env['AMENT_PREFIX_PATH']}"
    else:
        env['AMENT_PREFIX_PATH'] = install_path
    
    # Set ROS_PACKAGE_PATH to include your workspace
    if 'ROS_PACKAGE_PATH' in env:
        env['ROS_PACKAGE_PATH'] = f"{install_path}/share:{env['ROS_PACKAGE_PATH']}"
    else:
        env['ROS_PACKAGE_PATH'] = f"{install_path}/share"
    
    # Execute xacro with the modified environment
    urdf_xml = subprocess.check_output(['xacro', xacro_path], env=env).decode("utf-8")


    # 2. Parsing URDF (depuis string)
    robot = URDF.from_xml_string(urdf_xml)

    # 3. Extraction des informations
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
    data['robot_name'] = robot.name
    data['joints'] = joints
    data['links'] = links
    data['joint_names'] = [j['name'] for j in joints]
    data['link_names'] = [l['name'] for l in links]
    data['ee_link_candidates'] = terminal_links
    data['root_link'] = robot.get_root()

    return data


class RobotInfoExtractor(Node):
    def __init__(self):
        super().__init__('extractor')
        
        # Track attempted package installations
        self.attempted_packages = set()
        
        urdf_dir = '/home/lab/ros2_ws/src/arms_sim/urdf/'
        # Cherche tous les fichiers xacro et urdf dans le dossier
        files = glob(os.path.join(urdf_dir, '*.xacro')) + glob(os.path.join(urdf_dir, '*.urdf'))

        for file_path in files:
            self.get_logger().info(f"Extraction pour le fichier: {file_path}")
            try:
                info = self.extract_robot_info_with_auto_install(file_path)
                self.get_logger().info(f"Successfully extracted info from {file_path}")
                self.get_logger().info(str(info))
            except Exception as e:
                self.get_logger().error(f"Erreur avec le fichier {file_path}: {str(e)}")
    
    def extract_robot_info_with_auto_install(self, xacro_path, auto_install=True):
        """Extract robot info with automatic package installation."""
        try:
            # Try normal extraction first
            urdf_xml = subprocess.check_output(['xacro', xacro_path], stderr=subprocess.PIPE).decode("utf-8")
            if urdf_xml:
                return extract_robot_info(xacro_path)
        except subprocess.CalledProcessError as e:
            error_output = e.stderr.decode('utf-8') if e.stderr else str(e)
            
            if auto_install and "package" in error_output and "not found" in error_output:
                self.get_logger().warning(f"Package error: {error_output}")
                
                # Extract package name from error message
                match = re.search(r"package '([^']+)' not found", error_output)
                
                if match:
                    package_name = match.group(1)
                    
                    # Skip if we already tried to install this package
                    if package_name in self.attempted_packages:
                        self.get_logger().warn(f"Already attempted to install {package_name}, skipping")
                        raise
                    
                    self.attempted_packages.add(package_name)
                    self.get_logger().info(f"Attempting to install missing package: {package_name}")
                    
                    # If apt and rosdep fail, try cloning from GitHub
                    if clone_and_build_package(package_name, logger=self.get_logger()):
                        self.get_logger().info(f"Successfully built {package_name} from source")
                        # Try again with the package installed
                        return extract_robot_info(xacro_path)
                    
                    # Try installing with apt first
                    if install_ros_apt_package(package_name, logger=self.get_logger()):
                        self.get_logger().info(f"Successfully installed {package_name} from ros apt")
                        # Try again with the package installed
                        return extract_robot_info(xacro_path)
                    
                    # Then try with rosdep
                    if install_rosdep_package(package_name, logger=self.get_logger()):
                        self.get_logger().info(f"Successfully installed {package_name} from rosdep")
                        # Try again with the package installed
                        return extract_robot_info(xacro_path)
                    
                    self.get_logger().error(f"Failed to install {package_name} using any method")
            
            # If auto-install failed or was not attempted, re-raise with better logging
            self.get_logger().error(f"Failed to process {xacro_path}: {error_output}")
            raise


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