import os
import subprocess
import yaml # type: ignore

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
        print(f"Added {len(added_paths)} paths to GZ_SIM_RESOURCE_PATH:")
        for path in added_paths:
            print(f"  - {path}")
        print(f"New GZ_SIM_RESOURCE_PATH: {new_gz_path}")
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