# ROS 2 Robot Simulation and Control Project

This project provides a complete environment for working with a Kinova Gen3 Lite robot in ROS 2 Jazzy, with both simulation capabilities and real hardware control.

## 📚 Documentation Overview

* [Project Requirements](INSTALL.md) - Detailed setup instructions and dependencies
* [ROS 2 Basics Tutorial](notes/ros2_basics.ipynb) - Learn the fundamentals of ROS 2
* [Kinova Gen3 Lite Simulation Documentation](src/kinova_gen3_lite_sim/README.md)

## 🚀 Getting Started

### Prerequisites

Ensure you have ROS 2 Jazzy installed on Ubuntu 24.04. The project uses the following main components:

- ROS 2 Jazzy
- MoveIt 2
- Ignition Gazebo
- Kinova Gen3 Lite libraries
- Robotiq Gripper support

Sure! Here's a rewritten version of your installation section, clearly presenting the two options:



### 🚀 Installation Options

You can use this tool in **two different ways**:



#### 🔹 Option 1: Using Docker (Recommended)

Run the tool in a pre-configured container with NVIDIA support.

```bash
# Set up the Docker environment
./install_docker_nvidia.sh
```

This script will prepare everything needed to run the containerized version of the tool.



#### 🔹 Option 2: Native Installation on Ubuntu 24.04

If you prefer to run the tool directly on your system:

```bash
# Clone the repository and install dependencies
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
vcs import src < requirements_ros2.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Refer to the [INSTALL.md](INSTALL.md) file for full native setup instructions.


## 📖 Documentation

### [Project Requirements](INSTALL.md)
Comprehensive guide to installing all necessary components:
- Core ROS 2 middleware tools
- MoveIt and robot drivers
- Simulation environment
- Building and configuration instructions

### [ROS 2 Basics Tutorial](notes/ros2_basics.ipynb)
Interactive tutorial covering:
- ROS 2 architecture and concepts
- Nodes, topics, services, and actions
- Building packages and working with parameters
- TF2 and visualization

### [Kinova Gen3 Lite Simulation](src/kinova_gen3_lite_sim/README.md)

#### I. 📘 Simulated Data Generation for Vision-Based Learning
Instructions for generating synthetic training data using the simulated robot.

#### II. 🛠️ Debugging Camera Plugin Errors in Gazebo
Solutions for common camera and sensor configuration problems.

#### III. 🧾 Report: Resolving Camera Plugin & Bridge Issues
Detailed report on integrating ROS 2 with Ignition Gazebo sensors.

#### IV. Progress Report: Simulation & Image Data Capture
Latest updates on simulation development and data collection methods.

## 🤖 Usage Examples

```bash
# Build the container docker
docker build -t ros2-ws .

# Allow Docker to access your display (just once)
xhost +local:root && xhost +local:docker

# Start the container
docker compose up -d

# Stop the container
docker compose down

# Check started containers
docker ps

# New terminal in the container
docker exec -it ros2_dev bash -c "source /entrypoint.sh && exec bash"

# Launch MoveIt with a robot for test
ros2 launch kinova_gen3_lite_moveit_config demo.launch.py

# Launch MoveIt with gen3 lite arm for manipulation
colcon build --packages-select moveit2_tutorials kinova_gen3_lite_moveit_config && ros2 launch moveit2_tutorials gen3lite_demo.launch.py

# Run a basic motion example
colcon build --packages-select hello_moveit && ros2 run hello_moveit gen3_lite_hello_moveit

# Launch the simulated gen3 lite robot in Gazebo
colcon build --packages-select kinova_gen3_lite_sim && ros2 launch kinova_gen3_lite_sim sim_launch.py

colcon build --packages-select arms_sim && ros2 launch arms_sim sim_launch.py

colcon build --packages-select arms_sim && ros2 run arms_sim robot_info_extractor
```

## 🧰 Tools & Utilities

The project includes various utilities for:
- Joint-level robot control
- Cartesian path planning
- Image capture and processing
- Simulation-hardware bridging

## 🔧 Troubleshooting

For common issues with the simulation, camera plugins, or motion planning, refer to the debugging section in [Kinova Gen3 Lite Simulation documentation](src/kinova_gen3_lite_sim/README.md).

## 📝 License

This project is licensed under the Smart Systems Lab - UF License - see the LICENSE file for details.

## 🙏 Acknowledgements

- Kinova Robotics for the robot models and drivers
- The MoveIt and ROS 2 communities for their excellent frameworks
- Gazebo/Ignition community for the simulation environment