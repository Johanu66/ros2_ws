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

### Installation

```bash
# Clone the repository and install dependencies
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
vcs import src < requirements_ros2.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

See [detailed installation instructions](INSTALL.md) for complete setup.

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
# Launch MoveIt with the robot
ros2 launch kinova_gen3_lite_moveit_config demo.launch.py

# Run a basic motion example
ros2 run hello_moveit gen3_lite_hello_moveit

# Launch the simulated robot in Gazebo
ros2 launch kinova_gen3_lite_sim robot_simulation.launch.py
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