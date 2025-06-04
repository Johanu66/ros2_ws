# Project Requirements

## 🧠 Core Middleware & Tools
- `rclcpp`, `rclpy`, `rcl`, `rosidl_runtime_*`, `ros2cli`
- `ros2_control`, `controller_manager`, `hardware_interface`
- `joint_state_broadcaster`, `joint_trajectory_controller`, `velocity_controllers`, `effort_controllers`

## 🤖 MoveIt + Kortex Robot
- `moveit`, `moveit_ros`, `moveit_setup_assistant`
- `kinova_gen3_lite_moveit_config`, `kortex_driver`, `kortex_description`
- `robotiq_description`, `robotiq_driver`

## 🧩 Simulation (Ignition Gazebo)
- `gz_ros2_control`, `gz_ros2_control_demos`, `ros_gz_sim`, `gz_sim_vendor`
- `gz_plugin_vendor`, `gz_math_vendor`, `gz_rendering_vendor`

## 🧪 Other utilities
- `teleop_twist_keyboard`, `joy`, `image_transport`, `tf2_ros`, `diagnostic_updater`

## 🧰 Build Tools
- `ament_cmake`, `ament_lint`, `ament_index_cpp`, `pluginlib`, `class_loader`

---

## 📦 Install via rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```


# ROS 2 Project environnement Setup Guide

## 🧠 Core Middleware & Tools

Standard ROS 2 packages can be installed using:
```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-rcl ros-jazzy-rosidl-runtime-cpp ros-jazzy-ros2cli
sudo apt install ros-jazzy-ros2-control ros-jazzy-controller-manager ros-jazzy-hardware-interface
sudo apt install ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller ros-jazzy-velocity-controllers ros-jazzy-effort-controllers
```

## 🤖 MoveIt + Kortex Robot

### Installing MoveIt 2
```bash
# Clone MoveIt 2 repositories
git clone https://github.com/moveit/moveit2 -b main src/moveit2
git clone https://github.com/moveit/moveit_msgs -b ros2 src/moveit_msgs
git clone https://github.com/moveit/moveit_resources -b ros2 src/moveit_resources
git clone https://github.com/moveit/moveit_visual_tools -b ros2 src/moveit_visual_tools
git clone https://github.com/moveit/moveit_task_constructor.git -b ros2 src/moveit_task_constructor
# git clone https://github.com/moveit/moveit2_tutorials -b main src/moveit2_tutorials

# Install dependencies
sudo apt install ros-jazzy-ompl libomp-dev libbackward-cpp-dev
```

### Kinova Gen3 Lite and Robotiq Gripper
```bash
# Clone repositories
# git clone https://github.com/Kinovarobotics/ros2_kortex.git -b main src/ros2_kortex
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git -b main src/ros2_robotiq_gripper

# Clone supporting repositories
git clone https://github.com/PickNikRobotics/rviz_visual_tools.git -b ros2 src/rviz_visual_tools
git clone https://github.com/tylerjw/serial.git -b ros2 src/serial
git clone https://github.com/sea-bass/picknik_controllers.git -b fix-deprecated-realtime-tools-imports src/picknik_controllers
```

## 🧩 Simulation (Ignition Gazebo)

```bash
# Clone repositories
git clone https://github.com/gazebosim/ros_gz.git -b ros2 src/ros_gz
git clone https://github.com/ros-controls/gz_ros2_control.git -b jazzy src/gz_ros2_control
git clone https://github.com/gazebosim/gz-plugin -b gz-plugin3 src/gz-plugin
git clone https://github.com/gazebo-release/gz_sensors_vendor.git -b rolling src/gz_sensors_vendor
git clone https://github.com/ros-perception/vision_opencv.git -b rolling src/vision_opencv

# Install dependencies
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-gz-sim-vendor
sudo apt install ros-jazzy-gz-plugin-vendor ros-jazzy-gz-math-vendor ros-jazzy-gz-rendering-vendor
```

## 🧪 Other utilities

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard ros-jazzy-joy ros-jazzy-image-transport ros-jazzy-tf2-ros ros-jazzy-diagnostic-updater
```

## 🧰 Build Tools

```bash
sudo apt install ros-jazzy-ament-cmake ros-jazzy-ament-lint ros-jazzy-ament-index-cpp ros-jazzy-pluginlib ros-jazzy-class-loader
sudo apt install ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common ros-jazzy-ament-cmake-xmllint
sudo apt install ros-jazzy-rosidl-default-generators ros-jazzy-launch-param-builder ros-jazzy-ros2cli-common-extensions
```

## 📦 Complete easily Installation

1. Clone the repositories:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
vcs import src < requirements_ros2.repos
```

2. Install dependencies:
```bash
# Update rosdep database
sudo rosdep update

# Install dependencies (skipping packages with missing definitions)
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO --skip-keys="backward_ros ament_lint_common example_interfaces ament_cmake_xmllint ament_lint_auto launch_param_builder joy ros2cli_common_extensions rsl rosidl_default_generators urdfdom_headers ompl turtlesim"

# Install additional dependencies
sudo apt install ros-jazzy-turtlesim ros-jazzy-example-interfaces liburdfdom-headers-dev libbackward-cpp-dev
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Exporting Environment Setup

To export your project setup for reproducibility:

```bash
# Export repository information
vcs export src > requirements_ros2.repos

# Export Python dependencies
pip freeze > requirements.txt

# Export ROS 2 package dependencies
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO --simulate > ros2_dependencies.txt
```

This setup includes all required components for working with the Kinova Gen3 Lite robot with a Robotiq gripper in both real hardware and simulated environments.