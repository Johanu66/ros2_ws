#!/bin/bash
echo "[*] Exporting VCS repos to requirements_ros2.repos..."
vcs export src > requirements_ros2.repos

echo "[*] Exporting Python dependencies to requirements.txt..."
pip freeze > requirements.txt

# Use $ROS_DISTRO environment variable instead of ros2 distro command
echo "[*] Simulating rosdep installation to ros2_dependencies.txt..."
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO --simulate > ros2_dependencies.txt

echo "[✓] Done. You can now commit these files for reproducible setup."