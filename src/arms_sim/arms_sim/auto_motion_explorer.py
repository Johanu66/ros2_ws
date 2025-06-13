#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from arms_sim.robot_info_extractor import extract_robot_info_with_auto_install
from launch.logging import get_logger
import subprocess
import json
import re
from controller_manager_msgs.srv import ListControllers
from time import sleep

class AutoMotionExplorer(Node):
    def __init__(self):
        super().__init__('auto_motion_explorer')

        self.urdf_path = self.declare_parameter('urdf_path', '').value
        self.max_index = self.declare_parameter('max_poses', 100).value  # Number of poses to explore

        self.logger = get_logger("arms_sim")
        self.logger.info(f"Loading robot info from {self.urdf_path}")
        self.robot_info = extract_robot_info_with_auto_install(xacro_path=self.urdf_path, logger=self.logger)

        self.logger.info(self.robot_info)
        
        # # Wait for controllers to be available
        # self.logger.info("Waiting for controller_manager to be available...")
        # sleep(2.0)  # Give controller manager time to start
        
        # Get all active controllers and their joints
        self.controllers = self.discover_controllers()

        self.logger.info(self.controllers)
        
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.index = 0
        
        self.logger.info("AutoMotionExplorer initialized. Starting motion.")

    def discover_controllers(self):
        """Discover active controllers and their controlled joints"""
        controllers = {}
        
        # Create a service client
        client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Waiting for controller manager service...')
        
        # Call the service
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if not response:
            self.logger.error("Failed to get controller list")
            return controllers
            
        self.logger.info(response.controller)
        
        # Process each controller
        for controller in response.controller:
            if controller.state == 'active':
                self.logger.info(f"Found active controller: {controller.name}")
                
                # Get controller joints
                joints = []
                claimed_interfaces = controller.claimed_interfaces
                for interface in claimed_interfaces:
                    # Extract the joint name (remove "/position" or "/velocity" suffix)
                    joint_name = interface.split('/')[0] 
                    if joint_name not in joints:
                        joints.append(joint_name)
                
                if joints:
                    controllers[controller.name] = {
                        'joints': joints,
                        'joint_limits': self.get_joint_limits(joints)
                    }
                    self.logger.info(f"  - Controls joints: {joints}")
        
        return controllers

    def get_joint_limits(self, joint_names):
        """Get joint limits from robot_info for the specified joints"""
        limits = []
        
        for joint_name in joint_names:
            # Find this joint in robot_info
            joint_info = next((j for j in self.robot_info["joints"] if j["name"] == joint_name), None)
            
            if joint_info and joint_info["type"] in ["revolute", "prismatic"]:
                limits.append([joint_info["limit"]["lower"], joint_info["limit"]["upper"]])
            else:
                # Default limits if joint not found or is not movable
                self.logger.warning(f"Using default limits for joint {joint_name}")
                limits.append([-2.0, 2.0])
                
        return limits

    def timer_callback(self):
        if self.index >= self.max_index:
            self.logger.info("Motion exploration complete.")
            rclpy.shutdown()
            return
            
        # Explore each controller in sequence
        for controller_name, controller_info in self.controllers.items():
            self.explore_controller(controller_name, controller_info)
        
        self.index += 1

    def explore_controller(self, controller_name, controller_info):
        """Move joints for a specific controller"""
        joint_names = controller_info['joints']
        joint_limits = controller_info['joint_limits']
        
        if not joint_names or not joint_limits:
            return
            
        # Generate random positions within joint limits
        positions = []
        for low, high in joint_limits:
            positions.append(np.random.uniform(low=low, high=high))
        
        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(joint_names)  # Explicit zero velocities
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        
        traj.points = [point]
        
        # Create a publisher for this controller
        topic = f'/{controller_name}/joint_trajectory'
        publisher = self.create_publisher(JointTrajectory, topic, 10)
        
        self.logger.info(f"Publishing to {controller_name}, pose #{self.index}: {[round(p, 2) for p in point.positions]}")
        publisher.publish(traj)
        self.logger.info(f"Published to {controller_name}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoMotionExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()