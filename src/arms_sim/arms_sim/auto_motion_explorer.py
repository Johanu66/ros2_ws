#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from arms_sim.robot_info_extractor import extract_robot_info_with_auto_install
from launch.logging import get_logger

class AutoMotionExplorer(Node):
    def __init__(self):
        super().__init__('auto_motion_explorer')

        self.urdf_path = self.declare_parameter('urdf_path', '').value

        logger = get_logger("arms_sim")
        robot_info = extract_robot_info_with_auto_install(xacro_path=self.urdf_path, logger=logger)

        logger.warning(robot_info)

        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/gen3_lite_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.index = 0
        self.max_index = 100  # Number of poses to explore

        self.joint_names = robot_info["joint_names"]

        # Define joint limits for safe operation
        self.joint_limits = [
            [-2.5, 2.5],   # joint_1
            [-2.0, 2.0],   # joint_2
            [-2.5, 2.5],   # joint_3
            [-2.5, 2.5],   # joint_4
            [-2.5, 2.5],   # joint_5
            [-2.5, 2.5]    # joint_6
        ]

        self.get_logger().info("AutoMotionExplorer initialized. Starting motion.")

    def timer_callback(self):
        if self.index >= self.max_index:
            self.get_logger().info("Motion exploration complete.")
            rclpy.shutdown()
            return

        self.move_to_random_pose()
        self.index += 1

    def move_to_random_pose(self):
        # Generate random positions within joint limits
        positions = []
        for i, (low, high) in enumerate(self.joint_limits):
            positions.append(np.random.uniform(low=low, high=high))
        
        # Create a message exactly like the working example
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Explicit zero velocities
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        
        traj.points = [point]
        
        # Notice: we're not setting a header timestamp at all, just like the working example
        
        self.get_logger().info(f"Publishing to pose #{self.index}: {[round(p, 2) for p in point.positions]}")
        self.joint_pub.publish(traj)
        self.get_logger().info(f"Published to pose #{self.index}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoMotionExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()