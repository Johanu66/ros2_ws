#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time

class AutoMotionExplorer(Node):
    def __init__(self):
        super().__init__('auto_motion_explorer')

        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/gen3_lite_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.index = 0
        self.max_index = 100  # Number of poses to explore

        self.joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]

        self.get_logger().info("AutoMotionExplorer initialized. Starting motion.")

    def timer_callback(self):
        if self.index >= self.max_index:
            self.get_logger().info("Motion exploration complete.")
            rclpy.shutdown()
            return

        self.move_to_random_pose()
        self.index += 1
        time.sleep(2)  # Wait for robot to stabilize

    def move_to_random_pose(self):
        # You can replace this logic with a grid sampler or other algorithm if desired
        point = JointTrajectoryPoint()
        point.positions = np.random.uniform(low=-1.5, high=1.5, size=6).tolist()
        point.time_from_start.sec = 2

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points = [point]
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()

        self.joint_pub.publish(traj)
        self.get_logger().info(f"Moved to pose #{self.index}: {point.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoMotionExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
