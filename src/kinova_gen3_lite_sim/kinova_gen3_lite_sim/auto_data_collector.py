#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2 # type: ignore
from cv_bridge import CvBridge
import os
import numpy as np
import time

class AutoDataCollector(Node):
    def __init__(self):
        super().__init__('auto_data_collector')
        self.declare_parameter("output_dir", "/tmp/sim_data_auto")
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/rgbd/image', self.image_callback, 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.timer = self.create_timer(5.0, self.timer_callback)  # Trigger every 5 sec
        self.image_ready = False
        self.current_image = None
        self.index = 0

        self.joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]

        self.get_logger().info("AutoDataCollector node initialized.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            self.image_ready = True  # Mark image as ready for saving
            self.get_logger().info(f"Received image of shape: {cv_image.shape}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def timer_callback(self):
        if self.index >= 100:
            self.get_logger().info("Data collection complete.")
            rclpy.shutdown()
            return

        # Make sure that we are saving only one image at a time
        if self.image_ready:
            filename = os.path.join(self.output_dir, f"image_{self.index:05d}.png")
            cv2.imwrite(filename, self.current_image)
            self.get_logger().info(f"Saved image {filename}")
            self.index += 1
            self.image_ready = False
        else:
            self.get_logger().warn("Image not ready — skipping this round.")

        self.move_to_random_pose()
        time.sleep(2)  # wait for robot to stabilize

    def move_to_random_pose(self):
        point = JointTrajectoryPoint()
        point.positions = np.random.uniform(low=-1.5, high=1.5, size=6).tolist()
        point.time_from_start.sec = 2

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points = [point]
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()

        self.joint_pub.publish(traj)
        self.get_logger().info(f"Published random joint positions: {point.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoDataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
