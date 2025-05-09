import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgbd/image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_counter = 0
        self.save_dir = os.path.expanduser('~/image_saver_output')
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"Saving images to: {self.save_dir}")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.save_dir, f'image_{timestamp}.png')
            cv2.imwrite(filename, cv_image)
            self.image_counter += 1
            self.get_logger().info(f"Saved image {self.image_counter} to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert and save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
