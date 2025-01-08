import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import os
import supervision as sv
from ultralytics import YOLO
import math
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge


class IntelSubscriber(Node):
    def __init__(self):
        super().__init__("intel_subscriber")
        self.subscription_rgb = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.rgb_frame_callback, 10)
        self.br_rgb = CvBridge()


    def rgb_frame_callback(self, data):
        self.get_logger().warning("Receiving RGB frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv2.imshow("RGB", current_frame)
        cv2.waitKey(1)




def main(args = None):
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber)
    intel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main":
    main()