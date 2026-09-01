#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os


SAVE_DIR = "/home/ros/yolo_dataset/images/train"


class ImageCapture(Node):

    def __init__(self):

        super().__init__("image_capture")

        self.bridge = CvBridge()

        self.count = 0

        os.makedirs(SAVE_DIR, exist_ok=True)


        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.callback,
            10
        )


    def callback(self,msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            "bgr8"
        )


        if self.count % 15 == 0:

            filename = (
                SAVE_DIR +
                f"/frame_{self.count:04d}.jpg"
            )

            cv2.imwrite(
                filename,
                frame
            )

            self.get_logger().info(
                f"save {filename}"
            )


        self.count += 1



def main():

    rclpy.init()

    node = ImageCapture()

    rclpy.spin(node)


if __name__=="__main__":

    main()

