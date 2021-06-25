#!/usr/bin/env python3

# Copyright (c) 2021 Alfi Maulana
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import cv2
from cv_bridge.core import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class PoseDetector(Node):

    def __init__(self):
        super().__init__('pose_detector')
        self.image_subscription = self.create_subscription(
          Image, '/image_raw', self.image_callback, 10)
        self.image_subscription

        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        mat = self.cv_bridge.imgmsg_to_cv2(msg)
        mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)

        cv2.imshow('Pose Detector', mat)
        if cv2.waitKey(1) & 0xFF == 27:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    pose_detector = PoseDetector()
    rclpy.spin(pose_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
