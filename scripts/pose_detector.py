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

import argparse
import cv2
from cv_bridge.core import CvBridge
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vectors import Point, Vector


def landmark_to_point(landmark):
    return Point(landmark.x, landmark.y, landmark.z)


class PoseDetector(Node):

    def __init__(self, **kwargs):
        super().__init__(kwargs.get('node_name', 'pose_detector'))

        self.image_subscription = self.create_subscription(
          Image, kwargs.get('image_topic', '/image_raw'), self.image_callback, 10)
        self.image_subscription

        self.cv_bridge = CvBridge()
        self.pose = mp.solutions.pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

    def image_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        results = self.pose.process(image)

        if results.pose_landmarks:
            l_knee_landmark = results.pose_world_landmarks.landmark[25]
            r_knee_landmark = results.pose_world_landmarks.landmark[26]

            l_hip = landmark_to_point(results.pose_world_landmarks.landmark[23])
            r_hip = landmark_to_point(results.pose_world_landmarks.landmark[24])
            l_knee = landmark_to_point(l_knee_landmark)
            r_knee = landmark_to_point(r_knee_landmark)
            l_ankle = landmark_to_point(results.pose_world_landmarks.landmark[27])
            r_ankle = landmark_to_point(results.pose_world_landmarks.landmark[28])

            l_upper = Vector.from_points(l_knee, l_hip)
            l_bottom = Vector.from_points(l_knee, l_ankle)
            r_upper = Vector.from_points(r_knee, r_hip)
            r_bottom = Vector.from_points(r_knee, r_ankle)

            l_angle = l_bottom.angle(l_upper) if l_knee_landmark.visibility > 0.5 else -1
            r_angle = r_bottom.angle(r_upper) if r_knee_landmark.visibility > 0.5 else -1

            print("\033[2J\033[2H")
            print("angle", l_angle, r_angle)

        mp.solutions.drawing_utils.draw_landmarks(
            image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)

        cv2.imshow('Pose Detector', image)
        if cv2.waitKey(5) & 0xFF == 27:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(prog='pose_detector')

    parser.add_argument(
        '--node-name',
        nargs='?',
        default='pose_detector',
        help='node name to be used')

    parser.add_argument(
        '--image-topic',
        nargs='?',
        default='/image_raw',
        help='name of the image topic')

    options = parser.parse_args(args)

    pose_detector = PoseDetector(
        node_name=options.node_name,
        image_topic=options.image_topic)

    rclpy.spin(pose_detector)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
