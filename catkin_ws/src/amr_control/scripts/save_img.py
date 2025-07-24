#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import numpy as np  # ✅ Required for saving .npy depth data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from collections import deque

class ImageSaver:
    def __init__(self, topic_name, save_dir, encoding='bgr8'):
        self.topic_name = topic_name
        self.save_dir = save_dir
        self.encoding = encoding
        self.bridge = CvBridge()
        self.image_queue = deque(maxlen=10)

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        self.sub = rospy.Subscriber(topic_name, Image, self.callback)
        rospy.loginfo("Subscribed to {}".format(topic_name))

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
            self.image_queue.append(cv_image)
            self.save_images()
        except Exception as e:
            rospy.logerr("Error converting image from {}: {}".format(self.topic_name, e))

    def save_images(self):
        for idx, img in enumerate(self.image_queue):
            img_filename = os.path.join(self.save_dir, "img_{:02d}".format(idx))

            if len(img.shape) == 2:  # Likely a depth image (grayscale)
                # Save raw depth as .npy
                np.save(img_filename + ".npy", img)

                # Normalize for viewing and save as PNG
                img_normalized = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
                img_uint8 = img_normalized.astype('uint8')
                cv2.imwrite(img_filename + ".png", img_uint8)

            else:
                # Save color image as-is
                cv2.imwrite(img_filename + ".png", img)

if __name__ == '__main__':
    rospy.init_node('amr_image_collector', anonymous=True)

    # Define image topics and encodings
    topics = {
        "back_color": {
            "topic": "/amr/camera_back/color/image_raw",
            "encoding": "bgr8"
        },
        "front_depth": {
            "topic": "/amr/camera_front/depth/image_rect_raw",
            "encoding": "passthrough"
        },
        "front_color": {
            "topic": "/amr/camera_front/color/image_raw",
            "encoding": "bgr8"
        },
        "back_depth": {
            "topic": "/amr/camera_back/depth/image_rect_raw",
            "encoding": "passthrough"
        }
    }

    savers = []
    for name, info in topics.items():
        save_path = os.path.join("images", name)
        saver = ImageSaver(info["topic"], save_path, info["encoding"])
        savers.append(saver)

    rospy.spin()
