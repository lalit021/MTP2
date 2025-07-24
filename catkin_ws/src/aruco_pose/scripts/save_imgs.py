#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self):
        # ROS setup
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        self.bridge = CvBridge()

        # Save image every 5 seconds (0.2 Hz)
        self.save_rate = 5
        self.image_save_timer = rospy.Timer(rospy.Duration(1.0 / self.save_rate), self.save_image_callback)

        # Save directory
        self.save_folder = "/root/catkin_ws/src/aruco_pose/saved_images"
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

        # Filename to overwrite
        self.image_filename = os.path.join(self.save_folder, "latest_image.png")

        self.last_image = None

    def image_callback(self, msg):
        """Receive images from the camera."""
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: {}".format(e))

    def save_image_callback(self, event):
        """Save the latest image every 5 seconds, overwriting previous."""
        if self.last_image is not None:
            try:
                cv2.imwrite(self.image_filename, self.last_image)
                rospy.loginfo("Saved image to: {}".format(self.image_filename))
            except Exception as e:
                rospy.logerr("Failed to save image: {}".format(e))
        else:
            rospy.logwarn("No image received yet; skipping save.")

if __name__ == "__main__":
    rospy.init_node('image_saver', anonymous=True)
    saver = ImageSaver()
    rospy.spin()
