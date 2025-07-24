#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self, max_images=500):
        # ROS Setup
        self.image_sub = rospy.Subscriber("/amr/camera_front/color/image_raw", Image, self.image_callback, queue_size=10)
        self.bridge = CvBridge()

        # Timer setup for saving images at a specific rate
        self.save_rate = 1  # Target frequency (Hz) for saving images
        self.image_save_timer = rospy.Timer(rospy.Duration(1.0 / self.save_rate), self.save_image_callback)

        # Create directory to save images
        self.save_folder = "/root/catkin_ws/src/aruco_pose/saved_images"
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

        self.max_images = max_images
        self.image_counter = 0
        self.last_image = None
        self.saved_images = []  # To keep track of the saved images' file paths

    def image_callback(self, msg):
        """Callback to receive images from the camera."""
        try:
            # Convert ROS Image to OpenCV
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Error in image callback: {}".format(e))

    def save_image_callback(self, event):
        """Callback for saving images at a fixed rate."""
        if self.last_image is not None:
            # Create a unique filename for the image
            timestamp = rospy.get_time()
            filename = os.path.join(self.save_folder, "image_{:.2f}.png".format(timestamp))

            # Save the image
            cv2.imwrite(filename, self.last_image)
            rospy.loginfo("Saved image to: {}".format(filename))

            # Add the filename to the saved images list
            self.saved_images.append(filename)

            # Increment the image counter
            self.image_counter += 1

            # If the number of saved images exceeds the limit, delete the oldest one
            if len(self.saved_images) > self.max_images:
                oldest_image = self.saved_images.pop(0)  # Get the oldest image file path
                os.remove(oldest_image)  # Delete the oldest image
                rospy.loginfo("Deleted old image: {}".format(oldest_image))

        else:
            rospy.logwarn("No image received yet, skipping save.")

if __name__ == "__main__":
    rospy.init_node('image_saver', anonymous=True)

    # Set the maximum number of images to save
    max_images = rospy.get_param("~max_images", 500)  # Default is 500 if not specified in launch file

    # Start processing
    image_saver = ImageSaver(max_images=max_images)

    rospy.spin()
