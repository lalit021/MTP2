#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

class CameraPublisher:
    def __init__(self, use_realsense=True):
        rospy.init_node('camera_image_publisher', anonymous=True)

        self.publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.heartbeat_pub = rospy.Publisher('/camera/heartbeat', Bool, queue_size=10)
        self.bridge = CvBridge()
        self.use_realsense = use_realsense and REALSENSE_AVAILABLE
        self.pipeline = None
        self.config = None
        self.last_heartbeat = time.time()
        self.last_publish_success = False  # Track last state to reduce logging

        if self.use_realsense:
            self.initialize_realsense()
        else:
            rospy.logwarn("Using regular webcam (not RealSense).")
            self.cap = cv2.VideoCapture(0)  # Default to laptop/webcam

    def initialize_realsense(self):
        """Attempts to initialize the RealSense pipeline."""
        try:
            rospy.loginfo("Initializing RealSense camera...")
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(self.config)
            rospy.loginfo("RealSense camera initialized successfully.")
        except Exception as e:
            rospy.logerr("Failed to initialize RealSense camera: {}".format(e))
            self.pipeline = None

    def publish_image(self):
        image_success = False  # Track current image capture success

        if self.use_realsense:
            if not self.pipeline:
                rospy.logwarn("RealSense pipeline is not active. Attempting to reconnect...")
                self.initialize_realsense()
                return
            
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    rospy.logwarn("No color frame received. Camera might be disconnected.")
                    return
                
                color_image = np.asanyarray(color_frame.get_data())
                image_success = True
            except Exception as e:
                rospy.logwarn("Error capturing RealSense image: {}. Trying to reconnect...".format(e))
                self.initialize_realsense()
                return
        else:
            # Regular webcam capture
            ret, color_image = self.cap.read()
            if not ret:
                rospy.logwarn("Could not access webcam.")
                return
            image_success = True

        if image_success and not self.last_publish_success:
            rospy.loginfo("Image stream resumed.")

        self.last_publish_success = image_success

        ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.publisher.publish(ros_image)

        # Publish heartbeat every 5 seconds
        if time.time() - self.last_heartbeat > 5:
            self.heartbeat_pub.publish(True)
            self.last_heartbeat = time.time()

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_image()
            rate.sleep()

    def cleanup(self):
        if self.pipeline:
            self.pipeline.stop()
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("CameraPublisher shutdown complete.")

if __name__ == '__main__':
    rospy.init_node('camera_image_publisher', anonymous=True)

    # Get ROS parameter (instead of using argparse)
    use_realsense = rospy.get_param("~realsense", True)

    publisher = None
    try:
        publisher = CameraPublisher(use_realsense=use_realsense)
        publisher.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down camera publisher node.")
    finally:
        if publisher:
            publisher.cleanup()
