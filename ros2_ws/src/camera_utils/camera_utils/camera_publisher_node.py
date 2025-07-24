#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import sys
import numpy as np
import time

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # Create publisher for images
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        # Heartbeat setup: publish a heartbeat message every 5 seconds
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat_callback)
        
        # Check if 'realsense=True' argument is passed
        self.realsense = 'realsense=True' in sys.argv

        self.get_logger().info("Initializing camera publisher node...")

        # Camera initialization (RealSense or laptop camera)
        if self.realsense:
            try:
                self.get_logger().info("Using RealSense camera")
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self.pipeline.start(self.config)
                self.get_logger().info("RealSense camera initialized successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize RealSense camera: {e}")
                rclpy.shutdown()
                return
        else:
            try:
                self.get_logger().info("Using Laptop camera")
                self.cap = cv2.VideoCapture(0)
                if not self.cap.isOpened():
                    raise Exception("Failed to open laptop camera")
                self.get_logger().info("Laptop camera initialized successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize laptop camera: {e}")
                rclpy.shutdown()
                return

        # Timer callback at 10Hz to capture images continuously
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Heartbeat flag and counter
        self.last_heartbeat_time = time.time()
        self.heartbeat_interval = 5.0  # Seconds

    def heartbeat_callback(self):
        """
        Heartbeat callback to log every 5 seconds to confirm the system is alive.
        """
        current_time = time.time()
        elapsed_time = current_time - self.last_heartbeat_time
        if elapsed_time >= self.heartbeat_interval:
            self.get_logger().info(f"Heartbeat: Camera node is alive (elapsed time: {elapsed_time:.2f}s).")
            self.last_heartbeat_time = current_time  # Reset heartbeat time

    def timer_callback(self):
        """
        Capture an image from the camera (either RealSense or laptop webcam) and publish it.
        """
        try:
            if self.realsense:
                # Capture frame from RealSense camera
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    self.get_logger().error("No color frame received from RealSense camera.")
                    return
                # Convert RealSense frame to OpenCV format
                color_image = np.asanyarray(color_frame.get_data())
            else:
                # Capture frame from laptop camera
                ret, color_image = self.cap.read()
                if not ret:
                    self.get_logger().error("Failed to capture frame from laptop camera.")
                    return

            # Convert to ROS image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.publisher.publish(ros_image)
            self.get_logger().info(f"Published image frame: {color_image.shape}")

        except Exception as e:
            self.get_logger().error(f"Error during image capture or publishing: {e}")

    def destroy(self):
        """Release resources gracefully on shutdown."""
        if self.realsense:
            self.pipeline.stop()
        else:
            self.cap.release()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
