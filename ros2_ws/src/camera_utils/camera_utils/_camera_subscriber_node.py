#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class CameraSubscriberNode(Node):
    def __init__(self, model_callback, image_topic='camera/image', heartbeat_hz=1.0):
        """
        A generic camera subscriber node that accepts a callback for processing the image.

        Arguments:
            model_callback: A function that processes the image.
            image_topic: The ROS2 topic to subscribe to for image data. Defaults to 'camera/image'.
            heartbeat_hz: The frequency of the heartbeat message in Hz. Defaults to 1 Hz.
        """
        super().__init__('camera_subscriber_node')

        # Declare parameters
        self.declare_parameter('heartbeat_hz', heartbeat_hz)
        self.declare_parameter('image_topic', image_topic)

        # Get parameters
        heartbeat_hz = self.get_parameter('heartbeat_hz').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # Create subscriber for images
        self.subscriber = self.create_subscription(
            Image,
            image_topic,  # Remap the topic based on the passed argument
            self.image_callback,
            10
        )

        # Create a publisher for the heartbeat
        self.heartbeat_publisher = self.create_publisher(String, 'heartbeat', 10)

        # Set up a timer to send the heartbeat message based on the frequency parameter
        self.heartbeat_timer = self.create_timer(1.0 / heartbeat_hz, self.publish_heartbeat)

        self.bridge = CvBridge()
        self.model_callback = model_callback

        # Log message to confirm the subscriber is running
        self.get_logger().info(f"Camera Subscriber Node is running and listening to '{image_topic}' topic.")
        self.get_logger().info("Heartbeat system initialized on 'heartbeat' topic.")

    def image_callback(self, msg):
        """
        Callback to process the received image message.
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image using the provided model callback
            self.model_callback(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def publish_heartbeat(self):
        """
        Publishes a heartbeat message to indicate the node is still alive.
        """
        heartbeat_msg = String()
        heartbeat_msg.data = "CameraSubscriberNode is alive"
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().info("Heartbeat sent.")

def main(args=None):
    rclpy.init(args=args)
    
    # Example of model callback function
    def model_callback(cv_image):
        """
        Example model callback to process and display the image.
        """
        try:
            # Convert the image to grayscale as an example processing step
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Display the original and processed images
            cv2.imshow("Original Image", cv_image)

            # Wait for a brief moment to display the image
            cv2.waitKey(1)
        except Exception as e:
            print(f"Error in model_callback: {e}")

    # Initialize the CameraSubscriberNode with the given parameters
    camera_subscriber_node = CameraSubscriberNode(model_callback)
    
    rclpy.spin(camera_subscriber_node)
    
    # Cleanup before shutting down
    camera_subscriber_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
