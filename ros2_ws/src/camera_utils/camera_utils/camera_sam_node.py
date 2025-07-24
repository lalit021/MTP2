from camera_utils._camera_subscriber_node import CameraSubscriberNode
from camera_utils._sam_segment import SAM2MaskPredictor
import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge
from PIL import Image as PILImage
from threading import Lock
import time

class CameraSegmentationNode(CameraSubscriberNode):
    def __init__(self, model_cfg, checkpoint_path, image_topic="camera/image", interval=2.0):
        """
        A specialized node to perform segmentation on camera feed at regular intervals.

        Args:
            model_cfg (str): Path to the model configuration file.
            checkpoint_path (str): Path to the model checkpoint file.
            image_topic (str): ROS2 topic to subscribe to for images.
            interval (float): Interval in seconds to perform segmentation.
        """
        self.predictor = SAM2MaskPredictor(model_cfg=model_cfg, checkpoint_path=checkpoint_path)
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = Lock()
        self.interval = interval
        self.last_segmentation_time = time.time()

        super().__init__(self.image_callback, image_topic=image_topic, heartbeat_hz=1.0)

    def image_callback(self, msg):
        """
        Callback to process and store the latest image message.
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.latest_image = cv_image

            # Perform segmentation at the specified interval
            current_time = time.time()
            if current_time - self.last_segmentation_time >= self.interval:
                self.last_segmentation_time = current_time
                self.perform_segmentation()

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def perform_segmentation(self):
        """
        Perform segmentation on the latest image.
        """
        with self.image_lock:
            if self.latest_image is None:
                self.get_logger().warn("No image available for segmentation.")
                return

            # Convert OpenCV image to PIL Image for predictor
            pil_image = PILImage.fromarray(cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB))

            # Set the image for the predictor
            self.predictor.set_image(pil_image)

            # Compute the center point of the image
            height, width, _ = self.latest_image.shape
            center_point = np.array([[width // 2, height // 2]])  # Center point
            input_label = np.array([1])  # Foreground label

            # Perform segmentation using the center point
            masks, scores, logits = self.predictor.predict_with_points(center_point, input_label)

            # Display results
            self.predictor.display_results(
                pil_image, masks, scores, point_coords=center_point, input_labels=input_label, display_mode="top"
            )


def main(args=None):
    rclpy.init(args=args)

    # Configuration and checkpoint paths
    model_cfg = "configs/sam2.1/sam2.1_hiera_t.yaml"  # Replace with your model config path
    checkpoint_path = "/home/friday/MTP2/sam2/checkpoints/sam2.1_hiera_tiny.pt"  # Replace with your model checkpoint path

    # Initialize the camera segmentation node
    camera_segmentation_node = CameraSegmentationNode(model_cfg, checkpoint_path)

    rclpy.spin(camera_segmentation_node)

    # Cleanup before shutting down
    camera_segmentation_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
