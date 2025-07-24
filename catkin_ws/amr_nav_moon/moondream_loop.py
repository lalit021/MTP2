import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # To subscribe to image messages
from cv_bridge import CvBridge
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch
import cv2
import numpy as np

class MoondreamNode(Node):
    def __init__(self):
        super().__init__('moondream_node')

        # Load the Moondream2 model directly from Hugging Face without an API key
        self.model = AutoModelForCausalLM.from_pretrained(
            "vikhyatk/moondream2",  # Specify the model name
            revision="2025-01-09",
            trust_remote_code=True,
            device_map={"": "cuda"}  # or use "cpu" if no GPU
        )

        self.tokenizer = AutoTokenizer.from_pretrained("vikhyatk/moondream2")

        # Bridge to convert ROS Image to OpenCV Image
        self.bridge = CvBridge()

        # Subscribe to image topic (camera/image topic)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',  # Replace with your actual image topic
            self.image_callback,
            10  # Queue size
        )

        self.get_logger().info("Moondream Node Initialized and Subscribed to /camera/image")

    def image_callback(self, msg):
        """Callback function to process image once it is received."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert OpenCV image to PIL image for processing
            image_pil = PILImage.fromarray(cv_image)

            # Set the prompt directly (e.g., "detect econorm packet")
            prompt = "detect econorm packet"  # Modify as needed

            # Tokenize the input prompt
            inputs = self.tokenizer(prompt, return_tensors="pt").to(self.model.device)

            # Generate output from the model based on the prompt
            outputs = self.model.generate(**inputs, max_length=50)

            # Decode the output text from the model
            output_text = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

            # Log the generated output text
            self.get_logger().info(f"Generated Text: {output_text}")

            # Visualize the result (e.g., overlay the generated text on the image)
            self.visualize_results(image_pil, output_text)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def visualize_results(self, image, result_text):
        """Display the image with the generated result overlaid."""
        # Convert the PIL image to OpenCV format (BGR)
        image_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

        # Overlay the generated result text on the image
        cv2.putText(image_cv, result_text, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Show the image with the result text
        cv2.imshow("Result", image_cv)
        cv2.waitKey(0)  # Wait until a key is pressed to close the window
        cv2.destroyAllWindows()


def main(args=None):
    """Initialize ROS2 node and process image."""
    rclpy.init(args=args)
    moondream_node = MoondreamNode()
    rclpy.spin(moondream_node)  # Keep the node alive to process images
    rclpy.shutdown()  # Shutdown ROS2 node gracefully

if __name__ == '__main__':
    main()
