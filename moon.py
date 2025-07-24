import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from rclpy.node import Node
import os

# Check if a GPU is available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Function to download and load model if it's not already in the specified directory
def download_and_load_model(model_path, model_version="main"):
    """
    Download and load the model and tokenizer from the local directory with the specified version.
    
    Args:
        model_path (str): The directory where the model is stored or will be downloaded.
        model_version (str): The version or branch name to use from the model repository (default is 'main').

    Returns:
        model, tokenizer: Loaded model and tokenizer.
    """
    if not os.path.exists(model_path):  # If model not found locally, download it
        print(f"Model path does not exist. Downloading model to {model_path}...")
        model = AutoModelForCausalLM.from_pretrained(
            "vikhyatk/moondream2",  # Using the model ID from Hugging Face
            revision=model_version,  # Specify version
            cache_dir=model_path,  # Cache in the provided model path
            trust_remote_code=True
        )
        tokenizer = AutoTokenizer.from_pretrained(
            "vikhyatk/moondream2",
            revision=model_version,
            cache_dir=model_path,
            trust_remote_code=True
        )
    else:
        print(f"Loading model from local path: {model_path}...")
        model = AutoModelForCausalLM.from_pretrained(
            model_path,
            revision=model_version,  # Use the provided version
            trust_remote_code=True,
        )
        tokenizer = AutoTokenizer.from_pretrained(model_path, trust_remote_code=True)
    
    model.to(device)
    print(f"Model and tokenizer loaded from {model_path}, version {model_version}")

    return model, tokenizer

# Draw bounding boxes and points on the image
def draw_boxes_and_points(image, objects, points):
    image_cv = np.array(image)
    image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR)

    height, width, _ = image_cv.shape

    # Draw bounding boxes for detected objects
    for obj in objects:
        x_min = int(obj['x_min'] * width)
        y_min = int(obj['y_min'] * height)
        x_max = int(obj['x_max'] * width)
        y_max = int(obj['y_max'] * height)

        cv2.rectangle(image_cv, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Green box

    # Draw points for detected items
    for point in points:
        x, y = point['x'], point['y']
        x, y = int(x), int(y)
        cv2.circle(image_cv, (x, y), 5, (0, 0, 255), -1)  # Red points

    cv2.imshow("Image with Boxes and Points", image_cv)
    cv2.waitKey(1)

# Use the model for captioning and queries
def use_model_for_queries(model, image, query, mode):
    objects = []
    points = []
    
    if mode == "describe":
        print("\nCaptioning the image:")
        caption_short = model.caption(image, length="short")["caption"]
        print("Short caption:", caption_short)
        
        caption_normal = model.caption(image, length="normal")["caption"]
        print("\nNormal caption:", caption_normal)

    elif mode == "detect":
        print("\nObject Detection: Detecting trucks")
        objects = model.detect(image, "truck")["objects"]
        print(f"Found {len(objects)} truck(s)")

        # Draw the results on the image
        draw_boxes_and_points(image, objects, points)

    elif mode == "point":
        print("\nPointing: Detecting trucks")
        points = model.point(image, "truck")["points"]
        print(f"Found {len(points)} truck(s)")

        # Draw the results on the image
        draw_boxes_and_points(image, objects, points)

    else:
        print("Invalid mode selected!")

    return objects, points

class CameraSubscriberNode(Node):
    def __init__(self, model, model_callback, mode):
        super().__init__('camera_subscriber_node')
        self.subscription = self.create_subscription(
            ROSImage,
            '/camera/color/image_raw',  # Adjust topic based on your camera setup
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.model = model
        self.model_callback = model_callback
        self.mode = mode

    def listener_callback(self, msg):
        try:
            # Convert the ROS image message to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")

            # Convert to PIL image for model processing
            image = Image.fromarray(cv_image)
            
            # Use the model for queries (in this case, according to the selected mode)
            use_model_for_queries(self.model, image, "Describe the image", self.mode)
            
            # Send the processed image to the callback (e.g., for visualization)
            self.model_callback(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Define the model version and path
    model_path = "./moondream2_model"  # Path where the model is already downloaded or will be downloaded
    model_version = "2025-01-09"  # Specify the model version you want to use
    
    # Download and load model if not already present
    model, tokenizer = download_and_load_model(model_path, model_version)

    # Read the mode from command-line arguments or set a default
    mode = "describe"  # Default mode, can be "describe", "detect", or "point"

    # Create the ROS2 Node
    def model_callback(cv_image):
        try:
            # Convert the image to grayscale as an example processing step
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Original Image", cv_image)
            cv2.imshow("Processed Image", gray_image)
            cv2.waitKey(1)
        except Exception as e:
            print(f"Error in model_callback: {e}")

    # Initialize CameraSubscriberNode with the mode
    camera_subscriber_node = CameraSubscriberNode(model, model_callback, mode)
    
    # Spin ROS2 node to keep processing
    rclpy.spin(camera_subscriber_node)
    
    # Cleanup
    camera_subscriber_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
