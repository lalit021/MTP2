import os
import csv
import numpy as np
import cv2
from PIL import Image
from dotenv import load_dotenv
from transformers import AutoModelForCausalLM

# Initialize Moondream model with CUDA support (no API key needed)
model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2",
    revision="2025-01-09",
    trust_remote_code=True,
    device_map={"": "cuda"}  # Use GPU (CUDA) if available
)

# Moondream Wrapper class
class MoondreamWrapper:
    def __init__(self, model):
        """
        Initialize with a Moondream model instance.
        :param model: Moondream model (e.g., from transformers)
        """
        self.model = model

    def query_point(self, image_pil, prompt):
        """
        Runs the point detection on a PIL image directly (without needing a file path).
        :param image_pil: PIL Image object
        :param prompt: Textual prompt for object detection
        :return: (OpenCV image, list of pixel coordinates)
        """
        # Convert to OpenCV format
        image_np = np.array(image_pil.convert("RGB"))
        image_cv = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        h, w, _ = image_cv.shape

        # Query model for points (assuming model has a point detection method)
        result = self.model.point(image_pil, prompt)  # Assuming this method exists in the model
        pixel_points = [
            (int(pt["x"] * w), int(pt["y"] * h)) for pt in result.get("points", [])
        ]

        return image_cv, pixel_points

    def visualize_points(self, image_cv, points, window_title="Marked Image"):
        """
        Visualize point(s) on the image.
        :param image_cv: OpenCV image (BGR)
        :param points: List of (x, y) pixel coordinates
        :param window_title: Title of the window
        """
        for (x, y) in points:
            cv2.circle(image_cv, (x, y), radius=20, color=(255, 0, 0), thickness=-1)
            cv2.putText(image_cv, f"({x},{y})", (x + 25, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
        cv2.imshow(window_title, image_cv)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# CSV file setup
csv_file = "src/aruco_pose/detected_points.csv"
csv_header = ['Frame No', 'Prompt', 'Model Output']

if not os.path.exists(csv_file):
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)

frame_no = 1  # Frame counter

wrapper = MoondreamWrapper(model)

while True:
    input("Press Enter to capture points or Ctrl+C to exit.")

    # Load image
    img_path = "src/aruco_pose/saved_images/latest_image.png"
    if not os.path.exists(img_path):
        print(f"Image not found at {img_path}. Skipping...")
        continue

    img = Image.open(img_path)

    # Prompt and query
    prompt = "white paper inside blue box"
    image_cv, model_output = wrapper.query_point(img, prompt)

    # Print and save model output
    print(f"\nModel Output for Frame {frame_no}:\n{model_output}\n")

    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([frame_no, prompt, str(model_output)])
        print(f"Frame {frame_no}: Output saved to CSV.")

    # Visualize after saving
    wrapper.visualize_points(image_cv, model_output)

    frame_no += 1
