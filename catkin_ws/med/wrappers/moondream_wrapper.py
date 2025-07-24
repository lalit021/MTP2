import numpy as np
import cv2
from PIL import Image

class MoondreamWrapper:
    def __init__(self, model):
        """
        Initialize with a Moondream model instance.
        :param model: Moondream model (e.g., md.vl(api_key="..."))
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

        # Query model
        result = self.model.point(image_pil, prompt)
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
