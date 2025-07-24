from PIL import Image
import moondream as md
from wrappers.moondream_wrapper import MoondreamWrapper

# Load model
from dotenv import load_dotenv
import os

load_dotenv("api_key.env")  # Explicitly point to your file

api_key = os.getenv("moondream_api_key")


model = md.vl(api_key=api_key)
# Create wrapper
wrapper = MoondreamWrapper(model)

# Assume the image comes from a live camera feed, or another source.
# For example, `img` could be a PIL Image object that you capture from the camera.
# For now, you can use a mock image or load an image from another source.

# Mock example to show how you'd work with an image object
# Here you can replace this with the actual image from the camera feed
img = Image.open("1.jpeg")  # This can be replaced with live camera image

# Run query
image_cv, points = wrapper.query_point(img, "detect econorm packet")

# Show result
wrapper.visualize_points(image_cv, points)
