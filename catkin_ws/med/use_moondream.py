from PIL import Image
import moondream as md
from wrappers.moondream_wrapper import MoondreamWrapper
import os
import csv
import time
import cv2
from dotenv import load_dotenv

# Load environment variables
load_dotenv("api_key.env")
api_key = os.getenv("moondream_api_key")

# Initialize Moondream model and wrapper
model = md.vl(api_key=api_key)
wrapper = MoondreamWrapper(model)

# CSV file setup
csv_file = "detected_points.csv"
csv_header = ['Frame No', 'Prompt', 'Model Output']

if not os.path.exists(csv_file):
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)

frame_no = 1  # Frame counter

while True:
    input("Press Enter to capture points or Ctrl+C to exit.")

    # Load image
    img_path = "src/aruco_pose/saved_images/latest_image.png"
    if not os.path.exists(img_path):
        print(f"Image not found at {img_path}. Skipping...")
        continue

    img = Image.open(img_path)

    # Prompt and query
    prompt = "electral"
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
