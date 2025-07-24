import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image
import requests
from io import BytesIO
import cv2
import numpy as np

# Check if a GPU is available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Download and save the model
def download_and_save_model(model_name, save_path):
    # Load model and tokenizer
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        revision="2025-01-09",
        trust_remote_code=True,
    )
    tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
    
    # Move model to GPU if available
    model.to(device)
    
    # Save the model and tokenizer
    model.save_pretrained(save_path)
    tokenizer.save_pretrained(save_path)
    print(f"Model and tokenizer saved to {save_path}")

    return model, tokenizer


# Load image from URL (or local file)
def load_image(image_path):
    if image_path.startswith("http"):
        response = requests.get(image_path)
        image = Image.open(BytesIO(response.content))
    else:
        image = Image.open(image_path)
    return image


import cv2
import numpy as np

# Draw bounding boxes and points on the image
def draw_boxes_and_points(image, objects, points):
    # Convert PIL image to OpenCV format (BGR)
    image_cv = np.array(image)
    image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR)

    # Get image dimensions
    height, width, _ = image_cv.shape

    # Debugging: Print the object structure
    print("\nObjects detected:")
    print(objects)

    # Draw bounding boxes for detected objects (example with "truck" detection)
    for obj in objects:
        print(f"Object details: {obj}")  # Debug print for each object
        # Convert fractional coordinates to pixel coordinates
        x_min = int(obj['x_min'] * width)
        y_min = int(obj['y_min'] * height)
        x_max = int(obj['x_max'] * width)
        y_max = int(obj['y_max'] * height)

        # Draw rectangle: (x_min, y_min, x_max, y_max)
        cv2.rectangle(image_cv, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Green box

    # Draw points for detected items (example with "person" detection)
    for point in points:
        x, y = point['x'], point['y']
        # Ensure x and y are integers
        x, y = int(x), int(y)
        # Draw circle for points
        cv2.circle(image_cv, (x, y), 5, (0, 0, 255), -1)  # Red points

    # Display the image
    cv2.imshow("Image with Boxes and Points", image_cv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Use the model for captioning and queries
def use_model_for_queries(model, image, query):
    print("\nCaptioning the image:")
    
    # Short captioning
    print("Short caption:")
    caption_short = model.caption(image, length="short")["caption"]
    print(caption_short)
    
    # Normal captioning
    print("\nNormal caption:")
    for t in model.caption(image, length="normal", stream=True)["caption"]:
        print(t, end="", flush=True)
    print()
    caption_normal = model.caption(image, length="normal")["caption"]
    print(caption_normal)
    
    # Visual Querying
    print("\nVisual query: 'How many people are in the image?'")
    query_result = model.query(image, query)["answer"]
    print(f"Answer: {query_result}")
    
    # Object Detection: Detecting trucks
    print("\nObject detection: 'truck'")
    objects = model.detect(image, "truck")["objects"]
    print(f"Found {len(objects)} truck(s)")
    
    # Pointing: Detecting people
    print("\nPointing: 'truck'")
    points = model.point(image, "truck")["points"]
    print(f"Found {len(points)} truck(s)")

    # Draw the results on the image
    draw_boxes_and_points(image, objects, points)


# Main function to download model, load image, and run queries
def main():
    # Download and save the model
    model_name = "vikhyatk/moondream2"
    save_path = "./moondream2_model"
    model, tokenizer = download_and_save_model(model_name, save_path)
    
    # Load image
    image_path = "truck.jpg"  # Can be a URL or local path
    image = load_image(image_path)

    # Use the model for queries
    query = "What is written on the wheel?"
    use_model_for_queries(model, image, query)


if __name__ == "__main__":
    main()
