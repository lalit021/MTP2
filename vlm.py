import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image
import os
import requests
from io import BytesIO

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

# Use the model for captioning
def use_model_for_captioning(model, image):
    print("\nCaptioning the image:")
    
    # Short captioning
    print("Short caption:")
    caption_short = model.caption(image, length="short")["caption"]
    print(caption_short)
    
    # Normal captioning
    print("\nNormal caption:")
    caption_normal = model.caption(image, length="normal")["caption"]
    print(caption_normal)

# Loop through images in a folder and describe only the latest ones
def describe_latest_images_in_folder(folder_path, model, n_latest=5):
    # Get a list of image files in the folder and sort them by modification time (latest first)
    image_files = [
        os.path.join(folder_path, f) for f in os.listdir(folder_path)
        if f.lower().endswith(('.png', '.jpg', '.jpeg'))
    ]
    image_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)  # Sort by modified time (most recent first)

    # Process only the latest N images
    for image_path in image_files[:n_latest]:
        filename = os.path.basename(image_path)
        print(f"\nProcessing latest image: {filename}")
        image = load_image(image_path)
        
        # Use the model to caption the image
        use_model_for_captioning(model, image)

# Main function to download model, load image, and run queries
def main():
    # Download and save the model
    model_name = "vikhyatk/moondream2"
    save_path = "./moondream2_model"
    model, tokenizer = download_and_save_model(model_name, save_path)
    
    # Path to the folder containing images
    # folder_path = r"/home/dell/MTP@/MTP2/catkin_ws/src/aruco_pose/saved_images"  # Replace with your folder path
    folder_path = "img"
    
    # Describe the latest images in the folder (change `n_latest` to control how many images to process)
    describe_latest_images_in_folder(folder_path, model, n_latest=5)


if __name__ == "__main__":
    main()
