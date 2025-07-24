from wrappers.realsense_wrapper import RealSenseWrapper
from PIL import Image
import moondream as md
from wrappers.moondream_wrapper import MoondreamWrapper
from dotenv import load_dotenv
import os
import time
import cv2
import csv

# Load environment variable for API key
load_dotenv("api_key.env")
api_key = os.getenv("moondream_api_key")

# Initialize Moondream
model = md.vl(api_key=api_key)
wrapper = MoondreamWrapper(model)

# Directory to save results
SAVE_DIR = "saved_data"
os.makedirs(SAVE_DIR, exist_ok=True)

def my_pixel_suggester(image, prompt, model):
    img_pil = Image.fromarray(image)
    image_cv, points = wrapper.query_point(img_pil, prompt=prompt)
    return [(int(pt[0]), int(pt[1])) for pt in points] if points else []

if __name__ == "__main__":
    cam = RealSenseWrapper()
    prompt = "electral"  # Change as needed
    prompt_safe = prompt.replace(" ", "_").lower()
    
    csv_path = os.path.join(SAVE_DIR, f"{prompt_safe}_coordinates.csv")

    # Write CSV header
    if not os.path.exists(csv_path):
        with open(csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Frame", "Prompt", "Point #", "Pixel_X", "Pixel_Y", "X", "Y", "Z"])

    frame_idx = 0
    while True:
        print(f"[INFO] Capturing frame {frame_idx + 1}")
        xyz_list, pixel_list, img_color, img_depth = cam.process_frame(
            pixel_suggester=my_pixel_suggester,
            prompt=prompt,
            model=model
        )

        frame_txt_path = os.path.join(SAVE_DIR, f"frame_{frame_idx + 1}_coordinates.txt")
        with open(frame_txt_path, "w") as f_txt, open(csv_path, "a", newline='') as f_csv:
            writer = csv.writer(f_csv)

            if xyz_list and any(pt is not None for pt in xyz_list):
                f_txt.write(f"Prompt: {prompt}\n")
                for i, ((px, py), coord) in enumerate(zip(pixel_list, xyz_list)):
                    if coord:
                        line = f"Point {i + 1} -> Pixel: ({px}, {py}), XYZ: ({coord[0]:.3f}, {coord[1]:.3f}, {coord[2]:.3f})\n"
                        writer.writerow([frame_idx + 1, prompt, i + 1, px, py, f"{coord[0]:.3f}", f"{coord[1]:.3f}", f"{coord[2]:.3f}"])
                    else:
                        line = f"Point {i + 1} -> Pixel: ({px}, {py}), XYZ: Not available\n"
                        writer.writerow([frame_idx + 1, prompt, i + 1, px, py, "NaN", "NaN", "NaN"])
                    f_txt.write(line)
            else:
                f_txt.write(f"Prompt: {prompt}\nNo valid 3D points found in frame {frame_idx + 1}\n")
                print(f"[INFO] Frame {frame_idx + 1}: No valid 3D points received.")

        # Continue if no valid points
        if not xyz_list or all(pt is None for pt in xyz_list):
            print(f"[INFO] No valid points in frame {frame_idx + 1}, moving on...\n")
            frame_idx += 1
            continue

        input(f"Press Enter to capture frame {frame_idx + 2}...\n")
        frame_idx += 1

    cam.stop()
    cv2.destroyAllWindows()
