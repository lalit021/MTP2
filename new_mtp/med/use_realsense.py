from wrappers.realsense_wrapper import RealSenseWrapper
import time
import os

def my_pixel_suggester(image, prompt, model):
    """
    Replace this with your actual model-based logic.
    For example: Run inference on the image and return a list of (x, y) pixels.
    """
    if prompt == "Find the red ball":
        return [(200, 400), (250, 200)]
    return []

if __name__ == "__main__":
    cam = RealSenseWrapper()
    model = None  # Load your model here, if needed
    prompt = "Find the red ball"  # Example instruction

    # Define the folder where you want to save the output files
    save_folder = "output_coordinates"  # Folder name
    os.makedirs(save_folder, exist_ok=True)  # Create the folder if it doesn't exist

    try:
        frame_idx = 0
        while True:
            print(f"[INFO] Capturing frame {frame_idx + 1}")
            xyz_list, pixel_list, color_img, depth_img = cam.process_frame(
                pixel_suggester=my_pixel_suggester,
                prompt=prompt,
                model=model
            )

            if xyz_list and any(pt is not None for pt in xyz_list):
                print(f"[INFO] Frame {frame_idx + 1}:\n  Pixels: {pixel_list}\n  XYZs: {xyz_list}")
                
                # Save the coordinates to a file in the specified folder
                with open(f"{save_folder}/frame_{frame_idx + 1}_coordinates.txt", "w") as f:
                    for i, ((px, py), coord) in enumerate(zip(pixel_list, xyz_list)):
                        if coord is not None:
                            f.write(f"Point {i + 1} -> Pixel: ({px}, {py}), XYZ: ({coord[0]:.3f}, {coord[1]:.3f}, {coord[2]:.3f})\n")
                        else:
                            f.write(f"Point {i + 1} -> Pixel: ({px}, {py}), XYZ: Not available\n")
            else:
                print(f"[INFO] Frame {frame_idx + 1}: No valid 3D points received.")

            frame_idx += 1

            # You can optionally add a delay or break condition here
            input("Press Enter to continue to the next frame, or Ctrl+C to stop...")

    except KeyboardInterrupt:
        print("\n[INFO] Exiting on user interrupt.")
    finally:
        cam.stop()
