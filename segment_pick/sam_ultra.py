import cv2
import numpy as np
import pandas as pd
from ultralytics import SAM

# --- Parameters ---
input_path = "1.jpg"
size_ratio_threshold = 0.25
exclude_large_area_threshold = 0.95  # Ignore masks covering 95%+ of the image

# --- Step 1: Run SAM segmentation (with a larger model) ---
model = SAM("sam_b.pt")  # Use a larger model for more complex segmentation
results = model(input_path, save=False)
masks = results[0].masks.data.cpu().numpy()  # shape: (N, H, W)

# --- Step 2: Collect mask data ---
image = cv2.imread(input_path)
height, width, _ = image.shape

mask_data = []
for idx, mask in enumerate(masks):
    area = int(mask.sum())
    if area < 10:
        continue  # Skip tiny masks that are likely noise

    # Exclude masks that cover the majority of the image (e.g., > 95%)
    if area / (height * width) > exclude_large_area_threshold:
        continue

    y, x = np.where(mask)
    bbox = [x.min(), y.min(), x.max(), y.max()]
    
    mask_data.append({
        "id": idx,
        "area": area,
        "bbox": bbox,
        "mask": mask.astype(bool)
    })

# --- Step 3: Remove masks fully inside larger ones ---
to_remove = set()
for i, m1 in enumerate(mask_data):
    for j, m2 in enumerate(mask_data):
        if i == j or m1["area"] >= m2["area"]:
            continue
        if np.all(m2["mask"][m1["mask"]]):  # Check if m1 is fully inside m2
            ratio = m1["area"] / m2["area"]
            if ratio < size_ratio_threshold:
                to_remove.add(i)
                break

# --- Step 4: Filter masks ---
filtered = [m for i, m in enumerate(mask_data) if i not in to_remove]

# --- Step 5: Save CSV with additional bounding box details ---
rows = []
for m in filtered:
    x1, y1, x2, y2 = m["bbox"]
    rows.append({
        "id": m["id"],
        "area": m["area"],
        "bbox_xmin": x1,
        "bbox_ymin": y1,
        "bbox_xmax": x2,
        "bbox_ymax": y2
    })
df = pd.DataFrame(rows)
df.to_csv("filtered_segmentations.csv", index=False)
print(f"✅ Saved {len(rows)} filtered masks to filtered_segmentations.csv")

# --- Step 6: Visualize & save image with numbered bounding boxes ---
overlay = image.copy()
for m in filtered:
    color = np.random.randint(0, 255, size=3).tolist()  # Random color for each mask
    mask = m["mask"]
    
    # Apply color to the mask region
    for c in range(3):
        overlay[..., c][mask] = 0.6 * overlay[..., c][mask] + 0.4 * color[c]
    
    # Draw the bounding box and add the ID number to the image
    x1, y1, x2, y2 = m["bbox"]
    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
    cv2.putText(overlay, str(m["id"]), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# Save the final image
cv2.imwrite("filtered_segmented_output_with_bboxes.jpg", overlay)
print("✅ Saved segmentation visualization with bounding boxes to filtered_segmented_output_with_bboxes.jpg")

