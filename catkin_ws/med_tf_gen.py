import csv
import json

def csv_to_tf_dict(csv_file_path, parent_frame="base"):
    tf_dict = {"transforms": []}

    with open(csv_file_path, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            frame_id = row["Frame"].strip()
            x = float(row["X"])
            y = float(row["Y"])
            z = float(row["Z"])

            transform_entry = {
                "translation": [x, y, z],
                "child_frame": f"marker_{frame_id}",
                "rotation": [0.0, 0.0, 0.0, 1.0],  # Identity quaternion
                "parent_frame": parent_frame
            }

            tf_dict["transforms"].append(transform_entry)

    return tf_dict

# Example usage:
csv_path = "/home/dell/MTP2/catkin_ws/saved_data/electral_coordinates.csv"  # replace with actual path
tf_result = csv_to_tf_dict(csv_path)

# Save to JSON file (optional)
with open("tf_transforms.json", "w") as f:
    json.dump(tf_result, f, indent=4)

# Print it
print(json.dumps(tf_result, indent=4))
