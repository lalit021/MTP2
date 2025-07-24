import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import json

def rotate_point(point, normal):
    """
    Rotate a single point using Rodrigues' rotation formula to align with the normal vector.
    """
    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
    
    # Create the rotation axis (perpendicular to the plane of the circle)
    axis = np.cross([0, 0, 1], normal)
    axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) != 0 else np.array([1, 0, 0])
    
    # Angle between the z-axis and the normal vector
    angle = np.arccos(np.dot([0, 0, 1], normal))
    
    # Rodrigues' rotation formula
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    
    rotation_matrix = np.array([
        [cos_angle + axis[0]**2 * (1 - cos_angle), axis[0] * axis[1] * (1 - cos_angle) - axis[2] * sin_angle, axis[0] * axis[2] * (1 - cos_angle) + axis[1] * sin_angle],
        [axis[1] * axis[0] * (1 - cos_angle) + axis[2] * sin_angle, cos_angle + axis[1]**2 * (1 - cos_angle), axis[1] * axis[2] * (1 - cos_angle) - axis[0] * sin_angle],
        [axis[2] * axis[0] * (1 - cos_angle) - axis[1] * sin_angle, axis[2] * axis[1] * (1 - cos_angle) + axis[0] * sin_angle, cos_angle + axis[2]**2 * (1 - cos_angle)]
    ])
    
    # Rotate the point
    rotated_point = np.dot(rotation_matrix, point)
    return rotated_point

def draw_3d_circle(center_x, center_y, center_z, radius, step_size, normal, json_filename="circle_points.json"):
    # Generate points for the circle in the XY plane (this is the default orientation)
    angles = np.arange(0, 2 * np.pi, step_size)
    x_points = radius * np.cos(angles)
    y_points = radius * np.sin(angles)
    z_points = np.zeros_like(angles)
    
    # Stack the points together (XY plane)
    points = np.vstack([x_points, y_points, z_points])

    # Translate the points to the center of the circle
    points += np.array([[center_x], [center_y], [center_z]])
    
    # Apply rotation to each point individually
    rotated_points = np.array([rotate_point(point, normal) for point in points.T]).T  # Rotate each point

    # Prepare the data for JSON
    points_data = []
    for i in range(rotated_points.shape[1]):
        points_data.append({
            "x": rotated_points[0][i],
            "y": rotated_points[1][i],
            "z": rotated_points[2][i]
        })
    
    # Save points to JSON file
    with open(json_filename, 'w') as json_file:
        json.dump(points_data, json_file, indent=4)
    
    # Plot the circle in 3D
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(rotated_points[0], rotated_points[1], rotated_points[2], label=f'Circle with radius {radius}')
    
    # Label the center
    ax.scatter(center_x, center_y, center_z, color='red', label=f'Center ({center_x}, {center_y}, {center_z})')
    
    ax.set_title('3D Circle Drawing')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()
    
    plt.show()

# Example usage:
center_x = float(input("Enter the center X coordinate: "))
center_y = float(input("Enter the center Y coordinate: "))
center_z = float(input("Enter the center Z coordinate: "))
radius = float(input("Enter the radius of the circle: "))
step_size = float(input("Enter the step size (in radians, e.g. 0.1): "))

# Get the normal vector from the user (the plane's perpendicular orientation)
normal_x = float(input("Enter the normal vector X component: "))
normal_y = float(input("Enter the normal vector Y component: "))
normal_z = float(input("Enter the normal vector Z component: "))

normal = np.array([normal_x, normal_y, normal_z])

# Draw the circle in 3D space and save points to a JSON file
draw_3d_circle(center_x, center_y, center_z, radius, step_size, normal)
