import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import cv2  # For colormap heatmap

# RealSense pipeline setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

pc = rs.pointcloud()

def show_images(depth_frame, color_frame):
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Normalize depth for heatmap
    depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.03)
    depth_colormap = cv2.applyColorMap(depth_image_scaled, cv2.COLORMAP_JET)
    depth_colormap_rgb = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)

    # Display side-by-side
    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.title("Depth Heatmap")
    plt.imshow(depth_colormap_rgb)
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.title("Color Image")
    plt.imshow(color_image)
    plt.axis('off')

    plt.tight_layout()
    plt.show()

def get_point_clouds(depth_frame):
    points = pc.calculate(depth_frame)
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)

    # Downsample
    voxel_size = 0.01  # Finer resolution
    downsampled = pcd.voxel_down_sample(voxel_size)

    # Estimate normals
    downsampled.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30)
    )

    # Scale normals (increase length)
    normal_scale = 0.2  # Adjust this value for desired normal length
    scaled_normals = np.asarray(downsampled.normals) * normal_scale
    downsampled.normals = o3d.utility.Vector3dVector(scaled_normals)

    return pcd, downsampled

try:
    # Capture frames
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        raise RuntimeError("Frames not received.")

    # Step 1: Show images
    show_images(depth_frame, color_frame)

    # Step 2: Get point clouds
    original_pcd, down_pcd_with_normals = get_point_clouds(depth_frame)

    # Step 3: Show downsampled point cloud
    print("Showing downsampled point cloud with LONGER normals...")
    o3d.visualization.draw_geometries(
        [down_pcd_with_normals],
        point_show_normal=True,
        window_name="Downsampled Point Cloud with Long Normals"
    )

finally:
    pipeline.stop()
