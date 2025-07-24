import pyrealsense2 as rs

# Initialize the pipeline
pipeline = rs.pipeline()
config = rs.config()

# Start the pipeline
pipeline.start(config)

# Get the active stream's intrinsics (from the first color stream profile)
profile = pipeline.get_active_profile()
video_stream = profile.get_stream(rs.stream.color)  # or rs.stream.depth depending on your use case
intrinsics = video_stream.as_video_stream_profile().get_intrinsics()

# Print out the camera matrix and distortion coefficients
print("Camera Matrix:")
print("fx:", intrinsics.fx)
print("fy:", intrinsics.fy)
print("ppx:", intrinsics.ppx)
print("ppy:", intrinsics.ppy)

print("Distortion Coefficients:")
print("k1, k2, p1, p2, k3:", intrinsics.coeffs)

# Stop the pipeline
pipeline.stop()
