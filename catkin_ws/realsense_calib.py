import pyrealsense2 as rs

def print_intrinsics(intr):
    print("  Width:        ", intr.width)
    print("  Height:       ", intr.height)
    print("  fx (focal x): ", intr.fx)
    print("  fy (focal y): ", intr.fy)
    print("  ppx:          ", intr.ppx)
    print("  ppy:          ", intr.ppy)
    print("  Distortion:   ", intr.model)
    print("  Coefficients: ", intr.coeffs)

def get_realsense_intrinsics():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline_profile = pipeline.start(config)

    # Wait for a frame to ensure the camera is streaming
    frames = pipeline.wait_for_frames()

    # Get stream profiles
    color_profile = frames.get_color_frame().profile.as_video_stream_profile()
    depth_profile = frames.get_depth_frame().profile.as_video_stream_profile()

    print("\n📷 Color Camera Intrinsics:")
    color_intr = color_profile.get_intrinsics()
    print_intrinsics(color_intr)

    print("\n🌫️ Depth Camera Intrinsics:")
    depth_intr = depth_profile.get_intrinsics()
    print_intrinsics(depth_intr)

    pipeline.stop()

if __name__ == "__main__":
    get_realsense_intrinsics()
