import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseWrapper:
    def __init__(self, try_resolutions=[(1280, 720), (848, 480), (640, 480)], fps=30):
        self.pipe = rs.pipeline()
        self.pc = rs.pointcloud()

        success = False
        for res in try_resolutions:
            try:
                self.cfg = rs.config()
                self.cfg.enable_stream(rs.stream.color, *res, rs.format.bgr8, fps)
                self.cfg.enable_stream(rs.stream.depth, *res, rs.format.z16, fps)
                self.pipe.start(self.cfg)
                self.resolution = res
                print(f"[INFO] RealSense pipeline started at resolution {res} and {fps} FPS.")
                success = True
                break
            except RuntimeError as e:
                print(f"[WARNING] Failed to start RealSense at {res}: {e}")

        if not success:
            raise RuntimeError("Unable to start RealSense pipeline with any resolution.")

    def capture_frame(self):
        frames = self.pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None, None

        self.pc.map_to(color_frame)
        points = self.pc.calculate(depth_frame)
        vertices = np.asanyarray(points.get_vertices())
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image, vertices

    def get_coordinates(self, vertices, pixel):
        h, w = self.resolution
        x, y = pixel
        if 0 <= x < w and 0 <= y < h:
            idx = y * w + x
            v = vertices[idx]
            if np.isnan(v[2]) or v[2] == 0.0:
                return None
            return (float(v[0]), float(v[1]), float(v[2]))
        return None

    def draw_point(self, image, pixel, xyz):
        cv2.circle(image, pixel, 6, (255, 255, 0), -1)
        if xyz:
            try:
                cv2.putText(image, f"({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})",
                            (pixel[0] + 10, pixel[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            except Exception:
                pass  # Handle malformed point gracefully

    def process_frame(self, pixel_suggester=None, prompt=None, model=None):
        color_image, depth_image, vertices = self.capture_frame()
        if color_image is None:
            return None, None, None, None

        h, w, _ = color_image.shape
        suggested_pixels = []

        if pixel_suggester:
            result = pixel_suggester(color_image, prompt, model)
            if result:
                suggested_pixels = result

        xyz_list = [self.get_coordinates(vertices, pixel) for pixel in suggested_pixels]

        resized_color = cv2.resize(color_image.copy(), (640, 480))
        resized_depth = cv2.resize(depth_image, (640, 480))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(resized_depth, alpha=0.03), cv2.COLORMAP_JET)

        scale_x = 640 / w
        scale_y = 480 / h

        for pixel, xyz in zip(suggested_pixels, xyz_list):
            scaled_pixel = (int(pixel[0] * scale_x), int(pixel[1] * scale_y))
            self.draw_point(resized_color, scaled_pixel, xyz)
            self.draw_point(depth_colormap, scaled_pixel, xyz)

        stacked = np.hstack((resized_color, depth_colormap))
        cv2.imshow("Color | Depth", stacked)
        cv2.waitKey(1)

        return xyz_list, suggested_pixels, resized_color, depth_colormap

    def stop(self):
        self.pipe.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cam = RealSenseWrapper()
    try:
        while True:
            cam.process_frame()
    except KeyboardInterrupt:
        print("Stopping...")
        cam.stop()
