#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from aruco_pose.srv import Get3DPoint, Get3DPointResponse
import numpy as np
import cv2
import pyrealsense2 as rs
import struct

class RealSenseCameraNode:
    def __init__(self):
        rospy.init_node('realsense_camera_node')

        # Publishers for color, depth images and point cloud
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
        self.pc_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)

        self.bridge = CvBridge()

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # Get depth intrinsics for 3D point calculation
        profile = self.pipeline.get_active_profile()
        self.depth_stream = profile.get_stream(rs.stream.depth)
        self.depth_intrinsics = self.depth_stream.as_video_stream_profile().get_intrinsics()

        # Storage for latest frames
        self.latest_color_frame = None
        self.latest_depth_frame = None

        # Service to get 3D coordinates
        self.coord_service = rospy.Service('get_3d_coordinates', Get3DPoint, self.handle_get_3d_coordinates)

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            rospy.logwarn("No frames received from RealSense.")
            return

        self.latest_color_frame = color_frame
        self.latest_depth_frame = depth_frame

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert depth image to color map for visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Publish images
        self.color_pub.publish(self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8'))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8'))

        # Publish point cloud
        self.publish_pointcloud(depth_frame)

    def publish_pointcloud(self, depth_frame):
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        points = []
        for v in range(height):
            for u in range(width):
                depth = depth_frame.get_distance(u, v)
                if depth == 0:
                    # Invalid point
                    points.append([float('nan'), float('nan'), float('nan')])
                    continue
                point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [u, v], depth)
                points.append(point)

        # Create PointCloud2 message
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_link"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc2_msg)

    def handle_get_3d_coordinates(self, req):
        if self.latest_depth_frame is None:
            rospy.logwarn("No depth frame available for service request.")
            return Get3DPointResponse(0.0, 0.0, 0.0, False)

        depth = self.latest_depth_frame.get_distance(req.u, req.v)
        if depth <= 0:
            rospy.logwarn("Invalid depth at pixel ({}, {}).".format(req.u, req.v))
            return Get3DPointResponse(0.0, 0.0, 0.0, False)

        point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [req.u, req.v], depth)
        X, Y, Z = point_3d
        return Get3DPointResponse(X, Y, Z, True)

    def spin(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            self.publish_frames()
            rate.sleep()

    def cleanup(self):
        self.pipeline.stop()
        rospy.loginfo("RealSenseCameraNode stopped.")

if __name__ == '__main__':
    node = RealSenseCameraNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()
