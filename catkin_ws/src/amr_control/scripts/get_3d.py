#!/usr/bin/env python
import rospy
import tf
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class PixelTo3D:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.depth_image = None
        self.color_image = None
        self.tf_publisher = TFPublisher()

        # Topics
        self.camera_info_topic = "/amr/camera_front/color/camera_info"
        self.depth_image_topic = "/amr/camera_front/depth/image_rect_raw"
        self.color_image_topic = "/amr/camera_front/color/image_raw"

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        rospy.Subscriber(self.color_image_topic, Image, self.color_callback)

        rospy.loginfo("Waiting for image, depth, and camera info...")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # === Set your pixel of interest here ===
        u, v = 320, 240  # Example pixel

        if self.camera_matrix is None or self.depth_image is None:
            rospy.logwarn("Missing camera info or depth image")
            return

        if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            rospy.logwarn("Pixel out of bounds")
            return

        depth = self.depth_image[v, u]
        if depth == 0 or np.isnan(depth):
            rospy.logwarn("Invalid depth at pixel ({}, {})".format(u, v))
            return

        # Camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Convert to 3D
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        rospy.loginfo("3D Point at pixel ({}, {}): x={:.3f}, y={:.3f}, z={:.3f}".format(u, v, x, y, z))

        # Publish as TF transform
        self.tf_publisher.publish_tf(x, y, z)

        # Visualize
        self.visualize(u, v, x, y, z)

    def visualize(self, u, v, x, y, z):
        if self.color_image is not None:
            img = self.color_image.copy()
            cv2.circle(img, (u, v), 6, (0, 0, 255), -1)
            label = "x={:.2f}, y={:.2f}, z={:.2f}".format(x, y, z)
            cv2.putText(img, label, (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Color Image with 3D Point", img)
            cv2.waitKey(1)

class TFPublisher:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.parent_frame = "/amr/camera_color_frame_front"
        self.child_frame = "/detected_point"

    def publish_tf(self, x, y, z):
        self.br.sendTransform(
            (x, y, z),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            self.child_frame,
            self.parent_frame
        )

def main():
    rospy.init_node('pixel_to_3d_tf', anonymous=True)
    pixel_to_3d = PixelTo3D()
    rospy.spin()

if __name__ == '__main__':
    main()
