#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
import json
import cv2
import numpy as np
import rospkg
import os
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix

def load_json(file_path, default={}):
    """Loads a JSON file."""
    try:
        with open(file_path, "r") as file:
            return json.load(file)
    except Exception as e:
        rospy.logerr("Failed to load {}: {}".format(file_path, e))
        return default

class ArucoSubscriber:
    def __init__(self):
        # Set file paths for the config and camera parameters
        self.config_file = "/root/catkin_ws/src/aruco_pose/config/aruco_mapping.json"
        self.camera_params_file = "/root/catkin_ws/src/aruco_pose/config/camera_params.json"

        # ROS Setup
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        self.image_pub = rospy.Publisher("/camera/aruco/image_raw", Image, queue_size=10)
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize last received time for heartbeat check
        self.last_received_time = rospy.Time.now()

        # Set a timer for heartbeat check (every 1 second)
        rospy.Timer(rospy.Duration(1), self.heartbeat_callback)

        # Load Parameters
        self.marker_mapping = load_json(self.config_file, {})
        camera_params = load_json(self.camera_params_file, {"camera_matrix": [], "dist_coeffs": []})

        # Camera Parameters Validation
        self.camera_matrix = np.array(camera_params.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(camera_params.get("dist_coeffs", []), dtype=np.float32)

        if self.camera_matrix.size == 0 or self.dist_coeffs.size == 0:
            rospy.logwarn("Camera parameters not properly loaded! Using default parameters.")

        # ArUco Parameters
        self.dictionary_type = cv2.aruco.DICT_6X6_250
        self.marker_size = 0.05  # Meters

    def image_callback(self, msg):
        try:
            # Update the last received time whenever a new image is received
            self.last_received_time = rospy.Time.now()

            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            dictionary = cv2.aruco.getPredefinedDictionary(self.dictionary_type)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            if ids is not None:
                rospy.loginfo("Detected ArUco markers: {}".format(ids.flatten().tolist()))

                rvecs, tvecs = self.estimate_pose(corners)

                for i, marker_id in enumerate(ids.flatten()):
                    reverse = self.marker_mapping.get("marker_{}".format(marker_id), {}).get("reverse", False)
                    self.publish_marker_tf(marker_id, rvecs[i], tvecs[i], reverse)

                    # Draw marker ID on the image
                    top_left = corners[i][0][0]
                    cv2.putText(cv_image, "ID: {}".format(marker_id), (int(top_left[0]), int(top_left[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            else:
                rospy.logwarn("No ArUco markers detected.")

            # Publish annotated image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            rospy.logerr("Error in image callback: {}".format(e))

    def heartbeat_callback(self, event):
        """Heartbeat callback to check if camera input is being received."""
        current_time = rospy.Time.now()
        time_diff = (current_time - self.last_received_time).to_sec()

        # If no image received for more than 5 seconds, log a warning
        if time_diff > 5:
            rospy.logwarn("No camera input received for more than 5 seconds.")

    def estimate_pose(self, corners):
        """Estimates pose of detected ArUco markers."""
        marker_points = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                                  [self.marker_size / 2, self.marker_size / 2, 0],
                                  [self.marker_size / 2, -self.marker_size / 2, 0],
                                  [-self.marker_size / 2, -self.marker_size /2, 0]], dtype=np.float32)

        rvecs, tvecs = [], []
        for corner in corners:
            try:
                corner = np.array(corner, dtype=np.float32).reshape(-1, 1, 2)
                _, rvec, tvec = cv2.solvePnP(marker_points, corner, self.camera_matrix, self.dist_coeffs)
                rvecs.append(rvec)
                tvecs.append(tvec)
            except cv2.error as e:
                rospy.logerr("Error in solvePnP: {}".format(e))
                continue  # Skip the marker if pose estimation fails

        return rvecs, tvecs

    def publish_marker_tf(self, marker_id, rvec, tvec, reverse=False):
        """Publishes transform, with an optional reversal."""
        object_name = self.marker_mapping.get("marker_{}".format(marker_id), {}).get("name", "marker_{}".format(marker_id))

        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = quaternion_from_matrix(
            np.vstack([np.column_stack([rotation_matrix, [0, 0, 0]]), [0, 0, 0, 1]])
        )

        # Create TransformStamped
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()

        if reverse:
            # Reverse transformation: Swap frame names & invert translation
            t.header.frame_id, t.child_frame_id = object_name, "camera_frame"
            inverse_rotation_matrix = np.linalg.inv(rotation_matrix)
            inverse_translation = -np.dot(inverse_rotation_matrix, np.array([tvec[0][0], tvec[1][0], tvec[2][0]]))

            quaternion = quaternion_from_matrix(
                np.vstack([np.column_stack([inverse_rotation_matrix, [0, 0, 0]]), [0, 0, 0, 1]])
            )

            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = inverse_translation
        else:
            # Normal TF: camera_frame -> marker
            t.header.frame_id, t.child_frame_id = "camera_frame", object_name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = tvec[0][0], tvec[1][0], tvec[2][0]

        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quaternion

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node('aruco_detect', anonymous=True)

    # Get the package path dynamically
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('aruco_pose')  # Change 'aruco_pose' to your actual package name

    # Construct the full paths
    config_file = os.path.join(package_path, "config", "aruco_mapping.json")
    camera_params_file = os.path.join(package_path, "config", "camera_params.json")

    # Load JSON files
    marker_mapping = load_json(config_file)
    camera_params = load_json(camera_params_file)

    rospy.loginfo("Loaded config file: {}".format(config_file))
    rospy.loginfo("Loaded camera parameters file: {}".format(camera_params_file))

    # Start processing
    aruco_subscriber = ArucoSubscriber()

    rospy.spin()
