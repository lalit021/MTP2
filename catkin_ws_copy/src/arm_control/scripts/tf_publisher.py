#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import os
import json

# Define the global frame (world)
GLOBAL_FRAME = "world"


def load_tf_configs(config_files):
    """Loads all TF configuration files (JSON format)."""
    transforms = []
    for config_file in config_files:
        try:
            if os.path.exists(config_file):
                with open(config_file, "r") as file:
                    data = json.load(file)
                transforms.extend(data.get("transforms", []))
            else:
                rospy.logwarn("Config file not found: {}".format(config_file))
        except Exception as e:
            rospy.logerr("Failed to load TF config {}: {}".format(config_file, e))
    return transforms


def extract_transform_from_tf(tf_buffer, parent_frame):
    """Queries the TF buffer to get the full transform (translation + rotation) of parent_frame w.r.t. world."""
    try:
        trans = tf_buffer.lookup_transform(GLOBAL_FRAME, parent_frame, rospy.Time(0), rospy.Duration(1.0))
        translation = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ]
        rotation = trans.transform.rotation  # Quaternion format
        return translation, rotation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("TF lookup failed for {} → {}: {}".format(GLOBAL_FRAME, parent_frame, e))
        return None, None


def apply_rotation_to_child(parent_quaternion, child_rotation):
    """Applies the parent's rotation to the child's rotation."""
    parent_matrix = quaternion_matrix([parent_quaternion.x, parent_quaternion.y, parent_quaternion.z, parent_quaternion.w])[:3, :3]
    child_matrix = quaternion_matrix(quaternion_from_euler(*np.radians(child_rotation)))[:3, :3]

    # Compute final rotation: parent * child
    final_rotation_matrix = np.dot(parent_matrix, child_matrix)

    # Convert back to quaternion
    final_quaternion = quaternion_from_matrix(np.vstack([np.hstack([final_rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    
    return final_quaternion


def publish_tf(tf_broadcaster, parent_frame, child_frame, translation, rotation):
    """Helper function to publish a TF transform."""
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rotation
    tf_broadcaster.sendTransform(t)


def publish_dynamic_tfs(config_files):
    """Publishes dynamic TFs based on given configuration files."""
    rospy.init_node("dynamic_tf_publisher", anonymous=True)
    rospy.loginfo("Starting dynamic TF publisher")

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        transforms = load_tf_configs(config_files)
        for transform in transforms:
            parent_frame = transform["parent_frame"]
            child_frame = transform["child_frame"]
            translation = transform["translation"]
            rotation = transform["rotation"]
            offset_type = transform.get("offset", "local")

            # Check rotation format and convert if needed
            if len(rotation) == 4:
                # If rotation is quaternion (4 elements), use it directly
                child_quaternion = rotation
                rospy.loginfo("Using provided quaternion for rotation: {}".format(rotation))
            elif len(rotation) == 3:
                # If rotation is Euler angles (3 elements), convert to quaternion
                child_quaternion = quaternion_from_euler(*np.radians(rotation))  # Convert Euler to quaternion
                rospy.loginfo("Converted Euler angles to quaternion: {}".format(child_quaternion))
            else:
                rospy.logwarn("Invalid rotation format for transform {} → {}".format(parent_frame, child_frame))
                continue  # Skip this transform if rotation format is invalid

            if offset_type == "global":
                # Query the parent frame's transform (rotation + translation) in the world frame
                parent_translation, parent_quaternion = extract_transform_from_tf(tf_buffer, parent_frame)
                if parent_quaternion is None or parent_translation is None:
                    continue  # Skip if TF lookup failed

                # Apply parent's rotation to child's rotation (global transform)
                final_child_quaternion = apply_rotation_to_child(parent_quaternion, rotation)

                # Compute final child translation (apply parent's translation to child's offset)
                final_translation = [
                    parent_translation[0] + translation[0],
                    parent_translation[1] + translation[1],
                    parent_translation[2] + translation[2]
                ]

                # Publish the final child transform
                publish_tf(tf_broadcaster, GLOBAL_FRAME, child_frame, final_translation, final_child_quaternion)

                rospy.loginfo("Published global transform {} → {}, child inherits parent rotation and translation.".format(
                    GLOBAL_FRAME, child_frame))
            else:
                # Local transformation (apply Euler to quaternion)
                publish_tf(tf_broadcaster, parent_frame, child_frame, translation, child_quaternion)
                rospy.loginfo("Published local transform {} → {}".format(parent_frame, child_frame))

        rate.sleep()

if __name__ == "__main__":
    try:
        config_files = [
            os.path.expanduser("~/catkin_ws/src/arm_control/config/tf_config_static.json"),
            os.path.expanduser("~/catkin_ws/src/arm_control/config/tf_config_dynamic.json"),
            os.path.expanduser("~/catkin_ws/src/arm_control/config/pick_config.json"),
            os.path.expanduser("~/catkin_ws/src/arm_control/config/place_config.json"),
            os.path.expanduser("~/catkin_ws/src/arm_control/config/calib.json"),


            

        ]
        publish_dynamic_tfs(config_files)
    except rospy.ROSInterruptException:
        pass
