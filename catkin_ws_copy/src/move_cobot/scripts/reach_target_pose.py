#!/usr/bin/env python

from __future__ import print_function, division, absolute_import
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tf_trans
import copy
import math
from geometry_msgs.msg import Pose

class DisplayPickPoints:
    def __init__(self, approach_distance=0.1, lift_distance=0.1):
        self.approach_distance = approach_distance
        self.lift_distance = lift_distance

    def calculate_offset_pose(self, base_pose, offset, offset_local=True):
        """
        Calculate an offset pose from the given base pose.
        :param base_pose: The original pose.
        :param offset: The distance to offset.
        :param offset_local: Whether the offset is in the local frame.
        :return: Offset pose.
        """
        offset_pose = copy.deepcopy(base_pose)
        try:
            if offset_local:
                quaternion = [
                    base_pose.orientation.x,
                    base_pose.orientation.y,
                    base_pose.orientation.z,
                    base_pose.orientation.w,
                ]
                rotation_matrix = tf_trans.quaternion_matrix(quaternion)
                offset_vector = [0, 0, offset, 1]
                offset_transformed = rotation_matrix.dot(offset_vector)
                offset_pose.position.x += offset_transformed[0]
                offset_pose.position.y += offset_transformed[1]
                offset_pose.position.z += offset_transformed[2]
            else:
                offset_pose.position.z += offset
        except Exception as e:
            rospy.logerr("Error calculating offset pose: {}".format(e))
        return offset_pose

    def get_pick_sequence(self, target_pose):
        """
        Generate approach, grasp, and lift poses based on the target pose.
        :param target_pose: The target pose.
        :return: Tuple of approach, grasp, and lift poses.
        """
        try:
            approach_pose = self.calculate_offset_pose(target_pose, -self.approach_distance, offset_local=True)
            lift_pose = self.calculate_offset_pose(target_pose, self.lift_distance, offset_local=False)
            return approach_pose, target_pose, lift_pose
        except Exception as e:
            rospy.logerr("Error generating pick sequence: {}".format(e))
            return None, None, None


class TFVisualizer:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def broadcast_tf(self, pose, frame_id, child_frame_id):
        try:
            rospy.loginfo("Broadcasting TF: {} relative to {}".format(child_frame_id, frame_id))

            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = frame_id
            transform.child_frame_id = child_frame_id
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation.x = pose.orientation.x
            transform.transform.rotation.y = pose.orientation.y
            transform.transform.rotation.z = pose.orientation.z
            transform.transform.rotation.w = pose.orientation.w

            self.tf_broadcaster.sendTransform(transform)
            rospy.loginfo("Broadcasted TF: {}".format(child_frame_id))
        except Exception as e:
            rospy.logerr("Error broadcasting TF: {}".format(e))


def get_target_pose():
    """
    Get the target pose either from user input or by listening to a target TF frame.
    :return: Target pose.
    """
    try:
        mode = input("Enter '1' for manual input or '2' to listen to a target TF frame: ")
        mode = str(mode).strip()  # Ensure it's treated as a string and stripped of whitespace

        if mode == '1':
            # Manual input
            x = float(input("Enter x position (in meters): "))
            y = float(input("Enter y position (in meters): "))
            z = float(input("Enter z position (in meters): "))

            roll_deg = float(input("Enter roll angle in degrees: "))
            pitch_deg = float(input("Enter pitch angle in degrees: "))
            yaw_deg = float(input("Enter yaw angle in degrees: "))

            roll_rad = math.radians(roll_deg)
            pitch_rad = math.radians(pitch_deg)
            yaw_rad = math.radians(yaw_deg)

            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z

            quaternion = tf_trans.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
            target_pose.orientation.x = quaternion[0]
            target_pose.orientation.y = quaternion[1]
            target_pose.orientation.z = quaternion[2]
            target_pose.orientation.w = quaternion[3]

            return target_pose

        elif mode == '2':
            # Listen to a target TF frame
            rospy.loginfo("Listening for a target TF frame...")
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            target_frame = "target_frame"#input("Enter the name of the target TF frame to listen to: ").strip()

            while not rospy.is_shutdown():
                try:
                    transform = tf_buffer.lookup_transform("world", target_frame, rospy.Time(0), rospy.Duration(1.0))
                    pose = Pose()
                    pose.position.x = transform.transform.translation.x
                    pose.position.y = transform.transform.translation.y
                    pose.position.z = transform.transform.translation.z
                    pose.orientation.x = transform.transform.rotation.x
                    pose.orientation.y = transform.transform.rotation.y
                    pose.orientation.z = transform.transform.rotation.z
                    pose.orientation.w = transform.transform.rotation.w
                    return pose
                except Exception as e:
                    rospy.logwarn("Waiting for target frame '{}'. Error: {}".format(target_frame, e))
        else:
            rospy.logerr("Invalid mode selected. Exiting.")
            return None

    except Exception as e:
        rospy.logerr("Error in get_target_pose: {}".format(e))

        return None

def main():
    rospy.init_node("pick_and_place_tf_broadcaster", anonymous=True)

    display_points = DisplayPickPoints(approach_distance=0.15, lift_distance=0.1)
    tf_visualizer = TFVisualizer()

    rospy.loginfo("Waiting for user input to define the target pose...")
    target_pose = get_target_pose()

    approach_pose, grasp_pose, lift_pose = display_points.get_pick_sequence(target_pose)
    if not (approach_pose and grasp_pose and lift_pose):
        rospy.logerr("Failed to calculate pick sequence. Exiting.")
        return

    rate = rospy.Rate(10)  # 10 Hz, adjust as necessary

    while not rospy.is_shutdown():
        tf_visualizer.broadcast_tf(approach_pose, "world", "approach")
        tf_visualizer.broadcast_tf(grasp_pose, "world", "grasp")
        tf_visualizer.broadcast_tf(lift_pose, "world", "lift")

        rate.sleep()

    rospy.loginfo("TF frames for approach, grasp, and lift published continuously.")


if __name__ == "__main__":
    main()
