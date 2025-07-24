#!/usr/bin/env python

from __future__ import print_function, division, absolute_import
import rospy
import tf2_ros
import geometry_msgs.msg
import moveit_commander
import tf.transformations as tf_trans
import copy
import math
import sys
from geometry_msgs.msg import PoseStamped



class CollisionObjectManager:
    def __init__(self, scene):
        self.scene = scene
        self.objects = set()

    def add_flat_ground(self, size=(2.0, 2.0, 0.01), z_offset=-0.005):
        """
        Add a flat ground surface to the planning scene.
        :param size: Tuple of the size of the ground surface (x, y, z).
        :param z_offset: The z-coordinate offset for the ground plane.
        """
        ground_pose = geometry_msgs.msg.PoseStamped()
        ground_pose.header.frame_id = "world"
        ground_pose.pose.orientation.w = 1.0
        ground_pose.pose.position.x = 0
        ground_pose.pose.position.y = 0
        ground_pose.pose.position.z = z_offset
        self.scene.add_box("ground", ground_pose, size=size)
        self.objects.add("ground")
        rospy.loginfo("Added ground surface to the planning scene.")

    def add_box(self, name, pose, size):
        """
        Add a box to the planning scene.
        :param name: Name of the object.
        :param pose: geometry_msgs.msg.PoseStamped defining the object's pose.
        :param size: Tuple of the size of the box (x, y, z).
        """
        self.scene.add_box(name, pose, size)
        self.objects.add(name)
        rospy.loginfo("Added box '{}' to the planning scene.".format(name))

    def remove_object(self, name):
        """
        Remove an object from the planning scene.
        :param name: Name of the object to remove.
        """
        self.scene.remove_world_object(name)
        self.objects.discard(name)
        rospy.loginfo("Removed object '{}' from the planning scene.".format(name))

    def clear_scene(self):
        """
        Remove all objects from the planning scene.
        """
        for obj in list(self.objects):
            self.remove_object(obj)
        rospy.loginfo("Cleared all objects from the planning scene.")


def get_target_pose():
    # Get the target pose from user input
    x = float(raw_input("Enter x position (in meters): "))  # For Python 2
    y = float(raw_input("Enter y position (in meters): "))  # For Python 2
    z = float(raw_input("Enter z position (in meters): "))  # For Python 2
    
    roll_deg = float(raw_input("Enter roll angle in degrees: "))  # For Python 2
    pitch_deg = float(raw_input("Enter pitch angle in degrees: "))  # For Python 2
    yaw_deg = float(raw_input("Enter yaw angle in degrees: "))  # For Python 2
    
    # Convert angles from degrees to radians
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    # Create the target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    
    # Convert Euler angles (roll, pitch, yaw) to a quaternion
    quaternion = tf_trans.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    
    return target_pose


def main():
    # Initialize MoveIt and ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("integrated_pick_and_collision_demo", anonymous=True)

    # Initialize MoveGroupCommander and PlanningSceneInterface
    move_group = moveit_commander.MoveGroupCommander("tmr_arm")
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate the CollisionObjectManager
    collision_manager = CollisionObjectManager(scene)

    # Add a flat ground surface and a box to the planning scene
    rospy.sleep(2)  # Wait for the planning scene to initialize
    collision_manager.add_flat_ground()
    # box_pose = PoseStamped()
    # box_pose.header.frame_id = "world"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.x = 0.5
    # box_pose.pose.position.y = 0.0
    # box_pose.pose.position.z = 0.1
    # collision_manager.add_box("example_box", box_pose, size=(0.2, 0.2, 0.2))


    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()

