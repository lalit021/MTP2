#!/usr/bin/env python

from __future__ import print_function, division, absolute_import
import sys
import rospy
import moveit_commander
import tf.transformations as tf_trans
import math

class RobotPosePrinter:
    def __init__(self):
        # Initialize MoveIt Commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot_pose_printer", anonymous=True)

        # Initialize MoveGroupCommander for the arm
        group_name = "tmr_arm"  # Change this to match your robot's planning group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def get_robot_pose(self):
        pose = self.move_group.get_current_pose().pose
        position = pose.position
        orientation = pose.orientation

        # Convert quaternion to Roll, Pitch, Yaw
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rpy = tf_trans.euler_from_quaternion(quaternion)

        # Convert radians to degrees
        roll_deg = math.degrees(rpy[0])
        pitch_deg = math.degrees(rpy[1])
        yaw_deg = math.degrees(rpy[2])

        # Print the position and orientation
        print("Position: X={:.3f}, Y={:.3f}, Z={:.3f}".format(position.x, position.y, position.z))
        print("Orientation (Degrees): Roll={:.3f}, Pitch={:.3f}, Yaw={:.3f}".format(roll_deg, pitch_deg, yaw_deg))

    def start_printing(self, frequency=1.0):
        rate = rospy.Rate(frequency)  # Set the rate to the desired frequency
        while not rospy.is_shutdown():
            self.get_robot_pose()
            rate.sleep()


if __name__ == "__main__":
    try:
        printer = RobotPosePrinter()
        printer.start_printing(frequency=1.0)  # Print pose every second
    except rospy.ROSInterruptException:
        pass
