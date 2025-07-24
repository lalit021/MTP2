#!/usr/bin/env python

from __future__ import print_function, division, absolute_import
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
import tf.transformations as tf_trans
import copy
import math


class MoveCobot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_cobot_control", anonymous=True)

        # Initialize the RobotCommander and PlanningSceneInterface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialize MoveGroupCommander for the arm
        group_name = "tmr_arm"  # Change this for your robot's planning group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set up a publisher to visualize planned paths in RViz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

    def get_robot_pose(self):
        pose = self.move_group.get_current_pose().pose
        position = pose.position
        orientation = pose.orientation

        # Convert quaternion to RPY (Roll, Pitch, Yaw)
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rpy = tf_trans.euler_from_quaternion(quaternion)

        # Convert radians to degrees
        roll_deg = math.degrees(rpy[0])
        pitch_deg = math.degrees(rpy[1])
        yaw_deg = math.degrees(rpy[2])

        print("Current Pose:")
        print("Position: X={:.3f}, Y={:.3f}, Z={:.3f}".format(position.x, position.y, position.z))
        print("Orientation (Radians): Roll={:.3f}, Pitch={:.3f}, Yaw={:.3f}".format(rpy[0], rpy[1], rpy[2]))
        print("Orientation (Degrees): Roll={:.3f}, Pitch={:.3f}, Yaw={:.3f}".format(roll_deg, pitch_deg, yaw_deg))

        return pose
    
    def move_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        # Plan the Cartesian path
        plan, fraction = self.move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
        return plan, fraction

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def display_trajectory(self, plan):
        # Publish the trajectory for visualization in RViz
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def set_orientation(self, roll, pitch, yaw):
        # Convert Roll, Pitch, Yaw to Quaternion
        quaternion = tf_trans.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
        return quaternion


def parse_command(command):
    # Ensure command is a string and split by commas
    if isinstance(command, str):
        cmd_parts = command.split(',')
    else:
        print("Invalid input format. Expected a string.")
        return None

    # Extract the X, Y, Z values and convert them to floats
    try:
        x = float(cmd_parts[0].strip())
        y = float(cmd_parts[1].strip())
        z = float(cmd_parts[2].strip())
    except ValueError:
        print("Invalid input. Please provide values in the format X,Y,Z.")
        return None

    return x, y, z


def parse_orientation_command(command):
    # Ensure command is a string and split by commas
    if isinstance(command, str):
        cmd_parts = command.split(',')
    else:
        print("Invalid input format. Expected a string.")
        return None

    # Extract the Roll, Pitch, Yaw values and convert them to floats
    try:
        roll = float(cmd_parts[0].strip())
        pitch = float(cmd_parts[1].strip())
        yaw = float(cmd_parts[2].strip())
    except ValueError:
        print("Invalid input. Please provide values in the format Roll,Pitch,Yaw.")
        return None

    return roll, pitch, yaw


def main():
    cobot = MoveCobot()

    # Example usage
    while True:
        # Display the current pose
        current_pose = cobot.get_robot_pose()

        # Ask user for input (Position or Orientation mode)
        print("\nEnter 'p' to move by position (X,Y,Z) or 'o' to move by orientation (Roll,Pitch,Yaw), or 'exit' to quit:")
        mode = input() if sys.version_info[0] >= 3 else raw_input()

        if mode.lower() == 'exit':
            print("Exiting.")
            break

        if mode.lower() == 'p':
            print("\nEnter the movement increment as X,Y,Z (e.g., 0.1,0.2,0):")
            command = input() if sys.version_info[0] >= 3 else raw_input()

            # Parse command
            params = parse_command(command)
            if params is None:
                continue

            # Display planned movement
            x, y, z = params
            target_pose = copy.deepcopy(current_pose)
            target_pose.position.x += x
            target_pose.position.y += y
            target_pose.position.z += z

            print("\nPlanned Movement:")
            print("Move to Position: X={:.3f}, Y={:.3f}, Z={:.3f}".format(
                target_pose.position.x, target_pose.position.y, target_pose.position.z))

            # Create waypoints for Cartesian path
            print("\nCalculating the Cartesian Path...")
            waypoints = [copy.deepcopy(current_pose), copy.deepcopy(target_pose)]

            # Plan the Cartesian path
            plan, fraction = cobot.move_cartesian_path(waypoints)

            if fraction < 1.0:
                print("Warning: The path planning succeeded only partially ({}%).".format(fraction * 100))
            else:
                print("Path planning succeeded 100%.")

            # Display the trajectory in RViz (without executing it yet)
            cobot.display_trajectory(plan)

            # Ask for confirmation to execute the plan
            confirm = input("Execute this movement? (y/n): ").lower() if sys.version_info[0] >= 3 else raw_input("Execute this movement? (y/n): ").lower()
            if confirm == 'y':
                cobot.execute_plan(plan)
                print("Movement executed.")
            else:
                print("Movement canceled.")

        elif mode.lower() == 'o':
            print("\nEnter the desired orientation as Roll,Pitch,Yaw (e.g., 45,30,60):")
            command = input() if sys.version_info[0] >= 3 else raw_input()

            # Parse orientation command
            params = parse_orientation_command(command)
            if params is None:
                continue

            # Set new orientation
            roll, pitch, yaw = params
            quaternion = cobot.set_orientation(roll, pitch, yaw)
            current_pose.orientation.x = quaternion[0]
            current_pose.orientation.y = quaternion[1]
            current_pose.orientation.z = quaternion[2]
            current_pose.orientation.w = quaternion[3]

            print("\nPlanned Orientation:")
            print("Roll={:.3f}, Pitch={:.3f}, Yaw={:.3f}".format(roll, pitch, yaw))

            # Create waypoints for Cartesian path (without changing position)
            print("\nCalculating the Cartesian Path...")
            waypoints = [copy.deepcopy(current_pose)]

            # Plan the Cartesian path
            plan, fraction = cobot.move_cartesian_path(waypoints)

            if fraction < 1.0:
                print("Warning: The path planning succeeded only partially ({}%).".format(fraction * 100))
            else:
                print("Path planning succeeded 100%.")

            # Display the trajectory in RViz (without executing it yet)
            cobot.display_trajectory(plan)

            # Ask for confirmation to execute the plan
            confirm = input("Execute this movement? (y/n): ").lower() if sys.version_info[0] >= 3 else raw_input("Execute this movement? (y/n): ").lower()
            if confirm == 'y':
                cobot.execute_plan(plan)
                print("Movement executed.")
            else:
                print("Movement canceled.")

        else:
            print("Invalid mode. Please enter 'position', 'orientation', or 'exit'.")


if __name__ == "__main__":
    main()
