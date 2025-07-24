#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import tf
from tf.transformations import euler_from_quaternion

# Goal Docking Pose
dock_place_pose = {
        "qw": 0.900330448489, 
        "qy": 0.0, 
        "qx": 0.0, 
        "qz": 0.43520694333, 
        "y": 7.36401232207, 
        "x": 5.91434127625, 
        "z": 0.14
}

# Control parameters
LIN_SPEED = 0.15  # Speed for movement
DIST_TOL = 0.05   # Distance tolerance to stop (meters)
KP = 0.2  # Proportional gain for linear movement

class LinearController:
    def __init__(self, kp=KP):
        self.kp = kp

    def compute_linear_velocity(self, current_pose, goal_pose):
        """Compute the linear velocity using proportional control."""
        # Compute distance error
        dx = goal_pose["x"] - current_pose["x"]
        dy = goal_pose["y"] - current_pose["y"]
        distance = math.sqrt(dx**2 + dy**2)

        # Compute linear velocity with proportional control
        linear_velocity = self.kp * distance

        # Ensure the velocity doesn't exceed maximum speed
        linear_velocity = min(linear_velocity, LIN_SPEED)

        return linear_velocity, distance

class DockingController:
    def __init__(self):
        rospy.init_node("docking_controller")
        self.cmd_pub = rospy.Publisher("/amr/cmd_vel", Twist, queue_size=10)
        self.listener = tf.TransformListener()  # TF listener to get robot's position
        self.current_pose = None
        self.linear_controller = LinearController(KP)  # Instantiate linear controller
        self.rate = rospy.Rate(10)

    def get_current_pose(self):
        """Retrieve the robot's current pose using TF listener."""
        try:
            # Wait for the transform from "map" to "amr/base_link" to become available
            self.listener.waitForTransform("map", "amr/base_link", rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = self.listener.lookupTransform("map", "amr/base_link", rospy.Time(0))

            # Extract position (x, y) and orientation (theta) from the transform
            x = trans[0]
            y = trans[1]
            theta = self.get_yaw_from_quaternion(rot)

            return {"x": x, "y": y, "theta": theta}

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Could not get current pose: {}".format(e))
            return None

    def get_yaw_from_quaternion(self, rotation):
        """Converts quaternion rotation to yaw angle."""
        euler = euler_from_quaternion(rotation)
        return euler[2]  # Yaw angle (rotation around Z-axis)

    def compute_cmd_vel(self):
        """Moves forward or backward based on the correct direction to the goal."""
        if self.current_pose is None:
            rospy.logwarn("No odometry data received yet.")
            return

        cmd = Twist()

        # Compute linear velocity using the LinearController class
        linear_velocity, distance = self.linear_controller.compute_linear_velocity(self.current_pose, dock_place_pose)

        # Compute direction angle to goal
        dx = dock_place_pose["x"] - self.current_pose["x"]
        dy = dock_place_pose["y"] - self.current_pose["y"]
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - self.current_pose["theta"]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize angle

        # Print all relevant information in one line
        rospy.loginfo("Current error: dx=%.4f, dy=%.4f, distance=%.4f, angle_diff=%.4f, "
                      "Cmd: %s", dx, dy, distance, angle_diff, 
                      "Moving forward" if distance > DIST_TOL and abs(angle_diff) < math.pi / 2 else 
                      ("Moving backward" if distance > DIST_TOL else "Stopping"))

        # Move forward or backward if far from goal
        if distance > DIST_TOL:
            if abs(angle_diff) < math.pi / 2:
                cmd.linear.x = linear_velocity
            else:
                cmd.linear.x = -linear_velocity
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)
        return distance  # Return the distance for termination check

    def run(self):
        """Main loop to adjust the robot's position until it reaches the docking pose."""
        while not rospy.is_shutdown():
            self.current_pose = self.get_current_pose()  # Get current pose using TF
            if self.current_pose:
                distance = self.compute_cmd_vel()  # Get the distance after moving
                if distance <= DIST_TOL:  # Stop if robot is within the tolerance
                    rospy.loginfo("Goal reached, stopping the robot.")
                    break  # Terminate after stopping
            self.rate.sleep()

if __name__ == "__main__":
    controller = DockingController()
    controller.run()
