#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    # 1) Initialize moveit_commander & ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('lalit_move', anonymous=True)

    # 2) Instantiate a RobotCommander (interface to URDF + SRDF) & MoveGroupCommander
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("tmr_arm")

    # 3) Allow a moment for Rviz/servers to catch up
    rospy.sleep(1.0)

    # 4) (Optional) Print out basic info
    rospy.loginfo("Planning frame: %s", group.get_planning_frame())
    rospy.loginfo("End effector link: %s", group.get_end_effector_link())
    rospy.loginfo("Available groups: %s", robot.get_group_names())

    # 5) Move to a named target called "home" (make sure you have this defined)
    group.set_named_target("home")
    plan1 = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.loginfo("Moved to 'home': %s", plan1)

    # 6) Define a simple joint‐space goal
    current_joints = group.get_current_joint_values()
    target_joints = list(current_joints)
    # e.g. rotate joint-1 by +0.5 rad
    target_joints[0] += 0.5

    group.set_joint_value_target(target_joints)
    plan2 = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.loginfo("Executed offset joint goal: %s", plan2)

    # 7) Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
