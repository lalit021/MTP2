#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
import tf2_ros
import tf.transformations as tf_trans
import copy
import math

class PathPlanner:
    def __init__(self, group_name="tmr_arm"):
        rospy.init_node("path_planner", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander(group_name)
        self.display_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=10)

    def plan_to_pose(self, target_pose, cartesian=False):
        if cartesian:
            waypoints = []
            start_pose = self.group.get_current_pose().pose
            waypoints.append(start_pose)
            waypoints.append(target_pose)
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=10)
            return plan if fraction > 0 else None
        else:
            self.group.set_pose_target(target_pose)
            plan = self.group.plan()
            self.group.clear_pose_targets()
            return plan

    def display_path(self, plan):
        if plan:
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.display_publisher.publish(display_trajectory)

    def execute_path(self, plan):
        if plan:
            rospy.loginfo("Executing the planned path...")
            success = self.group.execute(plan, wait=True)
            if success:
                rospy.loginfo("Path execution succeeded.")
            else:
                rospy.logerr("Path execution failed.")

    def confirm_execution(self):
        user_input = raw_input("Do you want to execute the planned path? (yes/no): ").strip().lower()
        return user_input == "yes"

def listen_to_frames():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # frame_list = ["approach", "target_pick", "lift"]  # Example frames to listen to,
    frame_list = ["approach", "lift", "target_place","target_pick"]  # Example frames to listen to

    rate = rospy.Rate(1)  # 1 Hz loop rate

    while not rospy.is_shutdown():
        for frame in frame_list:
            try:
                transform = tf_buffer.lookup_transform("world", frame, rospy.Time(0))
                target_pose = Pose()
                target_pose.position.x = transform.transform.translation.x
                target_pose.position.y = transform.transform.translation.y
                target_pose.position.z = transform.transform.translation.z
                target_pose.orientation.x = transform.transform.rotation.x
                target_pose.orientation.y = transform.transform.rotation.y
                target_pose.orientation.z = transform.transform.rotation.z
                target_pose.orientation.w = transform.transform.rotation.w
                yield frame, target_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Frame {} not available.".format(frame))
        rate.sleep()

def main():
    planner = PathPlanner()

    for frame, target_pose in listen_to_frames():
        rospy.loginfo("Planning path to frame: {}".format(frame))

        # Hardcode Cartesian path planning decision:
        # For "approach" frame, set cartesian = False (non-Cartesian)
        # For others, set cartesian = True
        is_cartesian = False if frame == "approach" else True
        
        # Plan the path based on the cartesian value
        user_input = raw_input("Do you want to plan the path? (yes/no): ").strip().lower()
        if user_input == "yes":
            plan = planner.plan_to_pose(target_pose, cartesian=is_cartesian)

            if plan:
                rospy.loginfo("Path planned successfully for frame: {}".format(frame))
                planner.display_path(plan)

                if planner.confirm_execution():
                    planner.execute_path(plan)
                    rospy.loginfo("Path executed successfully for frame: {}".format(frame))
                else:
                    rospy.loginfo("Execution skipped.")
            else:
                rospy.logerr("Failed to plan path for frame: {}".format(frame))

if __name__ == "__main__":
    main()
