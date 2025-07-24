#!/bin/bash
#roslaunch tm5-700-moveit_config tm5-700_moveit_planning_execution.launch sim:=False robot_ip:=192.168.98.99
roslaunch tm5-700-moveit_config tm5-700_moveit_planning_execution.launch sim:=True

roslaunch gripper_with_robot_moveit_config moveit_planning_execution.launch sim:=False robot_ip:=192.168.98.99

roslaunch tm5-700-moveit_config tm5-700_moveit_planning_execution.launch sim:=False robot_ip:=192.168.98.99
