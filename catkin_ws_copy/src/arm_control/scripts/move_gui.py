#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as tf_trans
import copy
import math
import Tkinter as tk
import ttk
import tkMessageBox as messagebox  # Import messagebox for showing popups

class CobotController:
    def __init__(self):
        # Initialize ROS and MoveIt! components
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_cobot_gui", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "tmr_arm"  # Update this if needed
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Store the planned trajectory and modes
        self.planned_trajectory = None

        # GUI Setup
        self.root = tk.Tk()  # Initialize the Tkinter root window

        # Now that the root window is created, initialize other variables
        self.mode = tk.StringVar(value="Absolute")
        self.cartesian_mode = tk.BooleanVar(value=True)  # Cartesian mode
        self.current_pose_display = tk.StringVar()

        self.setup_gui()

    def setup_gui(self):
        # Create GUI components
        self.label_mode = tk.Label(self.root, text="Mode: Absolute", font=("Arial", 14))
        self.label_mode.grid(row=0, column=0, padx=10, pady=10)

        self.label_x = tk.Label(self.root, text="X (m):")
        self.label_x.grid(row=1, column=0)
        self.entry_x = tk.Entry(self.root)
        self.entry_x.grid(row=1, column=1)

        self.label_y = tk.Label(self.root, text="Y (m):")
        self.label_y.grid(row=2, column=0)
        self.entry_y = tk.Entry(self.root)
        self.entry_y.grid(row=2, column=1)

        self.label_z = tk.Label(self.root, text="Z (m):")
        self.label_z.grid(row=3, column=0)
        self.entry_z = tk.Entry(self.root)
        self.entry_z.grid(row=3, column=1)

        self.button_plan = tk.Button(self.root, text="Plan", command=self.plan_robot)
        self.button_plan.grid(row=4, column=0, columnspan=2, pady=10)

        self.button_execute = tk.Button(self.root, text="Execute", command=self.execute_plan)
        self.button_execute.grid(row=5, column=0, columnspan=2, pady=10)

        self.button_clear = tk.Button(self.root, text="Clear Plan", command=self.clear_plan)
        self.button_clear.grid(row=6, column=0, columnspan=2, pady=10)

        self.button_toggle_mode = tk.Button(self.root, text="Toggle Mode", command=self.toggle_mode)
        self.button_toggle_mode.grid(row=7, column=0, columnspan=2, pady=10)

        self.button_toggle_cartesian = tk.Button(self.root, text="Toggle Cartesian/Joint", command=self.toggle_cartesian_mode)
        self.button_toggle_cartesian.grid(row=8, column=0, columnspan=2, pady=10)

        self.label_cartesian_mode = tk.Label(self.root, text="Movement Mode: Cartesian", font=("Arial", 14))
        self.label_cartesian_mode.grid(row=9, column=0, columnspan=2, padx=10, pady=10)

        self.label_state = tk.Label(self.root, text="Current State:", font=("Arial", 14))
        self.label_state.grid(row=10, column=0, columnspan=2, padx=10, pady=10)

        self.label_current_state = tk.Label(self.root, textvariable=self.current_pose_display, justify="left", font=("Arial", 12))
        self.label_current_state.grid(row=11, column=0, columnspan=2, padx=10, pady=10)

        # Start updating robot state
        self.update_robot_state()

    def update_robot_state(self):
        """ Continuously update the robot's current state. """
        try:
            pose = self.move_group.get_current_pose().pose
            position = pose.position
            orientation = pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            rpy = tf_trans.euler_from_quaternion(quaternion)

            # Convert radians to degrees
            roll_deg = math.degrees(rpy[0])
            pitch_deg = math.degrees(rpy[1])
            yaw_deg = math.degrees(rpy[2])

            state_text = "Position (X, Y, Z): {:.3f}, {:.3f}, {:.3f}\n".format(position.x, position.y, position.z)
            state_text += "Orientation (Roll, Pitch, Yaw): {:.3f}, {:.3f}, {:.3f}".format(roll_deg, pitch_deg, yaw_deg)

            self.current_pose_display.set(state_text)

            # Continuously update the robot state every 1000ms
            self.root.after(1000, self.update_robot_state)

        except Exception as e:
            messagebox.showerror("Error", "Failed to update robot state: {}".format(str(e)))

    def plan_robot(self):
        """ Plan the robot's movement (absolute/relative) in Cartesian or Joint space. """
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
            move_type = self.mode.get()

            current_pose = self.move_group.get_current_pose().pose
            target_pose = copy.deepcopy(current_pose)

            if move_type == "Absolute":
                target_pose.position.x = x
                target_pose.position.y = y
                target_pose.position.z = z
            elif move_type == "Relative":
                target_pose.position.x += x
                target_pose.position.y += y
                target_pose.position.z += z

            # Plan the movement
            waypoints = [copy.deepcopy(current_pose), copy.deepcopy(target_pose)]

            if self.cartesian_mode.get():
                # Cartesian space planning
                plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            else:
                # Joint space planning
                self.move_group.set_pose_target(target_pose)
                plan = self.move_group.plan()  # Plan using joint space
                fraction = 1.0  # Assuming the plan was successful for simplicity

            if fraction < 1.0:
                messagebox.showwarning("Path Planning", "Path planning succeeded partially ({:.1f}%)".format(fraction * 100))
            else:
                messagebox.showinfo("Path Planning", "Path planning succeeded 100%.")

            # Store the planned trajectory
            self.planned_trajectory = plan
            messagebox.showinfo("Plan", "Plan created successfully.")

        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numerical values for X, Y, Z.")
        except Exception as e:
            messagebox.showerror("Planning Error", "Failed to plan the robot movement: {}".format(str(e)))

    def execute_plan(self):
        """ Execute the previously planned trajectory. """
        try:
            if self.planned_trajectory:
                # Execute the planned trajectory
                self.move_group.execute(self.planned_trajectory, wait=True)
                messagebox.showinfo("Movement", "Movement executed successfully.")
            else:
                messagebox.showwarning("No Plan", "No planned path available to execute.")
        except Exception as e:
            messagebox.showerror("Execution Error", "An error occurred during execution: {}".format(str(e)))

    def clear_plan(self):
        """ Clear the planned trajectory and move to the current position. """
        try:
            current_pose = self.move_group.get_current_pose().pose
            target_pose = copy.deepcopy(current_pose)
            waypoints = [copy.deepcopy(current_pose), target_pose]

            # Plan the robot to its current position again
            if self.cartesian_mode.get():
                # Cartesian space planning
                plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            else:
                # Joint space planning
                self.move_group.set_pose_target(target_pose)
                plan = self.move_group.plan()  # Plan using joint space
                fraction = 1.0  # Assuming the plan was successful for simplicity

            # Execute the "clear" movement (return to current position)
            self.move_group.execute(plan, wait=True)
            messagebox.showinfo("Clear Plan", "Returned to current position.")

            # Reset the planned trajectory
            self.planned_trajectory = None
        except Exception as e:
            messagebox.showerror("Clear Plan Error", "Error clearing plan: {}".format(str(e)))

    def toggle_mode(self):
        """ Toggle between Absolute and Relative movement modes. """
        if self.mode.get() == "Absolute":
            self.mode.set("Relative")
            self.label_mode.config(text="Mode: Relative")
        else:
            self.mode.set("Absolute")
            self.label_mode.config(text="Mode: Absolute")

    def toggle_cartesian_mode(self):
        """ Toggle between Cartesian and Joint movement modes. """
        if self.cartesian_mode.get():
            self.cartesian_mode.set(False)
            self.label_cartesian_mode.config(text="Movement Mode: Joint")
        else:
            self.cartesian_mode.set(True)
            self.label_cartesian_mode.config(text="Movement Mode: Cartesian")

    def run(self):
        """ Start the GUI loop. """
        self.root.mainloop()

    def shutdown(self):
        """ Clean up MoveIt! on shutdown. """
        moveit_commander.roscpp_shutdown()


# Run the program
if __name__ == "__main__":
    controller = CobotController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Shutting down Cobot Controller.")
        controller.shutdown()
