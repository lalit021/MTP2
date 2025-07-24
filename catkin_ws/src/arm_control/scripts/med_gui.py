#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf.transformations as tf_trans
import Tkinter as tk
import tkMessageBox as messagebox
import copy
import math

class CobotController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_cobot_gui", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "tmr_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # GUI Setup
        self.root = tk.Tk()
        self.root.title("Cobot Controller with TF Frame Targeting")

        self.mode = tk.StringVar(value="Absolute")
        self.cartesian_mode = tk.BooleanVar(value=True)
        self.current_pose_display = tk.StringVar()

        self.translation_enabled = tk.BooleanVar(value=True)
        self.rotation_enabled = tk.BooleanVar(value=False)

        self.frame_var = tk.StringVar(value="world")
        self.tf_frames = self.get_tf_frames()

        self.setup_gui()
        self.update_robot_state()

    def get_tf_frames(self):
        try:
            frames_str = self.tf_buffer.all_frames_as_string()
            lines = frames_str.splitlines()
            return [line.split()[1] for line in lines if len(line.split()) > 1]
        except:
            return ["world"]

    def setup_gui(self):
        tk.Label(self.root, text="Mode:").grid(row=0, column=0)
        tk.Button(self.root, text="Toggle Mode", command=self.toggle_mode).grid(row=0, column=1)

        tk.Label(self.root, text="X (m):").grid(row=1, column=0)
        self.entry_x = tk.Entry(self.root)
        self.entry_x.grid(row=1, column=1)

        tk.Label(self.root, text="Y (m):").grid(row=2, column=0)
        self.entry_y = tk.Entry(self.root)
        self.entry_y.grid(row=2, column=1)

        tk.Label(self.root, text="Z (m):").grid(row=3, column=0)
        self.entry_z = tk.Entry(self.root)
        self.entry_z.grid(row=3, column=1)

        tk.Button(self.root, text="Plan", command=self.plan_robot).grid(row=4, column=0, columnspan=2)
        tk.Button(self.root, text="Execute", command=self.execute_plan).grid(row=5, column=0, columnspan=2)
        tk.Button(self.root, text="Clear Plan", command=self.clear_plan).grid(row=6, column=0, columnspan=2)
        tk.Button(self.root, text="Toggle Cartesian/Joint", command=self.toggle_cartesian_mode).grid(row=7, column=0, columnspan=2)

        tk.Button(self.root, text="Refresh Frames", command=self.refresh_tf_frames).grid(row=8, column=2)

        tk.Label(self.root, text="Move to TF Frame:").grid(row=8, column=0)

        if not self.tf_frames:
            self.tf_frames = ["world"]
        self.frame_var.set(self.tf_frames[0])  # Ensure a valid default

        self.frame_menu = tk.OptionMenu(self.root, self.frame_var, *self.tf_frames)
        self.frame_menu.grid(row=8, column=1)

        tk.Checkbutton(self.root, text="Use Translation", variable=self.translation_enabled).grid(row=9, column=0)
        tk.Checkbutton(self.root, text="Use Rotation", variable=self.rotation_enabled).grid(row=9, column=1)

        tk.Button(self.root, text="Go to TF Frame", command=self.move_to_tf_frame).grid(row=10, column=0, columnspan=2)

        # State
        tk.Label(self.root, text="Robot State:").grid(row=11, column=0, columnspan=2)
        tk.Label(self.root, textvariable=self.current_pose_display).grid(row=12, column=0, columnspan=2)

    def update_robot_state(self):
        try:
            pose = self.move_group.get_current_pose().pose
            rpy = tf_trans.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            state = "Position: {:.3f}, {:.3f}, {:.3f}\n".format(pose.position.x, pose.position.y, pose.position.z)
            state += "RPY: {:.1f}, {:.1f}, {:.1f}".format(*[math.degrees(a) for a in rpy])
            self.current_pose_display.set(state)
        except:
            self.current_pose_display.set("Error reading state")
        self.root.after(1000, self.update_robot_state)

    def toggle_mode(self):
        if self.mode.get() == "Absolute":
            self.mode.set("Relative")
        else:
            self.mode.set("Absolute")

    def toggle_cartesian_mode(self):
        self.cartesian_mode.set(not self.cartesian_mode.get())

    def plan_robot(self):
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
            else:
                target_pose.position.x += x
                target_pose.position.y += y
                target_pose.position.z += z

            if self.cartesian_mode.get():
                plan, frac = self.move_group.compute_cartesian_path([current_pose, target_pose], 0.01, 0.0)
            else:
                self.move_group.set_pose_target(target_pose)
                plan = self.move_group.plan()
                frac = 1.0

            self.planned_trajectory = plan
            if frac < 1.0:
                messagebox.showwarning("Partial Plan", "Only {:.1f}% planned.".format(frac * 100))
            else:
                messagebox.showinfo("Plan", "Plan succeeded.")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def execute_plan(self):
        try:
            if self.planned_trajectory:
                self.move_group.execute(self.planned_trajectory, wait=True)
            else:
                messagebox.showwarning("No Plan", "No trajectory to execute.")
        except Exception as e:
            messagebox.showerror("Execute Error", str(e))

    def clear_plan(self):
        self.planned_trajectory = None
        messagebox.showinfo("Clear", "Plan cleared.")

    def move_to_tf_frame(self):
        frame = self.frame_var.get()
        try:
            tf = self.tf_buffer.lookup_transform("world", frame, rospy.Time(0), rospy.Duration(1.0))
            target_pose = self.move_group.get_current_pose().pose  # Start from current

            if self.translation_enabled.get():
                target_pose.position.x = tf.transform.translation.x
                target_pose.position.y = tf.transform.translation.y
                target_pose.position.z = tf.transform.translation.z

            if self.rotation_enabled.get():
                q = tf.transform.rotation
                target_pose.orientation.x = q.x
                target_pose.orientation.y = q.y
                target_pose.orientation.z = q.z
                target_pose.orientation.w = q.w

            if self.cartesian_mode.get():
                plan, frac = self.move_group.compute_cartesian_path(
                    [self.move_group.get_current_pose().pose, target_pose], 0.01, 0.0
                )
            else:
                self.move_group.set_pose_target(target_pose)
                plan = self.move_group.plan()
                frac = 1.0

            self.planned_trajectory = plan
            if frac < 1.0:
                messagebox.showwarning("Partial Plan", "Planned {:.1f}%".format(frac * 100))
            else:
                messagebox.showinfo("TF Plan", "Plan to TF frame succeeded.")
        except Exception as e:
            messagebox.showerror("TF Lookup Error", str(e))

    def run(self):
        self.root.mainloop()
    def refresh_tf_frames(self):
        new_frames = self.get_tf_frames()
        if not new_frames:
            new_frames = ["world"]
        self.tf_frames = new_frames

        menu = self.frame_menu["menu"]
        menu.delete(0, "end")
        for frame in self.tf_frames:
            menu.add_command(label=frame, command=lambda value=frame: self.frame_var.set(value))
        self.frame_var.set(self.tf_frames[0])


    def shutdown(self):
        moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    controller = CobotController()
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.shutdown()
