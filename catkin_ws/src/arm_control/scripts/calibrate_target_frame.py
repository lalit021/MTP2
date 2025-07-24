#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import math
import os
import json
import time
import Tkinter as tk
import ttk
import tkMessageBox


def quaternion_to_euler(x, y, z, w):
    """ Converts a quaternion to Euler angles (roll, pitch, yaw). """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def save_to_file(data, file_name="calibration_config.json"):
    """ Saves calibration data to a JSON file. """
    if os.path.exists(file_name):
        with open(file_name, 'r') as file:
            try:
                existing_data = json.load(file)
            except ValueError:
                existing_data = {}
    else:
        existing_data = {}

    existing_data.update(data)

    with open(file_name, 'w') as file:
        json.dump(existing_data, file, indent=4)

    rospy.loginfo("Saved calibration data to {}".format(file_name))


def perform_calibration(parent_frame, child_frame, robot_eff):
    """ Fetches TF transform and saves calibration data. """
    rospy.init_node('calibration_gui', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    timeout = 20  # Maximum wait time in seconds
    start_time = time.time()

    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform(parent_frame, robot_eff, rospy.Time(0))

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            roll, pitch, yaw = quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)

            target_frame_transformation = {
                "parent_frame": parent_frame,
                "child_frame": child_frame,
                "translation": [translation.x, translation.y, translation.z],
                "rotation": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
            }

            rospy.loginfo("Calibration Result: {}".format(target_frame_transformation))

            save_to_file({child_frame: target_frame_transformation})

            tkMessageBox.showinfo("Success", "Calibration saved for {}".format(child_frame))
            return
        except tf2_ros.LookupException:
            rospy.logwarn("Transform not found. Retrying...")
        except tf2_ros.ConnectivityException:
            rospy.logwarn("Frames are not connected. Retrying...")
        except tf2_ros.ExtrapolationException:
            rospy.logwarn("Transform extrapolation error. Retrying...")

        if time.time() - start_time > timeout:
            rospy.logerr("Timeout reached: Could not find transform within {} seconds.".format(timeout))
            tkMessageBox.showerror("Error", "Calibration failed due to timeout")
            return

        time.sleep(1)


class CalibrationGUI:
    """ Tkinter GUI for Pick and Place Calibration. """
    def __init__(self, root):
        self.root = root
        self.root.title("Calibration GUI")

        # Frames
        frame = tk.Frame(root, padx=20, pady=20)
        frame.pack(pady=10)

        # Parent Frame Dropdown
        tk.Label(frame, text="Parent Frame:").grid(row=0, column=0, sticky="w")
        self.parent_frame = ttk.Combobox(frame, values=["marker_pallet", "world", "base_link"], width=20)
        self.parent_frame.grid(row=0, column=1)
        self.parent_frame.set("marker_pallet")  # Default value

        # Child Frame Dropdown
        tk.Label(frame, text="Child Frame:").grid(row=1, column=0, sticky="w")
        self.child_frame = ttk.Combobox(frame, values=["target_pick", "target_place","marker_pallet"], width=20)
        self.child_frame.grid(row=1, column=1)
        self.child_frame.set("target_pick")  # Default value

        # Robot End Effector
        tk.Label(frame, text="Robot Effector:").grid(row=2, column=0, sticky="w")
        self.robot_eff = ttk.Combobox(frame, values=["tool0", "gripper_link"], width=20)
        self.robot_eff.grid(row=2, column=1)
        self.robot_eff.set("tool0")  # Default value

        # Buttons
        self.pick_btn = tk.Button(frame, text="Pick Calibration", command=self.calibrate_pick, bg="lightblue")
        self.pick_btn.grid(row=3, column=0, pady=10)

        self.place_btn = tk.Button(frame, text="Place Calibration", command=self.calibrate_place, bg="lightgreen")
        self.place_btn.grid(row=3, column=1, pady=10)

        # Status Message
        self.status_label = tk.Label(frame, text="", fg="blue")
        self.status_label.grid(row=4, column=0, columnspan=2)

    def calibrate_pick(self):
        """ Calibrate Pick Frame. """
        self.status_label.config(text="Calibrating Pick Frame...")
        self.root.update()
        perform_calibration(self.parent_frame.get(), "target_pick", self.robot_eff.get())
        self.status_label.config(text="Pick Calibration Done ✅")

    def calibrate_place(self):
        """ Calibrate Place Frame. """
        self.status_label.config(text="Calibrating Place Frame...")
        self.root.update()
        perform_calibration(self.parent_frame.get(), "target_place", self.robot_eff.get())
        self.status_label.config(text="Place Calibration Done ✅")


if __name__ == "__main__":
    try:
        rospy.init_node('calibration_gui', anonymous=True)
        root = tk.Tk()
        app = CalibrationGUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
