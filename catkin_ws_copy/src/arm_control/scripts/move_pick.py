#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Pose
import Tkinter as tk
import tkMessageBox as messagebox  # Import messagebox for showing popups
import tf  # Import the tf library for quaternion to Euler conversion

class FrameListener:
    def __init__(self):
        rospy.init_node("frame_listener", anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.available_frames = []  # List of all frames
        self.max_attempts = 10
        self.retry_delay = 1  # Retry delay in seconds
        self.wait_for_frames()

    def wait_for_frames(self):
        """ Wait for frames to be available in the TF tree. """
        rospy.loginfo("Waiting for TF frames...")
        attempts = 0
        while attempts < self.max_attempts:
            try:
                # Get all frames available in the TF tree
                all_frames = self.tf_buffer.all_frames_as_string().splitlines()
                rospy.loginfo("Found frames: \n{}".format(all_frames))  # Log all available frames

                # Store all frame names (without additional info)
                self.available_frames = [frame.split(' ')[1] for frame in all_frames]  # Extract frame names
                if self.available_frames:
                    return
                else:
                    rospy.loginfo("No frames found on attempt {}".format(attempts + 1))

            except Exception as e:
                rospy.logerr("Error getting frames: {}".format(e))

            attempts += 1
            rospy.sleep(self.retry_delay)  # Retry after a delay

    def get_transform(self, frame):
        """ Get the transform of the selected frame with respect to the 'world' frame. """
        try:
            transform = self.tf_buffer.lookup_transform("world", frame, rospy.Time(0))
            position = transform.transform.translation
            orientation = transform.transform.rotation
            return position, orientation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error looking up transform for frame {}: {}".format(frame, e))
            return None, None

    def quaternion_to_euler(self, quaternion):
        """ Convert quaternion to roll, pitch, yaw. """
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return roll, pitch, yaw

class GUI:
    def __init__(self, frame_listener):
        self.frame_listener = frame_listener

        # Setup GUI window
        self.root = tk.Tk()
        self.root.title("TF Frame Viewer")

        # Dropdown for frame selection
        self.frame_var = tk.StringVar(self.root)
        self.frame_var.set("Select Frame")  # Default value
        self.frame_menu = tk.OptionMenu(self.root, self.frame_var, *self.frame_listener.available_frames)
        self.frame_menu.pack(pady=10)

        # Button to select the frame and read the transform
        self.read_button = tk.Button(self.root, text="Read Transform", command=self.read_transform, font=("Arial", 12))
        self.read_button.pack(pady=10)

        # Label to display the transform
        self.transform_label = tk.Label(self.root, text="Position and Orientation will be displayed here.", font=("Arial", 12))
        self.transform_label.pack(pady=20)

        # Refresh button to update the frames
        self.refresh_button = tk.Button(self.root, text="Refresh Frames", command=self.refresh_frames, font=("Arial", 12))
        self.refresh_button.pack(pady=10)

        # Exit button
        self.exit_button = tk.Button(self.root, text="Exit", command=self.root.quit, font=("Arial", 12))
        self.exit_button.pack(pady=10)

    def refresh_frames(self):
        """ Refresh the list of frames """
        self.frame_listener.wait_for_frames()
        self.frame_menu['menu'].delete(0, 'end')  # Clear existing menu options
        for frame in self.frame_listener.available_frames:
            self.frame_menu['menu'].add_command(label=frame, command=tk._setit(self.frame_var, frame))

    def read_transform(self):
        """ Read the transform for the selected frame and display position and orientation. """
        selected_frame = self.frame_var.get()
        if selected_frame == "Select Frame":
            messagebox.showwarning("Selection Error", "Please select a valid frame.")
            return
        
        # Get transform for selected frame
        position, orientation = self.frame_listener.get_transform(selected_frame)
        
        if position and orientation:
            # Convert quaternion to roll, pitch, yaw
            roll, pitch, yaw = self.frame_listener.quaternion_to_euler(orientation)

            # Display the position and roll, pitch, yaw
            position_str = "Position: x = {:.2f}, y = {:.2f}, z = {:.2f}".format(position.x, position.y, position.z)
            orientation_str = "Roll = {:.2f}, Pitch = {:.2f}, Yaw = {:.2f}".format(roll, pitch, yaw)
            self.transform_label.config(text=position_str + "\n" + orientation_str)
        else:
            self.transform_label.config(text="Failed to get transform for frame {}".format(selected_frame))

    def run(self):
        """ Start the GUI loop. """
        self.root.mainloop()

def main():
    # Initialize the frame listener to get frames from the TF tree
    frame_listener = FrameListener()

    # Initialize the GUI
    gui = GUI(frame_listener)

    # Run the GUI loop
    gui.run()

if __name__ == "__main__":
    main()
