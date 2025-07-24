import rospy
import tf
import json
import Tkinter as tk
import ttk
import tkMessageBox as messagebox  # Import messagebox for showing popups

class TFFrameSaver:
    def __init__(self):
        rospy.init_node('save_tf_frames', anonymous=True)

        # Initialize the TF listener
        self.listener = tf.TransformListener()

        # GUI Setup
        self.root = tk.Tk()  # Initialize the Tkinter root window
        self.root.title("Save TF Frames")

        # Frames setup
        self.parent_frame_var = tk.StringVar()
        self.child_frame_var = tk.StringVar()

        self.setup_gui()

        # Start periodic updates of the frames dropdown
        self.update_frames()

    def setup_gui(self):
        """ Setup the GUI components """
        # Parent and Child Frame Dropdowns
        self.label_parent_frame = tk.Label(self.root, text="Parent Frame:")
        self.label_parent_frame.grid(row=0, column=0, padx=10, pady=5)
        self.parent_frame_dropdown = ttk.Combobox(self.root, textvariable=self.parent_frame_var)
        self.parent_frame_dropdown.grid(row=0, column=1, padx=10, pady=5)

        self.label_child_frame = tk.Label(self.root, text="Child Frame:")
        self.label_child_frame.grid(row=1, column=0, padx=10, pady=5)
        self.child_frame_dropdown = ttk.Combobox(self.root, textvariable=self.child_frame_var)
        self.child_frame_dropdown.grid(row=1, column=1, padx=10, pady=5)

        # Save button
        self.save_button = tk.Button(self.root, text="Save Transform Data", command=self.save_data)
        self.save_button.grid(row=2, column=0, columnspan=2, pady=20)

        # Refresh button
        self.refresh_button = tk.Button(self.root, text="Refresh Transforms", command=self.refresh_data)
        self.refresh_button.grid(row=3, column=0, columnspan=2, pady=10)

    def get_frames_from_world(self):
        """ Get all frames in the TF tree starting from 'world' """
        available_frames = []
        try:
            # Get the available frames from the TF listener
            all_frames = self.listener.getFrameStrings()

            # Filter out frames starting from 'world' and its children
            for frame in all_frames:
                try:
                    # Check if we can get a transformation between 'world' and each frame
                    self.listener.lookupTransform("world", frame, rospy.Time(0))
                    available_frames.append(frame)
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            return available_frames

        except rospy.ROSInterruptException:
            pass
        return []

    def update_frames(self):
        """ Update available frames in dropdowns continuously from 'world' frame """
        try:
            # Get the frames from 'world' and update the dropdown options
            available_frames = self.get_frames_from_world()

            # Update the dropdown options (parent and child frames)
            self.parent_frame_dropdown['values'] = available_frames
            self.child_frame_dropdown['values'] = available_frames

        except rospy.ROSInterruptException:
            pass

        # Schedule the next update (every second)
        self.root.after(1000, self.update_frames)

    def save_data(self):
        """ Save the transformation data in JSON format """
        try:
            parent_frame = self.parent_frame_var.get()
            child_frame = self.child_frame_var.get()

            # Check if both parent and child frames are selected
            if not parent_frame or not child_frame:
                messagebox.showerror("Error", "Please select both Parent and Child frames.")
                return

            # Continuously check for the latest transform before saving
            self.listener.waitForTransform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(1.0))
            (translation, rotation) = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

            # Save the latest transformation data in the desired format
            transform_data = {
                "transforms": [
                    {
                        "parent_frame": parent_frame,
                        "child_frame": child_frame,
                        "translation": translation,
                        "rotation": rotation
                    }
                ]
            }

            # Save data to a JSON file
            with open("transforms_data.json", "w") as json_file:
                json.dump(transform_data, json_file, indent=4)

            messagebox.showinfo("Success", "Data saved successfully!")

        except (tf.Exception, rospy.ROSInterruptException) as e:
            messagebox.showerror("Error", "An error occurred: {}".format(e))

    def refresh_data(self):
        """ Refresh the available frames and update the dropdowns """
        self.update_frames()

    def run(self):
        """ Start the GUI loop. """
        self.root.mainloop()


# Run the program
if __name__ == "__main__":
    frame_saver = TFFrameSaver()
    frame_saver.run()
