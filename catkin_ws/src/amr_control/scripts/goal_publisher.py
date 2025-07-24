#!/usr/bin/env python
import rospy
import json
import os
import Tkinter as tk
import ttk
import tkMessageBox as messagebox  # For showing popups
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID  # Used for canceling goals
import tf  # For obtaining the current robot pose

# Global variables to store goals and JSON path
goals = {}
json_path = None

def load_goals():
    global json_path
    # Build the absolute path to the JSON config file
    script_dir = os.path.dirname(os.path.realpath(__file__))
    json_path = os.path.join(script_dir, "../config/goal_locations.json")
    try:
        with open(json_path, 'r') as f:
            loaded_goals = json.load(f)
        return loaded_goals
    except Exception as e:
        rospy.logerr("Could not load goal config: %s", e)
        return {}

def save_goals(new_goals):
    global json_path
    try:
        with open(json_path, 'w') as f:
            json.dump(new_goals, f, indent=4)
        rospy.loginfo("Goals saved successfully.")
    except Exception as e:
        rospy.logerr("Error saving goals: %s", e)

def publish_goal(goal_name, goal_data, publisher):
    # Retrieve the selected goal's data from the config
    goal = goal_data.get(goal_name)
    if goal is None:
        rospy.logerr("Goal '%s' not found", goal_name)
        return
    # Create and fill the PoseStamped message
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.position.x = goal['x']
    msg.pose.position.y = goal['y']
    msg.pose.position.z = goal['z']
    msg.pose.orientation.x = goal['qx']
    msg.pose.orientation.y = goal['qy']
    msg.pose.orientation.z = goal['qz']
    msg.pose.orientation.w = goal['qw']
    rospy.loginfo("Publishing goal: %s", goal_name)
    publisher.publish(msg)

def fill_current_pose(entries):
    """Fill the entry fields with the robot's current pose using a TF lookup."""
    listener = tf.TransformListener()
    try:
        # Wait for the transform from "map" to "amr/base_link" to become available
        listener.waitForTransform("map", "amr/base_link", rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = listener.lookupTransform("map", "amr/base_link", rospy.Time(0))
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        messagebox.showerror("Error", "Could not get current pose: {}".format(e))
        return
    entries['x'].delete(0, tk.END)
    entries['x'].insert(0, str(trans[0]))
    entries['y'].delete(0, tk.END)
    entries['y'].insert(0, str(trans[1]))
    entries['z'].delete(0, tk.END)
    entries['z'].insert(0, str(trans[2]))
    entries['qx'].delete(0, tk.END)
    entries['qx'].insert(0, str(rot[0]))
    entries['qy'].delete(0, tk.END)
    entries['qy'].insert(0, str(rot[1]))
    entries['qz'].delete(0, tk.END)
    entries['qz'].insert(0, str(rot[2]))
    entries['qw'].delete(0, tk.END)
    entries['qw'].insert(0, str(rot[3]))

def open_add_location_window(root, publisher, dropdown_var, dropdown_menu):
    """Open a popup window to add a new location."""
    add_window = tk.Toplevel(root)
    add_window.title("Add New Location")
    
    # Labels and entry fields for the new location data
    labels = ['Name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    entries = {}
    for i, label_text in enumerate(labels):
        tk.Label(add_window, text=label_text).grid(row=i, column=0, padx=5, pady=5)
        ent = tk.Entry(add_window)
        ent.grid(row=i, column=1, padx=5, pady=5)
        entries[label_text] = ent

    # Button to fill the entry fields with the current robot pose
    fill_btn = tk.Button(add_window, text="Fill with Current Pose", 
                         command=lambda: fill_current_pose(entries))
    fill_btn.grid(row=len(labels), column=0, columnspan=2, pady=5)
    
    def save_new_location():
        new_name = entries['Name'].get().strip()
        if not new_name:
            messagebox.showerror("Error", "Location name cannot be empty.")
            return
        try:
            # Convert numerical values
            new_pose = {
                'x': float(entries['x'].get()),
                'y': float(entries['y'].get()),
                'z': float(entries['z'].get()),
                'qx': float(entries['qx'].get()),
                'qy': float(entries['qy'].get()),
                'qz': float(entries['qz'].get()),
                'qw': float(entries['qw'].get())
            }
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numerical values for pose.")
            return
        global goals
        if new_name in goals:
            messagebox.showerror("Error", "A location with that name already exists.")
            return
        goals[new_name] = new_pose
        save_goals(goals)
        # Update the drop-down: add new option
        dropdown_menu['menu'].add_command(label=new_name, command=tk._setit(dropdown_var, new_name))
        messagebox.showinfo("Success", "New location added successfully!")
        add_window.destroy()
    
    # Save button in the add location window
    save_btn = tk.Button(add_window, text="Save Location", command=save_new_location)
    save_btn.grid(row=len(labels)+1, column=0, columnspan=2, pady=10)

def refresh_dropdown(dropdown_var, dropdown_menu):
    """Reload goals from the config file and update the drop-down options."""
    global goals
    goals = load_goals()
    dropdown_menu['menu'].delete(0, 'end')
    for key in goals.keys():
        dropdown_menu['menu'].add_command(label=key, command=tk._setit(dropdown_var, key))
    # Optionally, set the variable to the first option if available
    if goals:
        first = list(goals.keys())[0]
        dropdown_var.set(first)
    else:
        dropdown_var.set("")

def main():
    rospy.init_node('goal_gui_publisher')
    # Create publishers
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    
    # Load goal definitions from the JSON configuration file
    global goals
    goals = load_goals()
    if not goals:
        rospy.logerr("No goals loaded. Exiting.")
        return

    rospy.sleep(1)
    
    # Create a simple Tkinter GUI
    root = tk.Tk()
    root.title("Goal Selection GUI")
    
    # Create a drop-down for selecting a goal
    dropdown_var = tk.StringVar(root)
    initial = list(goals.keys())[0] if goals else ""
    dropdown_var.set(initial)
    dropdown_menu = tk.OptionMenu(root, dropdown_var, *goals.keys())
    dropdown_menu.config(width=20)
    dropdown_menu.pack(pady=10)
    
    # Button to publish the selected goal
    pub_btn = tk.Button(root, text="Publish Selected Goal", width=20, height=2,
                        command=lambda: publish_goal(dropdown_var.get(), goals, goal_pub))
    pub_btn.pack(pady=5)
    
    # Button to cancel the current goal
    def cancel_goal():
        cancel_msg = GoalID()  # An empty GoalID cancels all active goals
        cancel_pub.publish(cancel_msg)
        messagebox.showinfo("Goal Cancelled", "Goal cancelled. The robot will remain at its current position.")
    cancel_btn = tk.Button(root, text="Cancel Goal", width=20, height=2, command=cancel_goal)
    cancel_btn.pack(pady=5)
    
    # Button to add a new location
    add_location_btn = tk.Button(root, text="Add New Location", width=20, height=2,
                                 command=lambda: open_add_location_window(root, goal_pub, dropdown_var, dropdown_menu))
    add_location_btn.pack(pady=5)
    
    # Button to refresh the drop-down (reload config file)
    refresh_btn = tk.Button(root, text="Refresh Locations", width=20, height=2,
                            command=lambda: refresh_dropdown(dropdown_var, dropdown_menu))
    refresh_btn.pack(pady=5)
    
    root.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
