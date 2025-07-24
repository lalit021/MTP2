# Use the official ROS Noetic desktop-full image from osrf
FROM osrf/ros:noetic-desktop-full

# Set environment variables to avoid interactive prompts
ENV LANG C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    ros-noetic-pcl-ros \
    # Add any other packages here
    && rm -rf /var/lib/apt/lists/*

# Set up the catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Set the working directory for the build process
WORKDIR /root/catkin_ws/src

# The default command when the container runs will be bash
CMD ["bash"]
