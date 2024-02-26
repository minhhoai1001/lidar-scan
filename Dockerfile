FROM ros:noetic-ros-base

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install ROS dependencies
RUN apt-get update -y && \
    apt-get install -y ros-noetic-diagnostic-updater ros-noetic-laser-geometry

# Create and set working directory
RUN mkdir -p /usr/src/lidar-scan
WORKDIR /usr/src/lidar-scan

# Copy the source code to the container
COPY . /usr/src/lidar-scan

# Initialize ROS workspace and build
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /usr/src/lidar-scan; catkin_make'

# Source the ROS workspace setup in the container
RUN echo "source /usr/src/lidar-scan/devel/setup.bash" >> ~/.bashrc

# Set the entry point for the container
CMD ["roslaunch", "lidar_scan", "lidar_scan.launch"]
