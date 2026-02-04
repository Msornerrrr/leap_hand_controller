FROM ros:noetic-ros-base-focal

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-xacro \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir numpy dynamixel-sdk

# Create catkin workspace
RUN mkdir -p /catkin_ws/src

# Copy the leap_hand_controller packages into the workspace
COPY leap_hand /catkin_ws/src/leap_hand
COPY leap_description /catkin_ws/src/leap_description

# Build the catkin workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make

# Copy and set up entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
