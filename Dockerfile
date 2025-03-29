FROM osrf/ros:humble-desktop

# Workspace setup
RUN mkdir -p /ros_ws/src/ros2_latency
WORKDIR /ros_ws
COPY . /ros_ws/src/ros2_latency

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install
