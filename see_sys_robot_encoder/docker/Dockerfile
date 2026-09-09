FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# TODO: copy ros2_ws/src/robot_base_driver into /ros2_ws/src and colcon build it here, once that package exists.

WORKDIR /ros2_ws
CMD ["bash"]
