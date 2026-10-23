FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    python3-pytest \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    x11-utils \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2_ws/src src
COPY docker/start_rviz_web.sh /usr/local/bin/start_rviz_web.sh
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc

CMD ["bash"]
