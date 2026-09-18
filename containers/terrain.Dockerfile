FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    gcc python3-numpy python3-serial python3-colcon-common-extensions xvfb xauth \
    libgl1-mesa-dri mesa-utils ros-jazzy-ros-gz ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization \
    ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher \
    ros-jazzy-laser-filters ros-jazzy-topic-tools ros-jazzy-diagnostic-msgs \
    ros-jazzy-sensor-msgs-py ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*
ENV LIBGL_ALWAYS_SOFTWARE=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=76
WORKDIR /work
CMD ["sleep", "infinity"]
