FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
ARG UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu
ARG UBUNTU_SECURITY_MIRROR=http://security.ubuntu.com/ubuntu
ARG ROS_MIRROR=http://packages.ros.org/ros2/ubuntu
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_SECURITY_MIRROR}|g" /etc/apt/sources.list.d/ubuntu.sources \
    && sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_MIRROR}|g; s/^Types:.*/Types: deb/" /etc/apt/sources.list.d/ros2.sources \
    && apt-get -o Acquire::Retries=3 update \
    && DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
    gcc python3-numpy python3-scipy python3-matplotlib python3-pil python3-yaml \
    python3-serial python3-colcon-common-extensions xvfb xauth procps fonts-noto-cjk \
    libgl1-mesa-dri mesa-utils ros-jazzy-ros-gz ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization \
    ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher \
    ros-jazzy-laser-filters ros-jazzy-topic-tools ros-jazzy-diagnostic-msgs \
    ros-jazzy-sensor-msgs-py ros-jazzy-rviz2
ENV LIBGL_ALWAYS_SOFTWARE=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=76
WORKDIR /work
CMD ["sleep", "infinity"]
