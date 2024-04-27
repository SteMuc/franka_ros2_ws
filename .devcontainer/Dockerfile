FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

WORKDIR /workspaces

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown user:user /run/user/"${USER_UID}" \
    && chown user:user /workspaces \
    && chown user:user /run/user/"${USER_UID}"/gdm \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL /bin/bash


RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-ros2-control-test-assets \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ign-ros2-control \
    libignition-gazebo6-dev \
    libpoco-dev \
    libeigen3-dev \
    nano \
    lsb-release \
    wget \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir ~/source_code
RUN cd ~/source_code && git clone https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout 0.13.3 \
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF  .. \
    && make franka -j$(nproc) \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

## Setup Ignition Gazebo Harmonic LTS until Sep. 2028
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-${ROS_DISTRO}-ros-ign-bridge
    
RUN addgroup realtime
RUN usermod -a -G realtime $USERNAME

ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

# set the default user to the newly created user
USER $USERNAME
CMD ["/bin/bash"]


RUN echo "source /ros_entrypoint.sh" >>~/.bashrc
RUN echo "export ROS_DOMAIN_ID=100" >>~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >>~/.bashrc 
RUN echo "export LC_NUMERIC=en_US.UTF-8" >>~/.bashrc 
