
FROM ros:jazzy
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip wget
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
# Install OpenGL dependencies
RUN apt-get install -y \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libglfw3-dev \
    libglew-dev

RUN mkdir -p /home/build_motor_driver \
    && cd /home/build_motor_driver \
    && wget https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can/archive/refs/tags/v0.2.tar.gz \
    && tar xvf v0.2.tar.gz \
    && cd mini-cheetah-tmotor-can-0.2 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install


RUN mkdir -p /home/mujoco \
    && cd /home/mujoco \
    && wget https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz \
    && tar xvf mujoco-3.2.3-linux-x86_64.tar.gz

RUN addgroup realtime \
    && usermod -a -G realtime $USERNAME \
    && echo "@realtime soft rtprio 99" >> /etc/security/limits.conf \
    && echo "@realtime soft priority 99" >> /etc/security/limits.conf \
    && echo "@realtime soft memlock 102400" >> /etc/security/limits.conf \
    && echo "@realtime hard rtprio 99" >> /etc/security/limits.conf \
    && echo "@realtime hard priority 99" >> /etc/security/limits.conf \
    && echo "@realtime hard memlock 102400" >> /etc/security/limits.conf


# Install ROS2 dependencies
RUN apt-get update \
    && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-xacro \
    ros-jazzy-rqt-graph \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-position-controllers \
    ros-jazzy-velocity-controllers \
    ros-jazzy-effort-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-imu-sensor-broadcaster \
    ros-jazzy-pinocchio

# Install Mujoco Python package
RUN pip install mujoco --break-system-packages


# Install required packages for ROS2 development
pip3 install -U \
    colcon-common-extensions \
    vcstool \
    rosdep \
    pytest \
    pytest-ros \
    launch-testing \
    launch-testing-ros \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    ament-flake8 \
    ament-lint \
    ament-copyright \
    ament-pep257 \
    ament-index-python

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]