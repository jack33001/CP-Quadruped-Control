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
    ros-jazzy-imu-sensor-broadcaster

# Install Mujoco Python package
RUN pip install mujoco --break-system-packages

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]