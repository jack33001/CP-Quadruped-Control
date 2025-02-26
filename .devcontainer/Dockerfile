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

# RUN mkdir -p /home/build_motor_driver \
#     && cd /home/build_motor_driver \
#     && wget https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can/archive/refs/tags/v0.2.tar.gz \
#     && tar xvf v0.2.tar.gz \
#     && cd mini-cheetah-tmotor-can-0.2 \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make \
#     && sudo make install


RUN addgroup realtime \
    && usermod -a -G realtime $USERNAME \
    && echo "@realtime soft rtprio 99" >> /etc/security/limits.conf \
    && echo "@realtime soft priority 99" >> /etc/security/limits.conf \
    && echo "@realtime soft memlock 102400" >> /etc/security/limits.conf \
    && echo "@realtime hard rtprio 99" >> /etc/security/limits.conf \
    && echo "@realtime hard priority 99" >> /etc/security/limits.conf \
    && echo "@realtime hard memlock 102400" >> /etc/security/limits.conf

# ACADOS installation in separate steps
RUN mkdir -p /home/acados
WORKDIR /home/acados

RUN git clone https://github.com/acados/acados.git
WORKDIR /home/acados/acados

RUN git submodule update --recursive --init

RUN mkdir -p build
WORKDIR /home/acados/acados/build

RUN cmake -DACADOS_WITH_QPOASES=ON ..
RUN make install -j4

# Install CasADi
RUN mkdir -p /home/casadi \
    && cd /home/casadi \
    && wget https://github.com/casadi/casadi/archive/refs/tags/3.6.7.tar.gz \
    && tar -xf 3.6.7.tar.gz \
    && cd casadi-3.6.7 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \ 
    && make install

# Install ROS2 dependencies
RUN apt-get update \
    && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update \
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
    ros-jazzy-pinocchio \
    ros-jazzy-joint-state-publisher-gui 

# Install ROS2 debugging dependencies
RUN apt-get update \
    && apt-get update \
    sudo apt install xterm gdb gdbserver



# Clone ros2_control repositories
RUN git clone https://github.com/ros-controls/ros2_control.git src/ros2_control
RUN git clone https://github.com/ros-controls/ros2_controllers.git src/ros2_controllers


# More ACADOS setup
RUN pip install --break-system-packages -e /home/acados/acados/interfaces/acados_template
RUN pip install --break-system-packages \
    casadi
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/acados/acados/lib"' >> /home/$USERNAME/.bashrc \
    && echo 'export ACADOS_SOURCE_DIR="/home/acados/acados"' >> /home/$USERNAME/.bashrc

# Fix t_renderer installation
RUN cd /home/acados/acados/bin \
    && wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
    && mv t_renderer-v0.0.34-linux t_renderer \
    && sudo chown $USERNAME:$USERNAME t_renderer 


# Install required packages for ROS2 development
RUN pip3 install --break-system-packages -U colcon-common-extensions
RUN pip3 install --break-system-packages -U vcstool
RUN pip3 install --break-system-packages -U rosdep

# Install testing packages without upgrading system packages
RUN pip3 install --break-system-packages --ignore-installed pytest
RUN apt-get install -y \
    python3-pytest \
    ros-jazzy-launch-testing \
    ros-jazzy-launch-testing-ros

RUN pip3 install --break-system-packages --ignore-installed -U \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes
    
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

# FROM ros:rolling

# ARG USERNAME=USERNAME
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

# # Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
# RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# # Create the user
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     #
#     # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME
# RUN apt-get update && apt-get upgrade -y
# RUN apt-get install -y python3-pip wget
# ENV SHELL /bin/bash

# # ********************************************************
# # * Anything else you want to do like clean up goes here *
# # ********************************************************
# # Install OpenGL dependencies
# RUN apt-get install -y \
#     libgl1-mesa-dev \
#     libglu1-mesa-dev \
#     libegl1-mesa-dev \
#     libgles2-mesa-dev \
#     libglfw3-dev \
#     libglew-dev

# RUN addgroup realtime \
#     && usermod -a -G realtime $USERNAME \
#     && echo "@realtime soft rtprio 99" >> /etc/security/limits.conf \
#     && echo "@realtime soft priority 99" >> /etc/security/limits.conf \
#     && echo "@realtime soft memlock 102400" >> /etc/security/limits.conf \
#     && echo "@realtime hard rtprio 99" >> /etc/security/limits.conf \
#     && echo "@realtime hard priority 99" >> /etc/security/limits.conf \
#     && echo "@realtime hard memlock 102400" >> /etc/security/limits.conf

# # ACADOS installation in separate steps
# RUN mkdir -p /home/acados
# WORKDIR /home/acados

# RUN git clone https://github.com/acados/acados.git
# WORKDIR /home/acados/acados

# RUN git submodule update --recursive --init

# RUN mkdir -p build
# WORKDIR /home/acados/acados/build

# RUN cmake -DACADOS_WITH_QPOASES=ON ..
# RUN make install -j4

# # Install CasADi
# RUN mkdir -p /home/casadi \
#     && cd /home/casadi \
#     && wget https://github.com/casadi/casadi/archive/refs/tags/3.6.7.tar.gz \
#     && tar -xf 3.6.7.tar.gz \
#     && cd casadi-3.6.7 \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make -j4 \ 
#     && make install

# # Install ROS2 dependencies
# RUN apt-get update \
#     && apt-get install -y software-properties-common \
#     && add-apt-repository universe \
#     && apt-get update \
#     && apt-get install -y \
#     ros-rolling-xacro \
#     ros-rolling-rqt-graph \
#     ros-rolling-joint-state-publisher \
#     ros-rolling-ros-gz \
#     ros-rolling-gz-ros2-control \
#     ros-rolling-joint-state-broadcaster \
#     ros-rolling-position-controllers \
#     ros-rolling-velocity-controllers \
#     ros-rolling-effort-controllers \
#     ros-rolling-joint-trajectory-controller \
#     ros-rolling-imu-sensor-broadcaster \
#     ros-rolling-pinocchio \
#     ros-rolling-joint-state-publisher-gui 

# # Set environment variables for ros2_control
# ENV ROS_UNDERLAY /root/ws_ros2_control/install
# WORKDIR $ROS_UNDERLAY/../src

# # Download and import ros2_control repositories
# ADD https://raw.githubusercontent.com/ros-controls/ros2_control/master/ros2_control.rolling.repos ros2_control.repos
# RUN vcs import < ros2_control.repos

# # Update and install dependencies
# RUN apt-get update && rosdep update && \
#     rosdep install -iy --from-paths . && \
#     rm -rf /var/lib/apt/lists/

# # Build the ros2_control workspace
# RUN cd $ROS_UNDERLAY/.. && \
#        . /opt/ros/${ROS_DISTRO}/setup.sh && \
#        colcon build

# # Source the setup.bash for ros2_control
# RUN sed --in-place --expression \
#       '$isource "$ROS_UNDERLAY/setup.bash"' \
#       /ros_entrypoint.sh

# # More ACADOS setup
# RUN pip install --break-system-packages -e /home/acados/acados/interfaces/acados_template
# RUN pip install --break-system-packages \
#     casadi
# RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/acados/acados/lib"' >> /home/$USERNAME/.bashrc \
#     && echo 'export ACADOS_SOURCE_DIR="/home/acados/acados"' >> /home/$USERNAME/.bashrc

# # Fix t_renderer installation
# RUN cd /home/acados/acados/bin \
#     && wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
#     && mv t_renderer-v0.0.34-linux t_renderer \
#     && sudo chown $USERNAME:$USERNAME t_renderer 

# # Install required packages for ROS2 development
# RUN pip3 install --break-system-packages -U colcon-common-extensions
# RUN pip3 install --break-system-packages -U vcstool
# RUN pip3 install --break-system-packages -U rosdep

# # Install testing packages without upgrading system packages
# RUN pip3 install --break-system-packages --ignore-installed pytest
# RUN apt-get install -y \
#     python3-pytest \
#     ros-rolling-launch-testing \
#     ros-rolling-launch-testing-ros

# RUN pip3 install --break-system-packages --ignore-installed -U \
#     flake8 \
#     flake8-blind-except \
#     flake8-builtins \
#     flake8-class-newline \
#     flake8-comprehensions \
#     flake8-deprecated \
#     flake8-docstrings \
#     flake8-import-order \
#     flake8-quotes
    
# # [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash"]