# BRUCE!
BRUCE is the Cal Poly Legged Robotics Group's 8-DOF quadruped. It is designed to be a research platform, with plenty of computational overhead and easily manufacturable parts. This repository contains the software necessary to use BRUCE. Note that this repository does *not* contain the firmware for BRUCE's power distribution board. For that, check [insert link](staggeringbeauty.com). 

# Installation
This system is built to run on an NVIDIA Jetson Orin AGX Nano, inside of a Docker container. 

## Dockerfile Setup
First, clone this repository to your desired workspace. You will need to spin up a docker container using the included Dockerfile; this takes a while. There is also a devcontainer.json file included for use with VSCode's remote development extension. 

## First-Time Setup
After the Docker container has been created, source /opt/ros/jazzy/setup.bash. Next, colcon_build the project. Lastly, source install/setup.bash to source the project files, and the system should be ready to run.

# Use
## Running in simulation
Currently, Gazebo is the only simulation environment supported within this project. To run a simulation, launch the file `sim.launch.py` from the `quadruped_bringup` folder. 

## Running on hardware