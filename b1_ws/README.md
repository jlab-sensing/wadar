# Unitree B1 Workspace

This subfolder contains the necessary files and instructions to integrate the wadar sensing system with the Unitree B1 quadruped robot using ROS 2 Iron. Follow the steps below to set up and build the package. These steps assume you have already cloned this repository and have followed the directions in the root README.md to install all dependencies.

## Setup

### 1. Clone the Repository and Submodule

These steps assume you have already cloned this repository and have followed the directions in the root README.md to install all dependencies.

```bash
cd b1_ws/src/inertial-sense-sdk
git submodule update --init --recursive
```

### 2. Create a Symbolic Link

To allow colcon build to find the appropriate ROS 2 packages, return to the workspace directory and create a symbolic link from the inertial-sense-sdk submodule's ROS 2 directory to your workspace src directory:

```bash
cd src
ln -s wadar/inertial-sense-sdk/ros2 .
```

### 3. Build the Workspace

Return to the workspace and build the package using colcon:
```bash
source /opt/ros/iron/setup.bash
colcon build
```

### 4. Source the Setup Script

For ROS 2 to function properly, you need to source the setup script to set up the necessary environment variables. Run the following command:
```bash
. install/setup.bash
```

## Packages

TODO: