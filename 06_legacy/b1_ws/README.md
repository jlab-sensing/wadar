# This portion of the project is stored in legacy because it is not currently being pursued.

# Unitree B1 Workspace

This subfolder contains the necessary files and instructions to integrate the wadar sensing system with the Unitree B1 quadruped robot using ROS 2 Iron. Follow the steps below to set up and build the package. These steps assume you have already cloned this repository and have followed the directions in the root README.md to install general dependencies.

## B1 Workspace Dependencies

```bash
sudo apt install libgeographic-dev
sudo apt install ros-iron-navigation2
sudo apt install ros-iron-nav2-bringup
sudo apt install ros-iron-turtlebot3-gazebo
sudo apt install ros-iron-joint-state-publisher-gui
sudo apt install ros-iron-xacro
sudo apt install ros-iron-gazebo-ros-pkgs
sudo apt install ros-iron-robot-localization
sudo apt install ros-iron-slam-toolbox
sudo apt install ros-iron-tf2-tools
sudo apt install ros-iron-mapviz
sudo apt install ros-iron-mapviz-plugins
sudo apt install ros-iron-tile-map
```

## Setup

### 1. Clone the Repository and Submodule

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

See [package descriptions](https://github.com/jlab-sensing/wadar/blob/master/b1_ws/src/README.md).


## Citations

```
@inproceedings{macenski2020marathon2,
    title={Marathon 2: A Navigation System},
    author={Macenski, Steve and Foote, Tully and Gerkey, Brian and Lalancette, Martin and Woodall, William},
    booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    pages={2718--2725},
    year={2020},
    organization={IEEE}
}
```