# Unitree B1 Integration

This subfolder contains the necessary files and instructions to integrate the wadar sensing system with the Unitree B1 quadruped robot using ROS 2 Iron. Follow the steps below to set up and build the package.

##Setup

### 1. Clone the Repository and Submodule

First, clone this repository and ensure the inertial-sense-sdk submodule is included. Navigate to your ROS 2 workspace and clone the required package:

```bash
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/jlab-sensing/wadar
cd wadar
git submodule update --init --recursive
```

### 2. Create a Symbolic Link

To allow colcon build to find the appropriate ROS 2 package, create a symbolic link from the inertial-sense-sdk submodule's ROS 2 directory to your workspace src directory:

```bash
cd dev_ws/src
sudo ln -s wadar/inertial-sense-sdk/ros2 .
sudo ln -s wadar/chipotle-radar .
```

### 3. Build the Workspace

Return to the root of your ROS 2 workspace and build the package using colcon:
```bash
cd dev_ws
colcon build
```

### 4. Source the Setup Script

For ROS 2 to function properly, you need to source the setup script to set up the necessary environment variables. Run the following command:
```bash
. install/setup.bash
```

You will need to run this command every time you open a new terminal to have access to the ROS 2 commands. To avoid this, you can add the line to your ~/.bashrc:
```bash
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```