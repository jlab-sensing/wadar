# GPS Based Navigation for Simulation

This node performs path-planning using GPS coordinates for Gazebo simulation.

## Building the Package

To build the package use:
```bash
colcon build --packages-select nav_farm_sim
. install/setup.bash
ros2 launch nav_farm_sim gps_waypoint_follower.launch.py use_rviz:=True
```

## Launching Map Visualization

To launch the map visualization use:
```bash
. install/setup.bash
ros2 launch nav_farm_sim mapviz.launch.py
```
If the map does not display, set up mapproxy properly.

## Running Logged Waypoint Follower

To run the logged waypoint follower use:
```bash
. install/setup.bash
ros2 run nav_farm_sim logged_waypoint_follower /home/ericdvet/jlab/wadar/b1_ws/src/nav_farm_sim/config/demo_waypoints.yaml
```

## Running Interactive Waypoint Follower

To run the interactive waypoint follower use:
```bash
. install/setup.bash
ros2 run nav_farm_sim interactive_waypoint_follower
```

## Additional Setup

In a new terminal, run:
```bash
. install/setup.bash
ros2 run nav_farm_sim gui
```
