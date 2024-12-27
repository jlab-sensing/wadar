# GPS Based Navigation for Walker

This node performs path-planning using GPS coordinates for a hand-held radar.

## Building the Package

To build the package use:
```bash
colcon build --packages-select nav_farm_walker
. install/setup.bash
ros2 launch nav_farm_walker gps_waypoint_follower.launch.py use_rviz:=True
```

## Launching Map Visualization

To launch the map visualization use:
```bash
. install/setup.bash
ros2 launch nav_farm_walker mapviz.launch.py
```
If the map does not display, set up mapproxy properly.

## Running Logged Waypoint Follower

To run the logged waypoint follower use:
```bash
. install/setup.bash
ros2 run nav_farm_walker logged_waypoint_follower /home/ericdvet/jlab/wadar/b1_ws/src/nav_farm_walker/config/demo_waypoints.yaml
```

## Running Interactive Waypoint Follower

To run the interactive waypoint follower use:
```bash
. install/setup.bash
ros2 run nav_farm_walker interactive_waypoint_follower
```

## Additional Setup

In a new terminal, run:
```bash
. install/setup.bash
ros2 run nav_farm_walker gui
```
