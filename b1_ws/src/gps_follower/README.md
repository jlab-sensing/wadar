# gps_follower Node

## Gazebo and Rviz

Run
```bash
colcon build --packages-select gps_follower
. install/setup.bash
ros2 launch gps_follower gps_waypoint_follower.launch.py use_rviz:=True
```

## Mapviz

Run
```bash
. install/setup.bash
ros2 launch gps_follower mapviz.launch.py
```

In "Base URL:" field, enter
```bash
http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
```

## Logged Waypoint Follower

Run
```bash
. install/setup.bash
ros2 run gps_follower logged_waypoint_follower /home/ericdvet/jlab/wadar/b1_ws/src/gps_follower/config/demo_waypoints.yaml
```

## Iterative Point Follower

Run
```bash
. install/setup.bash
ros2 run gps_follower interactive_waypoint_follower
```