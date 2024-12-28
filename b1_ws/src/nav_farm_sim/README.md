# nav_farm_sim Simulation Node

## Gazebo and Rviz

Run
```bash
colcon build --packages-select nav_farm_sim
. install/setup.bash
ros2 launch nav_farm_sim gps_waypoint_follower.launch.py use_rviz:=True
```

## Mapviz

Run
```bash
. install/setup.bash
ros2 launch nav_farm_sim mapviz.launch.py
```

In "Base URL:" field, enter
```bash
http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
```

## Logged Waypoint Follower

Run
```bash
. install/setup.bash
ros2 run nav_farm_sim logged_waypoint_follower /home/ericdvet/jlab/wadar/b1_ws/src/nav_farm_sim/config/demo_waypoints.yaml
```

## Iterative Point Follower

Run
```bash
. install/setup.bash
ros2 run nav_farm_sim interactive_waypoint_follower
```