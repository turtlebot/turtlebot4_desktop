# turtlebot4_desktop
Packages for interacting with Turtlebot4 on a PC

## Rviz2

Rviz2 can be used to view the robot model, sensor data, generated maps, and more.

### View and interact with model

```bash
ros2 launch turtlebot4_viz view_model.launch.py
```

### Top-down view for mapping and navigating

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

## Diagnostics

The Turtlebot4 runs a diagnostic updater and aggregator on boot. The diagnostic data can be viewed with `rqt_robot_monitor`.

```bash
ros2 launch turtlebot4_viz view_diagnostics.launch.py
```
