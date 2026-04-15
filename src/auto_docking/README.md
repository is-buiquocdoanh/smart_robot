# auto_docking

Small package to perform automatic docking:

- Listens to `/battery_state` (sensor_msgs/BatteryState). When percentage <= battery_threshold it:
  1. Sends a NavigateToPose goal to Nav2 to move to a configured charging pose.
  2. When navigation succeeds, runs an ultrasonic-based reverse docking controller that publishes to `/cmd_vel` until sonar range <= stop distance.

How to use

1. Build and source workspace:

   colcon build --packages-select auto_docking
   source install/setup.bash

2. Run (example):

   ros2 launch auto_docking auto_docking.launch.py

Configuration

Parameters in the launch file (or via parameter override):
- battery_topic (default /battery_state)
- battery_threshold (default 0.20)
- charging_pose_frame, charging_pose_x, charging_pose_y, charging_pose_yaw
- sonar_topic (default /sonar_range)
- cmd_vel_topic (default /cmd_vel)

Notes & next steps

- This is a minimal starting point. In production you may want:
  - Safety checks, obstacle clearance before reverse
  - Use a dedicated docking action/feedback from charger hardware
  - Publish charging state and integrate with battery node
