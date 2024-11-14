#!/bin/bash

# Constant feedback_vel publisher at 100 Hz
ros2 topic pub -r 100 /visual_slam/tracking/odometry nav_msgs/msg/Odometry "header:
  frame_id: 'odom'
child_frame_id: 'base_link'
twist:
  twist:
    linear:
      x: 0.5
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0" &

# Constant cmd_vel_in publisher at 5 Hz
ros2 topic pub -r 5 /cmd_vel_in_pid geometry_msgs/msg/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" &

# Wait for background processes to finish
wait
