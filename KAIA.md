On microcontroller
Terminal
```
Telem avg 51 max 52ms, LiDAR RPM 0.00, wheels RPM 0.00 0.00, battery 0.00V, RSSI -29dBm
```
Micro-ROS messages
```
---
stamp:
  sec: 1762967915
  nanosec: 874411000
seq: 193171
odom_pos_x: 0.026938902214169502
odom_pos_y: 0.5149940848350525
odom_pos_yaw: -0.1071779727935791
odom_vel_x: 0.0
odom_vel_yaw: 0.0
joint:
- pos: -124.45195007324219
  vel: 0.0
- pos: 123.92835235595703
  vel: 0.0
wifi_rssi_dbm: -33
battery_mv: 0
distance_mm: []
bumper: []
cliff: []
touch: []
scan_start_hint: false
lds: []
---
```

Micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i 192.168.18.16

Wifi setup
http://192.168.4.1/

Telemetry for robot topics
ros2 run kaiaai_telemetry telem

Publish robot urdf and visualize
ros2 launch kaiaai_bringup inspect_urdf.launch.py joints:=nogui robot_model:=makerspet_mini

teleop
ros2 topic echo /scan
