# PlatformIO commands

## Build the project
platformio run

## Upload to device
platformio run --target upload

## Upload filesystem (SPIFFS)
platformio run --target uploadfs

## Monitor serial output
platformio device monitor

## Run micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i 192.168.18.16
ros2 topic echo /telemetry
