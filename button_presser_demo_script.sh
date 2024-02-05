#!/bin/bash

# Function to execute a ROS 2 launch file in a new Konsole terminal
run_ros2_launch_in_konsole() {
    konsole --hold -e bash -c "ros2 launch $1"
}

run_ros2_launch_in_konsole "multi_aruco_plane_detection multi_aruco_plane_detection" &

sleep 5 &

# Spawn Konsole terminals for each ROS 2 launch file
run_ros2_launch_in_konsole "igus_rebel_moveit_config moveit_controller.launch.py hardware_protocol:=cri load_base:=false" &

sleep 10 &

run_ros2_launch_in_konsole "igus_rebel_commander button_press_demo.launch.py load_base:=false" 


# You can add more launch files as needed
exit 0
