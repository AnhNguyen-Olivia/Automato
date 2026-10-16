#!/bin/bash

# Terminal 1: MoveIt + RViz
cmd.exe /c start wsl.exe -- bash -c 'source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch ur10e_rg2_moveit_config demo.launch; exec bash'

# Wait a few seconds before opening Terminal 2
sleep 5

# Terminal 2: Harvest Server
cmd.exe /c start wsl.exe -- bash -c 'source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun ur10e_rg2_moveit harvest_pose_server.py; exec bash'

# Terminal 3: Unity Endpoint
cmd.exe /c start wsl.exe -- bash -c 'source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch ros_tcp_endpoint endpoint.launch; exec bash'