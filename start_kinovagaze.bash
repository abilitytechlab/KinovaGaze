#!/bin/bash

echo Path in script: $PATH >> path.txt

source /opt/ros/noetic/setup.bash
source /home/kinovagaze/catkin_ws/devel/setup.bash

rm -r kinovagaze
mkdir -p kinovagaze

screen -dm -S "kinova-ros" -L -Logfile "kinovagaze/kinova-ros.log" bash -c "source /opt/ros/noetic/setup.bash; source /home/kinovagaze/catkin_ws/devel/setup.bash; roslaunch --wait kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300"
screen -dm -S "rosbridge" -L -Logfile "kinovagaze/rosbridge.log" bash -c "source /opt/ros/noetic/setup.bash; source /home/kinovagaze/catkin_ws/devel/setup.bash; roslaunch rosbridge_server rosbridge_websocket.launch"
screen -dm -S "rosbridge_listener" -L -Logfile "kinovagaze/eye_control.log" bash -c "source /opt/ros/noetic/setup.bash; source /home/kinovagaze/catkin_ws/devel/setup.bash; while true; do rosrun kinovagaze rosbridge_listener.py; done"
# until rostopic list ; do sleep 1; done
screen -dm -S "gui_server" -L -Logfile "kinovagaze/gui_server.log" bash -c "while true; do python3 ~/catkin_ws/src/kinovagaze/gui/gui_server.py; done"

echo "Started kinovagaze on separate screen instances."
screen -ls

sleep 10
service isc-dhcp-server restart

sleep 1

