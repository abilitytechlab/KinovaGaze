#!/bin/bash

screen -X -S "kinova-ros" stuff $'\003'
screen -X -S "rosbridge" stuff $'\003'
screen -X -S "rosbridge_listener" stuff $'\003'
screen -X -S "gui_server" stuff $'\003'
sleep 0.5
screen -X -S "rosbridge_listener" stuff $'\003'
screen -X -S "gui_server" stuff $'\003'

echo "Sent stop command to all kinovagaze screens, waiting 3 seconds..."
sleep 3
echo "The following screens are still running:"
screen -ls
