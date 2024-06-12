#!/bin/bash

dhclient -v
sleep 5
systemctl start isc-dhcp-server
echo none | tee /sys/class/leds/led0/triggers
echo 0 | tee /sys/class/leds/led0/brightness
sleep 1

for i in {1..10}
do
    echo 1 | tee /sys/class/leds/led0/brightness
    sleep .1
    echo 0 | tee /sys/class/leds/led0/brightness
    sleep .1
done

echo mmc0 | tee /sys/class/leds/led0/triggers
