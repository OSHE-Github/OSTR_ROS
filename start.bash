#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/OSTR/modem_ws2/install/setup.bash
sudo dtoverlay w1-gpio gpiopin=4 
ros2 launch /home/OSTR/modem_ws2/install/autostartlaunch.xml
