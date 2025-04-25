##This is the ROS 2 workspace used for the Open Source Thunniform Robot project
Check us out and find more documentation at oshe.io!

#Important Files:
*start.bash -- This is an attempt at a startup script so the ROS2 environment could be started on boot with systemd. We never got this working quite right
*autostartlunch.xml -- This starts all of the required nodes for wireless operation
*minimalstartlaunch.xml -- This starts all non-communication related nodes

#Startup
*run "colcon build" from the workspace directory
*run "source install/setup.bash" from the workspace directory
*run "ros2 launch _________.xml" from the workspace directory to launch a preset selection of multiple nodes
*run "ros2 run py_pubsub ______" from the workspace directory to launch individual nodes 