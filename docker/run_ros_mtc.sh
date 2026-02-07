#!/bin/bash
# run_ros_mtc.sh

# allow X11 forwarding for GUI
xhost +local:docker

# Start the container, mapping the real device
export ARDUINO_DEV=$(readlink -f /dev/arduino)
docker compose run ros_mtc


