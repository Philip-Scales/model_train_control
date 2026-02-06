#!/bin/bash
# run_ros_mtc.sh

# Start the container, mapping the real device
export ARDUINO_DEV=$(readlink -f /dev/arduino)
docker compose run ros_mtc


