#!/bin/bash
# Create /dev/arduino inside container pointing to host device
if [ -n "$ARDUINO_DEV" ]; then
  ln -sf "$ARDUINO_DEV" /dev/arduino
fi

# Start bash / ROS
exec /bin/bash

