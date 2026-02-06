cmake_minimum_required(VERSION 2.8.3)
project(model_train_control_arduino)

# ==========================
# Find Catkin packages
# ==========================
find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
)

# ==========================
# Messages
# ==========================
add_message_files(
  FILES
  point_command.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# ==========================
# Force C++11 for host code
# ==========================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-std=c++11)

# ==========================
# Locate Arduino AVR Toolchain automatically
# ==========================
if(NOT DEFINED ROSSERIAL_ARDUINO_TOOLCHAIN)
  set(DEFAULT_AVR_PATH "/opt/arduino-1.8.19/hardware/tools/avr")
  if(EXISTS "${DEFAULT_AVR_PATH}/bin/avr-g++")
    set(ROSSERIAL_ARDUINO_TOOLCHAIN "${DEFAULT_AVR_PATH}/avr.cmake")
    message(STATUS "Using Arduino AVR toolchain at: ${DEFAULT_AVR_PATH}")
  else()
    message(FATAL_ERROR "Arduino AVR toolchain not found at ${DEFAULT_AVR_PATH}. Please install it or set ROSSERIAL_ARDUINO_TOOLCHAIN.")
  endif()
endif()

# ==========================
# Generate ros_lib for Arduino
# ==========================
# Make sure rosserial uses C++11 for the client build
set(ROSSERIAL_CLIENT_CXX_FLAGS "-std=gnu++11")

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

# ==========================
# Configure Arduino firmware
# ==========================
rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

# ==========================
# Add Arduino client targets
# ==========================
# These targets inherit ROSSERIAL_CLIENT_CXX_FLAGS
rosserial_add_client_target(firmware mtca ALL)
rosserial_add_client_target(firmware mtca-upload)

