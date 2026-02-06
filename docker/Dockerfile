# Use Ubuntu 20.04 with ROS Noetic
FROM ros:noetic-ros-core-focal

# Install necessary packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    tar \
    qt5-default \
    qtbase5-dev \
    qtbase5-dev-tools \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python \
    python3-pip \
    xterm \
    qtdeclarative5-dev \
    qml-module-qtquick-controls \
    qml-module-qtquick-controls2 \
    qml-module-qtquick-layouts \
    qml-module-qtquick-window2 \
    qml-module-qtquick2 \
    qml-module-qtgraphicaleffects \
    alsa-utils \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Install Arduino IDE for AVR GCC >= 7
# ------------------------------
WORKDIR /opt
RUN wget https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz && \
    tar -xf arduino-1.8.19-linux64.tar.xz && \
    rm arduino-1.8.19-linux64.tar.xz

# Add AVR toolchain to PATH
ENV PATH="/opt/arduino-1.8.19/hardware/tools/avr/bin:${PATH}"

# ------------------------------
# Setup Catkin workspace
# ------------------------------
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Initialize workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source ROS and workspace automatically in interactive shells
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]

