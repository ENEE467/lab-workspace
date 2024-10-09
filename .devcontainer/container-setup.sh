#!/bin/bash

set -e

source /opt/ros/$ROS_DISTRO/setup.bash

# Enable permission for webcam
sudo usermod -aG video $USER

# Enable permission for display
echo $'\nexport XAUTHORITY=$(ls /run/user/$UID/.mutter-*)' >> ~/.bashrc

# Check if the ROS workspace exists
if [ ! -d $ROS_WS ]
then
  printf "Setup failed: ROS workspace directory not found\n"
  exit 1
fi

cd $ROS_WS

# Install matplot++ dependencies
sudo apt update && sudo apt install -y \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  gnuplot \
  zlib1g-dev \
  libblas-dev \
  liblapack-dev

# Install matplot++
if [ ! -d matplotplusplus ]
then
  git clone https://github.com/alandefreitas/matplotplusplus.git
fi

cd matplotplusplus

if [ -d build ]
then
  rm -rf build
fi

# Build the library and install it
cmake --preset=system
cmake --build --preset=system
sudo cmake --install build/system

# Move to workspace root directory and clean up matplot++ source files
cd $ROS_WS && rm -rf matplotplusplus

# Install some ROS Packages
sudo apt update && sudo apt install -y --no-install-recommends \
  ros-humble-usb-cam \
  ros-humble-aruco-opencv

# Install package dependencies
rosdep update && rosdep install --from-paths src -y --ignore-src

# Clean up previous workspace build artifacts if they exist
if [ -d build ]
then
  rm -r build
fi

if [ -d install ]
then
  rm -r install
fi

if [ -d log ]
then
  rm -r log
fi

# Build the workspace
colcon build --symlink-install

# Always source the workspace and start a new session in the workspace root.
echo "if [ -f $ROS_WS/install/setup.bash ]; then source $ROS_WS/install/setup.bash; fi" >> ~/.bashrc
echo "if [ -d $ROS_WS ]; then cd $ROS_WS; fi" >> ~/.bashrc

printf "\nSetup complete, workspace is now ready to use! \n"
