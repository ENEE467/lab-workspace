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

# Some settings for the lab camera
printf "\n\
v4l2-ctl -d /dev/video0 --set-ctrl focus_automatic_continuous=0\n\
v4l2-ctl -d /dev/video0 --set-ctrl focus_absolute=0\n\
v4l2-ctl -d /dev/video0 --set-ctrl auto_exposure=1\n\
v4l2-ctl -d /dev/video0 --set-ctrl exposure_dynamic_framerate=0\n" >> ~/.bashrc

cd $ROS_WS

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
echo "export ROS_WS=$ROS_WS" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

printf "\nSetup complete, workspace is now ready to use! \n"
