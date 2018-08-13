#! /bin/bash

# This is what roslaunch uses as its <machine ... env-loader="/home/pi/rosbots_remote_env_source.sh" .../> param
# Don't forget to chmod +x this script
#
# !!! Need to replace ROS_IP and ROS_MASTER_URI fields manually
#

date > ros_remote.log
echo "Start" >> ros_remote.log

export ROSBOTS_HOME=/home/pi
export ROSBOTS_WS_PATH=/home/pi/ros_catkin_ws

export ROS_IP="<THIS REMOTE MACHINE'S IP ADDRESS>"
export ROS_MASTER_URI="http://MASTER_URI:11311"

. ${ROSBOTS_WS_PATH}/build/opt/ros/kinetic/setup.sh
. ${ROSBOTS_HOME}/rosbots_catkin_ws/devel/setup.sh

export ROSBOTS_MASTER=0
export PYTHONPATH="${ROSBOTS_HOME}/lib/python:${PYTHONPATH}"

# Add some more paths to make ros work
export PATH=${PATH}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games:/sbin:/usr/sbin:/bin:/usr/bin

echo "Done" >> ros_remote.log

exec "$@"
