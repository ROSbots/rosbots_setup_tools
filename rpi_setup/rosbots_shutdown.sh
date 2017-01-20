#export ROSBOTS_HOME=/home/pi
#export ROSBOTS_WS_PATH=/home/pi/ros_catkin_ws
. ${ROSBOTS_WS_PATH}/build/opt/ros/kinetic/setup.sh

export PYTHONPATH="${ROSBOTS_HOME}/lib/python:${PYTHONPATH}"

echo "Starting rosbots_shutdown script" >> ${ROSBOTS_HOME}/roscore.log


#killall nodes
for i in $( rosnode list ); do
    echo "Killing node ${i}" >> ${ROSBOTS_HOME}/roscore.log
    rosnode kill $i;
done

#stop roscore
echo "Killing roscore" >> ${ROSBOTS_HOME}/roscore.log
killall roscore
echo "Done with shutdown script" >> ${ROSBOTS_HOME}/roscore.log
