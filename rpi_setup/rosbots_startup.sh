#export ROSBOTS_HOME=/home/pi
#export ROSBOTS_WS_PATH=/home/pi/ros_catkin_ws
. ${ROSBOTS_WS_PATH}/build/opt/ros/kinetic/setup.sh

export PYTHONPATH="${ROSBOTS_HOME}/lib/python:${PYTHONPATH}"

touch ${ROSBOTS_HOME}/roscore.log
mv ${ROSBOTS_HOME}/roscore.log ${ROSBOTS_HOME}/roscore.log_old
echo "Starting rosbots_startup script" > ${ROSBOTS_HOME}/roscore.log

# Try to get wifi address first
wait_cnt=1
my_ip="$(ifconfig wlan0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"
until [ "$my_ip" != "" ]
do
    if [ $wait_cnt -eq 10 ]; then
	echo "Waiting for IP address timed out" >> ${ROSBOTS_HOME}/roscore.log
	break
    fi
    echo "Waiting ${wait_cnt} to get IP address" >> ${ROSBOTS_HOME}/roscore.log
    #aaa="$(ifconfig)"
    #echo "ifconfig - ${aaa}" >> ${ROSBOTS_HOME}/roscore.log
    sleep 3

    # Wireless IP?
    my_ip="$(ifconfig wlan0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"

    # No wireless IP, what about ethernet?
    if  [ "$my_ip" = "" ]; then
	my_ip="$(ifconfig eth0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"
    fi

    # Found IP?
    if [ "$my_ip" != "" ]; then
	break
    fi    

    wait_cnt=$((wait_cnt+1))
done

export ROS_IP=${my_ip}

# Add some more paths to make ros work
export PATH=${PATH}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games:/sbin:/usr/sbin:/bin:/usr/bin


echo "ROS IP - ${ROS_IP}" >> ${ROSBOTS_HOME}/roscore.log
echo "Starting roscore..." >> ${ROSBOTS_HOME}/roscore.log
nohup roscore >> ${ROSBOTS_HOME}/roscore.log 2>&1 &

wait_cnt=1
until fff="$(ps ax| grep -i python | grep -i rosmaster)"
do
    echo "waiting ${wait_cnt}... for rosmaster to start" >> ${ROSBOTS_HOME}/roscore.log
    sleep 3
    wait_cnt=$((wait_cnt+1))
    if [ $wait_cnt -eq 5 ]; then
	echo "Waiting for rosmaster timed out" >> ${ROSBOTS_HOME}/roscore.log
	break
    fi
done
sleep 3


mv /home/pi/rosbots_driver_motor_driver.log /home/pi/rosbots_driver_motor_driver.log.old
nohup rosrun rosbots_driver motor_driver.py >> /home/pi/rosbots_driver_motor_driver.log 2>> /home/pi/rosbots_driver_motor_driver.log &
# nohup rostopic pub -r 0.5 /twist geometry_msgs/Twist "[1,0,0]" "[2,0,0]" > /home/pi/rostopic.log 2>&1 &

echo "Done with rosbots_startup script..." >> ${ROSBOTS_HOME}/roscore.log
