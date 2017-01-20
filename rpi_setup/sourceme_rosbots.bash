#! /bin/bash

if [ "${ROS_MASTER_URI}" == "" ]; then
    echo "You need to source the main ROS setup.bash file first... exiting"
    return
fi

# Try to get wifi address first
my_ip="$(ifconfig wlan0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"

if  [ "${my_ip}" == "" ]; then
    my_ip="$(ifconfig eth0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"
fi

#echo ${my_ip}

export ROS_IP=${my_ip}

if [ "${ROSBOTS_MASTER}" == "1" ]
then
    rcrunning="$(ps ax | grep -i roscore | grep -i python)"
    echo ""
    echo ""
    if [ "${rcrunning}" == "" ]
    then
        echo '!! Need to run roscore - "roscore > ~/roscore.out 2>&1 &"'
        echo '...roscore should be running though - since it should be started by the init.d rosbots service which forks the rosbots_startup script'
        #nohup roscore > ~/roscore.out 2>&1 &
    else
        echo "ROSCORE already running..."
    fi
    echo "For all slaves, \"export ROS_MASTER_URI=http://${ROS_IP}:11311\""
    echo ""
else    
    echo 'As ROS slave node, you will also need to "export ROS_MASTER_URI=http://<master_ip>:11311"'
fi
