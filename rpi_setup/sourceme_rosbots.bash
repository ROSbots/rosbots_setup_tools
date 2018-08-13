#! /bin/bash

# Let us know if we are in a screen session
if [ -n "$STY" ]; then export PS1="(screen) $PS1"; fi

###################
# Functions
function initialize_rosbots_image() {
    cd ~
    echo "Creating new SSH keys..."
    sudo rm /etc/ssh/ssh_host_*
    sudo dpkg-reconfigure openssh-server
    echo "Set up a new password"
    passwd
    echo "Expand your filesystem, then reboot"
    sudo raspi-config
}

function setup_rosbots_code() {
    echo "Installing Python Fabric if need be..."
    sudo apt-get install -y fabric

    echo "Git cloning rosbots set up code..."
    gitdir="/home/pi/gitspace/"
    echo "cd ${gitdir}"
    cd ${gitdir}

    git clone https://github.com/ROSbots/rosbots_setup_tools.git

    cd rosbots_setup_tools/rpi_setup/
    fab -H 127.0.0.1 main_setup_only_rosbots_components

    sleep 60

    cd ~
    rosnode list
}

function upload_firmware() {
    if [ "$1" == "" ]
    then
        echo "Please specify a platformio directory (where a platform.ini lives)"
    else
        echo "Compiling then uploading $1"
        sudo systemctl stop rosbots
        cd $1
        platformio run -e uno -t upload
        sudo systemctl start rosbots
    fi
}


function update_rosbots_git_repos() {
    gitdir="/home/pi/gitspace/rosbots_driver/"
    echo "cd ${gitdir}"
    cd ${gitdir}
    echo "Fetching from origin (ie Github)..."
    git fetch origin
    echo "git stash"
    git stash
    echo "git rebase origin/master"
    git rebase origin/master
    echo "git stash pop"
    git stash pop
    echo "Unless there's a conflict, your rosbots ROS scripts should be up to date!"
    echo "---"
    echo "Updating the latest source_rosbots.bash file with latest utility funcs"
    cd "/home/pi"
    mv sourceme_rosbots.bash sourceme_rosbots.bash.old
    wget https://raw.githubusercontent.com/ROSbots/rosbots_setup_tools/master/rpi_setup/sourceme_rosbots.bash
    source .bashrc
    
    echo "Latest bash source for rosbots updated!"
    
}

function check_wifi_restart_rosbots {
    /bin/date
    if [ "$1" == "" ]
    then
        echo "Please specify a SSID to check (case-sensitive)"
    else
        ssid=$1
        conn=`sudo iwconfig wlan0 | grep -i "${ssid}"`
        wlan0ip=`sudo ifconfig wlan0 | grep -i inet`

        if [[ ("${conn}" = *"${ssid}"*) && ("$wlan0ip" != "") ]]
        then
            echo "Connection to ${ssid} is live"                              
        else            
            echo "Restarting wifi and rosbots service"

            echo "Shutting down rosbots service"
            sudo systemctl stop rosbots.service
            sleep 5
            echo "Shutting down wlan0"
            sudo ip link set wlan0 down
            sleep 5
            echo "Starting wlan0"
            sudo ip link set wlan0 up
            sleep 120
            echo "Starting rosbots.service"
            sudo systemctl start rosbots.service
        fi
    fi
}


# END FUNCTIONS
######################################################



if [ "${ROS_MASTER_URI}" == "" ]; then
    echo "You need to source the main ROS setup.bash file first... exiting"
    return
fi

# Try to get eth address first (takes priority)
my_ip="$(ifconfig eth0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"

if  [ "${my_ip}" == "" ]; then
    my_ip="$(ifconfig wlan0 | grep inet | grep -v inet6 | awk '{print $2}' | sed 's/addr://g')"
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

