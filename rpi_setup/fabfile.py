#
# This file is part of ROSbots Setup Tools.
#
# Copyright
#
#     Copyright (C) 2017 Jack Pien <jack@rosbots.com>
#
# License
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU Lesser General Public License as published
#     by the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU Lesser General Public License for more details at
#     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
#
# Documentation
#
#     http://www.rosbots.com
#

import os
import datetime as dt
import random
import time

from fabric.api import *
import fabric.contrib.files as fabfiles
from fabric.utils import fastprint
#env.hosts = ["localhost"]
env.user = 'pi'    
env.shell = '/bin/bash -l -c' 

is_debug = False
    
def _get_input(msg, force_need_query=False):
    global is_debug

    if is_debug or force_need_query:
        val = raw_input(msg + "\n")
        return val
    else:
        return ""

def _fp(msg):
    fastprint(msg + "\n")

def _pp(msg):
    """ 
    Print then pause
    """
    global is_debug
    
    _fp(msg)
    
    if is_debug:
        programPause = _get_input("Press the <ENTER> key to continue...")


WS_DIR = "/ros_catkin_ws"
INSTALL_DIR = WS_DIR + "/build/opt/ros/kinetic"

def main_setup_only_rosbots_components():
    step_7_setup_ros_rosbots_packages()
    step_8_setup_mcu_uno_support()
    step_9_setup_mcu_uno_support_part_2()

def main_setup_ros_opencv_for_rosbots():
    step_1_setup_ros_for_pi()
    step_2_setup_ros_robot_packages()
    #step_3_setup_ros_rosbots_packages()
    step_4_setup_opencv_for_pi()
    step_5_setup_ros_robot_image_common_package()
    step_6_setup_ros_robot_vision_packages()

    step_7_setup_ros_rosbots_packages()
    step_8_setup_mcu_uno_support()
    step_9_setup_mcu_uno_support_part_2()


def main_setup_ros_opencv():
    step_1_setup_ros_for_pi()
    step_2_setup_ros_robot_packages()
    
    step_4_setup_opencv_for_pi()
    step_5_setup_ros_robot_image_common_package()
    step_6_setup_ros_robot_vision_packages()
    
def helloworld():
    run("ls -la")

    #with cd("~"):
    #    home_path = run("pwd")
    #    ws_dir = home_path + WS_DIR

    #    put("./rosbots_service_template.bash", "~/rosbots_template")
    #    run("cat rosbots_template | sed 's/_TEMPLATE_HOME/" + home_path.replace("/", "\/") + "/' | sed 's/_TEMPLATE_WS_PATH/" + ws_dir.replace("/", "\/") + "/' > rosbots")

def how_to_test_rosbots_python_scripts():
    _fp("Say you wrote a rosbots python script called foo.py. (1) chmod +x foo.py. (2) scp it over to the /home/pi/ros_catkin_ws/build/opt/ros/kinetic/share/rosbots_driver. (3) from remote machine 'rosrun rosbots_driver foo.py'")

def push_test_ros_script(path_fn=None):
    if path_fn == None:
        _fp("\nERROR\nPlease specify local ROS script name")
        _fp("$ fab push_test_ros_script:<script>")
        return

    fn = path_fn.split("/")[-1]
    remote_path = "/home/pi/ros_catkin_ws/build/opt/ros/kinetic/share"
    ros_pkg_name = "rosbots_driver"
    _fp("Pushing " + path_fn + " to remote location: " +
        remote_path + "/" + ros_pkg_name)

    put(path_fn, remote_path + "/" + ros_pkg_name)
    run("chmod +x " + remote_path + "/" + ros_pkg_name + "/" + fn)

    #open_shell("rosrun " + ros_pkg_name + " " + fn)
    run("sudo su -c 'source /home/pi/ros_catkin_ws/build/opt/ros/kinetic/setup.bash && export PYTHONPATH=/home/pi/lib/python:${PYTHONPATH} && rosrun " + ros_pkg_name + " " + fn + "'")

def push_test_rosbots_motor_driver_script():
    run("echo 'Starting...'")

    home_path = run("pwd")
    rosbots_startup_fn = "rosbots_startup.sh"
    local_md_dir = "../../ros_ws/src/rosbots_driver/scripts/rosbots_driver"
    remote_md_dir = "/home/pi/ros_catkin_ws/build/opt/ros/kinetic/lib/rosbots_driver"
    md_fn = "motor_driver.py"
    rosnode_name = "/motor_driver"
    

    # Kill current motor_driver node
    old_shell = env.shell
    env.shell = '/bin/bash -l -c -i'
    if run("rosnode list | grep -i " + rosnode_name, warn_only=True).succeeded:
        _fp("Killing current " + rosnode_name + " rosnode")
    
        run("rosnode kill `rosnode list | grep -i " + rosnode_name + "`")
        #_fp(actual_name)
        #run("rosnode kill " + rosnode_name)

    env.shell = old_shell

    # Push new startup script
    if False:
        put("./rosbots_startup.sh", "~/rosbots_startup.sh")
        run("chmod +x ~/rosbots_startup.sh")

    # Push the new motor driver file
    if fabfiles.exists(remote_md_dir + "/" + md_fn) == False:
        _fp("No remote " + md_fn + " found!!!  Quitting")
        return        
    else:
        put(local_md_dir + "/" + md_fn, remote_md_dir + "/" + md_fn)
        run("rm " + remote_md_dir + "/" + md_fn + "c", warn_only=True)

    # Start the rosbots startup script
    sudo("export ROSBOTS_HOME=/home/pi; export ROSBOTS_WS_PATH=/home/pi/ros_catkin_ws; " + home_path + "/" + rosbots_startup_fn)

    old_shell = env.shell
    env.shell = '/bin/bash -l -c -i'
    _fp("List of running ros nodes")
    run("rosnode list")
    env.shell = old_shell
    

def setup_wifi_on_pi():
    supplicant_fn = "/etc/wpa_supplicant/wpa_supplicant.conf"
    run("echo 'Starting...'")
    
    #if run("grep 'country=GB' " + supplicant_fn, warn_only=True).succeeded:
    #    pass
    #else:
    #    _fp("")
    #    _pp("You should probably set 'country=US' in your supplicant file " + \
    #        supplicant_fn + " when you get a chance...")

    wifi_reg_domain = _get_input("What is your country's wifi regulatory domain (ISO 3166 alpha2 country code, ie 'US')?", force_need_query=True)
    _fp(wifi_reg_domain)

    ssid_name = _get_input("What is the SSID?", force_need_query=True)
    _fp(ssid_name)

    if sudo("grep 'ssid=\"" + ssid_name + "\"' " + supplicant_fn, \
           warn_only=True).succeeded:
        _fp("This SSID is already set up")
    else:
        wpa_pwd = _get_input("What is the WPA pwd?", force_need_query=True)
        _fp(wpa_pwd)
        name = _get_input("What do you want to name this network?", force_need_query=True)
        _fp(name)

        _fp("Adding the network you specified into " + supplicant_fn)
        network_config = "country=" + wifi_reg_domain + "\n" + \
                         "\n\n" + \
                         "network={\n" + \
                         "    ssid=\"" + ssid_name + "\"\n" + \
                         "    psk=\"" + wpa_pwd + "\"\n" + \
                         "    id_str=\"" + name + "\"\n" + \
                         "}\n"
        sudo("cp " + supplicant_fn + " " + supplicant_fn + ".old")
        sudo("echo '" + network_config + "' >> " + supplicant_fn)

    _fp("To get IP address of Pi, from a linux system - 'arp -a'")

def step_8_setup_mcu_uno_support():
    _pp("Plug in the UNO board to the RPi's USB port")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"
    rosbots_path = git_path + "/rosbots_driver"
    pio_path = rosbots_path + "/platformio/rosbots_firmware"

    rosserial_path = git_path + "/rosserial"
    ws_dir = home_path + "/rosbots_catkin_ws" 
    install_dir = home_path + INSTALL_DIR
    main_ros_ws_dir = home_path + WS_DIR

    # Just download, we'll build it isolated later
    #_setup_ros_other_packages("actionlib_msgs", run_rosdep=False)
    _setup_ros_other_packages("nav_msgs", run_rosdep=False)

    # Need nav_msgs compiled
    with cd(main_ros_ws_dir):
        #run("./src/catkin/bin/catkin_make_isolated --pkg rosbots_driver --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        
        old_shell = env.shell
        env.shell = '/bin/bash -l -c -i'
        #run(main_ros_ws_dir + "/src/catkin/bin/catkin_make -j1 --pkg nav_msgs")
        #run(main_ros_ws_dir + "/src/catkin/bin/catkin_make install -j1 --pkg nav_msgs")
        #run("./src/catkin/bin/catkin_make_isolated --pkg actionlib_msgs --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        run("./src/catkin/bin/catkin_make_isolated --pkg nav_msgs --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        env.shell = old_shell

    # Old pip causes incompleteread importerror
    sudo("easy_install --upgrade pip")
    
    # So we can access USB serial port
    sudo("usermod -a -G dialout pi")

    # Some requirements
    sudo("pip install -U testresources")
    
    sudo("pip install -U platformio")

    sudo("pip install -U backports.functools_lru_cache")

    _fp("=============")
    _pp("If this is the first time running setup, the next step will most likely fail since you need a reboot to enable the UNO drivers. If it fails, reboot and run this step again.")
    _fp("=============\n")



def step_9_setup_mcu_uno_support_part_2():
    _pp("Plug in the UNO board to the RPi's USB port")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"
    rosbots_path = git_path + "/rosbots_driver"
    pio_path = rosbots_path + "/platformio/rosbots_firmware"

    rosserial_path = git_path + "/rosserial"
    ws_dir = home_path + "/rosbots_catkin_ws" 
    install_dir = home_path + INSTALL_DIR
    main_ros_ws_dir = home_path + WS_DIR
    
    with cd(pio_path):
        run("platformio run -e uno -t upload")

    # We need diagnostic_msgs, but just download, we'll compile
    # it on our own
    _setup_ros_other_packages("diagnostic_msgs", run_rosdep=False)

    # Download and install rosserial
    if not fabfiles.exists(rosserial_path):
        with cd(git_path):
            run("git clone https://github.com/ros-drivers/rosserial.git")
            
        _fp("Creating symbolic link to main ros workspace")
        with cd(ws_dir + "/src"):
            if fabfiles.exists("rosserial"):
                run("rm rosserial")
            run("ln -s " + rosserial_path) 
    else:
        _fp("Found rosserial repo, just fetching top and rebasing")
        with cd(rosserial_path):
            run("git fetch origin")
            run("git rebase origin/jade-devel")

    with cd(ws_dir):
        #run("./src/catkin/bin/catkin_make_isolated --pkg rosbots_driver --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        
        old_shell = env.shell
        env.shell = '/bin/bash -l -c -i'
        run(main_ros_ws_dir + "/src/catkin/bin/catkin_make -j1")
        run(main_ros_ws_dir + "/src/catkin/bin/catkin_make install -j1")
        env.shell = old_shell

    # Need diagnostic_msgs which rosserial_python needs
    # catkin_make_isolated --pkg diagnostic_msgs --install -DCMAKE_BUILD_TYPE=Release --install-space /home/pi/ros_catkin_ws/build/opt/ros/kinetic
    subpackage = "diagnostic_msgs"
    with cd(main_ros_ws_dir):
        run("./src/catkin/bin/catkin_make_isolated --pkg " + subpackage + " --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j1")
            
    #Update pip if necessary
    sudo("easy_install --upgrade pip")
    
    # Rerun the init script
    sudo("systemctl stop rosbots")
    sudo("systemctl start rosbots")

    

def step_5_setup_ros_robot_image_common_package():
    home_path = run("pwd")
    install_dir = home_path + INSTALL_DIR
    main_ros_ws_dir = home_path + WS_DIR

    _pp("Usually done after you set up OpenCV and the other robot and rosbot packages.  This mainly sets up image_transport.")
    _setup_ros_other_packages("image_common")

def step_2_setup_ros_robot_packages():
    
    _pp("After you successfully install ros_com stuff, install some others. This installs geometry_msgs needed for Twist among other types of basic telemetry messages.")

    _setup_ros_other_packages("geometry_msgs")
    _setup_ros_other_packages("teleop_twist_keyboard")
    

def _setup_ros_packages_from_git(ros_package_name, git_url, subpackage_list):
    run("echo 'Starting...'")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"
    ros_package_path = git_path + "/" + ros_package_name #"/rosbots"
    ws_dir = home_path + WS_DIR
    install_dir = home_path + INSTALL_DIR

    _fp("Do we need to create gitspace folder?")
    if not fabfiles.exists(git_path):
        run("mkdir " + git_path)

    _fp("Do we need to git clone the repo?")
    if not fabfiles.exists(ros_package_path):
        _fp("Did not find " + ros_package_name + " repo, cloning...")
        with cd(git_path):
            run("git clone " + git_url)
            
        _fp("Creating symbolic link to main ros workspace")
        with cd(ws_dir + "/src"):
            if fabfiles.exists(ros_package_name):
                run("rm " + ros_package_name)
            run("ln -s " + ros_package_path)
    else:
        #_fp("Found the repo, just fetching top and rebasing")
        #with cd(ros_package_path):
        #    run("git fetch origin")
        #    run("git rebase origin/master")
        _pp("Found the repo, not doing anything - feel free to git fetch and rebase manually")

    for subpackage in subpackage_list:
        _fp("Compiling " + subpackage + "...")
        with cd(ws_dir):
            run("./src/catkin/bin/catkin_make_isolated --pkg " + subpackage + " --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j1")

def step_6_setup_ros_robot_vision_packages():
    _fp("Usually done after you set up OpenCV and the other robot and rosbot packages")
    _pp("This sets up mainly cv_bridge so we can pass CV image messages around. Setting up from github instead of rosinstall because rosinstall will pull in OpenCV automatically and you should have already built it from source.")
    _setup_ros_packages_from_git("vision_opencv", \
                "https://github.com/ros-perception/vision_opencv.git", \
                ["cv_bridge", "image_geometry", "vision_opencv"])

def step_7_setup_ros_rosbots_packages():
    run("echo 'Starting...'")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"
    rosbots_path = git_path + "/rosbots_driver"
    ws_dir = home_path + "/rosbots_catkin_ws" # home_path + WS_DIR
    install_dir = home_path + INSTALL_DIR
    main_ros_ws_dir = home_path + WS_DIR

    
    # Just download tf and geometry2, which includes tf2.
    # We'll compile it ourselves later
    _setup_ros_other_packages("geometry", run_rosdep=False)
    _setup_ros_other_packages("geometry2", run_rosdep=False)
    
    # Need tf and tf2 compiled
    with cd(main_ros_ws_dir):
        #run("./src/catkin/bin/catkin_make_isolated --pkg rosbots_driver --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        old_shell = env.shell
        env.shell = '/bin/bash -l -c -i'
        package_list = [
            "angles", "actionlib_msgs", "actionlib", "tf2_msgs", "tf2", "tf2_py", "tf2_ros", "tf"]
        for pkg in package_list:
            run("./src/catkin/bin/catkin_make_isolated --pkg " + pkg + " --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        env.shell = old_shell

    sudo("apt-get install -y python-pip")
    sudo("pip install picamera")

    # Create a separate rosbots_catkin_ws outside of core ROS
    if not fabfiles.exists(ws_dir):
        _fp("Need to create and init rosbots catkin workspace")
        run("mkdir -p " + ws_dir + "/src")
        
        old_shell = env.shell
        env.shell = '/bin/bash -l -c -i'
        with cd(ws_dir + "/src"):
            run(main_ros_ws_dir + "/src/catkin/bin/catkin_init_workspace")

        with cd(ws_dir):
            run(main_ros_ws_dir + "/src/catkin/bin/catkin_make")
            run(main_ros_ws_dir + "/src/catkin/bin/catkin_make install")
        env.shell = old_shell
        
        src_cmd = "source " + ws_dir + "/devel/setup.bash"
        if run("grep '" + src_cmd + "' ~/.bashrc", warn_only=True).succeeded:
            _fp("Sourcing of ROSbots catkin ws env setup.bash is already in your bashrc")
        else:
            _pp("Going to add ROSbots catkin ws source setup into your bashrc")
            run("echo '" + src_cmd + "\n' >> ~/.bashrc")
    
    if not fabfiles.exists(git_path):
        _fp("Did not find rosbots repo, cloning...")
        run("mkdir " + git_path)

    if not fabfiles.exists(rosbots_path):
        with cd(git_path):
            run("git clone https://github.com/ROSbots/rosbots_driver.git")
            
        _fp("Creating symbolic link to main ros workspace")
        with cd(ws_dir + "/src"):
            if fabfiles.exists("rosbots_driver"):
                run("rm rosbots_driver")
            run("ln -s " + rosbots_path) 
    else:
        _fp("Found rosbots repo, just fetching top and rebasing")
        with cd(rosbots_path):
            run("git fetch origin")
            run("git rebase origin/master")

    with cd(ws_dir):
        #run("./src/catkin/bin/catkin_make_isolated --pkg rosbots_driver --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")
        
        old_shell = env.shell
        env.shell = '/bin/bash -l -c -i'
        run(main_ros_ws_dir + "/src/catkin/bin/catkin_make")
        run(main_ros_ws_dir + "/src/catkin/bin/catkin_make install")
        env.shell = old_shell

    # Installing RPIO DMA PWM library
    with cd(git_path):
        # Don't install RPIO library. May be causing non-deterministic
        # kernel panic when used.
        #if not fabfiles.exists("RPIO"):
        if False:        
            _pp("Did not find RPIO library so downloading and setting up")

            # Old library does not support RPi 3
            #run("git clone https://github.com/metachris/RPIO.git --branch v2 --single-branch")
            #run("git clone https://github.com/limuxy/RPIO.git")
            run("git clone https://github.com/ROSbots/RPIO.git --branch v2_branch --single-branch")
            with cd("RPIO"):
                run("python setup.py build")
                _pp("Did build complete for RPIO?")
                run("mkdir -p " + home_path + "/lib/python")
                run("export PYTHONPATH=" + home_path + "/lib/python; python setup.py -v install --home " + home_path)

                _pp("Did RPIO install correctly into " + home_path + "?")
                
    # Update with newest bashrc for rosbots
    put("./sourceme_rosbots.bash", "~/")
                
    # Rerun the init script
    sudo("systemctl stop rosbots")
    sudo("systemctl start rosbots")
    

def _setup_ros_other_packages(rospkg, run_rosdep=True):
    run("echo 'Starting...'")

    home_path = run("pwd")
    ws_dir = home_path + WS_DIR
    if not fabfiles.exists(ws_dir):
        _fp("ROS Workspace not found - run the main set up first")
        return

    with cd(ws_dir):
        ts = str(time.time()).split(".")[0]
        fn = "kinetic-custom_" + str(ts) + "_ros.rosinstall"
        run("rosinstall_generator " + rospkg + " --rosdistro kinetic --deps --wet-only --tar > " + fn)

        run("cat " + fn)

        _pp("Did rosinstall generator create the install file correctly? If so, we're going to merge and update the workspace. (If there are duplicate packages, hit DELETE and REPLACE!)")
        
        run("wstool merge -y -t src " + fn)

        _pp("Did the wstool merge correctly?  If so, we are going to update on the install file for the workspace.")
        
        run("wstool update --delete-changed-uris -t src")

        
        _pp("Did the wstool update correctly?  If so, we are going to update dependencies.")

        if run_rosdep:
            run("rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie")

            _pp("Did the dependencies update ok?  If so, let's compile the new packages.")

            run("./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space " + home_path + INSTALL_DIR + " -j1")


def step_4_setup_opencv_for_pi():
    """
To build this in a Docker container:

run:
docker run -it --name rosbots_build rosbots-raspbian:lite /bin/bash

apt-get update; apt-get -y upgrade
apt-get install -y libgdk-pixbuf2.0-dev libpango1.0-dev libcairo2-dev 
apt-get install -y libgtk2.0-dev 
apt-get install -y build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libatlas-base-dev gfortran python2.7-dev python3-dev libavutil-dev python-pip git
pip install numpy
mkdir -p /home/pi/gitspace
cd /home/pi/gitspace
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout -b 3.4.6_branch tags/3.4.6
cd ../
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout -b 3.4.6_branch tags/3.4.6
cd ../opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_EXTRA_MODULES_PATH=/home/pi/gitspace/opencv_contrib/modules -D BUILD_EXAMPLES=ON ..
make -j4

On physcial RPi:
cd /home/pi/gitspace
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout -b 3.4.6_branch tags/3.4.6
cd ../
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout -b 3.4.6_branch tags/3.4.6
copy /home/pi/gitspace/opencv/build to /home/pi/gitspace/opencv
sudo apt-get update; sudo apt-get -y upgrade
sudo apt-get install -y libgdk-pixbuf2.0-dev libpango1.0-dev libcairo2-dev 
sudo apt-get install -y libgtk2.0-dev 
sudo apt-get install -y build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libatlas-base-dev gfortran python2.7-dev python3-dev libavutil-dev python-pip git
then cd /home/pi/gitspace/opencv/build,
'sudo make install/fast', 'sudo ldconfig'

    """
    _pp("Roughly following http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/")

    #_fp("Update system first")
    #sudo("apt-get update")
    #sudo("apt-get -y upgrade")

    _fp("Installing dependencies for OpenCV")

    # Need to install libgtk2.0 first in Stretch?!?
    sudo("apt-get install -y libgtk2.0-dev")
    sudo("apt-get install -y build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libatlas-base-dev gfortran python2.7-dev python3-dev")

    # Needed for web_video_server and perhaps help with OpenCV support as well
    sudo("apt-get install -y libavutil-dev")

    sudo("apt-get install -y python-pip")
    sudo("sudo pip install numpy")
    sudo("sudo pip install --upgrade numpy")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"

    _fp("Do we need to create gitspace folder?")
    if not fabfiles.exists(git_path):
        run("mkdir " + git_path)

    _fp("Git cloning OpenCV if need be")
    if not fabfiles.exists(git_path + "/opencv"):
        with cd(git_path):
            run("git clone https://github.com/opencv/opencv.git")
        with cd(git_path + "/opencv"):
            run("git tag -l")
            #_pp("We are compiling 3.4.1 - make sure this is the latest from the tag list printed above")
            #run("git checkout -b 3.4.1_branch tags/3.4.1")
            _pp("We are compiling 3.4.6 - make sure this is the latest from the tag list printed above")
            run("git checkout -b 3.4.6_branch tags/3.4.6")

    opencv_contrib_path = git_path + "/opencv_contrib"
    if not fabfiles.exists(opencv_contrib_path):
        with cd(git_path):
            run("git clone https://github.com/opencv/opencv_contrib.git")
        with cd(opencv_contrib_path):
            run("git tag -l")
            #_pp("We are compiling 3.4.1 - make sure this is the latest from the tag list printed above")
            #run("git checkout -b 3.4.1_branch tags/3.4.1")
            _pp("We are compiling 3.4.6 - make sure this is the latest from the tag list printed above")
            run("git checkout -b 3.4.6_branch tags/3.4.6")
            
    _fp("Setting up OpenCV cmake if need be")
    if not fabfiles.exists(git_path + "/opencv/build"):
        with cd(git_path + "/opencv"):
            run("mkdir build")

        # Set up compile
        with cd(git_path + "/opencv/build"):
            run("cmake -D CMAKE_BUILD_TYPE=RELEASE " + \
                "-D CMAKE_INSTALL_PREFIX=/usr/local " + \
                "-D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_ENABLE_NONFREE=ON " + \
                "-D OPENCV_EXTRA_MODULES_PATH=" + \
                opencv_contrib_path + "/modules " + \
                "-D BUILD_EXAMPLES=ON ..")

    # Compile
    _fp("Compiling OpenCV...")
    with cd(git_path + "/opencv/build"):
        run("make -j1")
        sudo("make install")
        sudo("ldconfig")

def step_x_setup_ros_for_ubuntu_mate_pi():
    run("echo 'Roughly following http://wiki.ros.org/kinetic/Installation/Ubuntu'")
    _pp("* If you need to do raspi-config stuff, CTRL-C out and do that before running this script")
    
    # Setup ROS Repositories
    if not fabfiles.exists("/etc/apt/sources.list.d/ros-latest.list"):
        sudo("apt-get update")
        sudo("sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'")
        sudo("apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116") #apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116")
        sudo("apt-get update")
        sudo("apt-get -y upgrade")
    else:
        _fp("ros-lastest.list already exists... skipping set up")
        sudo("apt-get update")
        sudo("apt-get -y upgrade")

    sudo("apt-get install -y ros-kinetic-ros-base")
    

def step_1_setup_ros_for_pi():
    """
To compile ros2 on in a Docker Raspbian container:

docker run -it --name rosbots_ros2_build rosbots-raspbian:lite /bin/bash

update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8
export LC_ALL=en_GB.UTF-8
apt update && apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python-rosdep \
  libxml2-dev \
  libxslt1-dev \
  wget
apt install -y virtualenvwrapper
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh
mkvirtualenv py_3 --python=/usr/bin/python3
pip install -U argcomplete catkin_pkg colcon-common-extensions coverage empy flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes lark-parser mock nose pep8 pydocstyle pyparsing setuptools vcstool \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  lxml \
  rosdep
apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
mkdir -p /home/pi/ros2_ws/src
cd /home/pi/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import src < ros2.repos
(sudo) rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro crystal -y -r --os=debian:stretch
pip install -U lark-parser
colcon build --symlink-install --packages-skip ros1_bridge --packages-ignore qt_gui_cpp rqt_gui_cpp

On the physical RPi, do all steps above except the colcon build step
then, docker cp /home/pi/ros2_ws/install ./build ./log to the physical RPi /home/pi/ros2_ws

Install python3.6

Change these scripts to use the python3 in the correct virtualenv directory
install/ros2cli/bin/_ros2_daemon:#!/root/.virtualenvs/py_3/bin/python3
install/ros2cli/bin/ros2:#!/root/.virtualenvs/py_3/bin/python3

. ~/ros2_ws/install/local_setup.bash (or setup.bash)
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener

    """
    global WS_DIR
    global INSTALL_DIR

    run("echo 'Roughly following http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi'")

    _fp("Set up / compile ROS on Rasbian Jessie Lite 2016-05-27")
    _pp("* If you need to do raspi-config stuff, CTRL-C out and do that before running this script")
    
    # Setup ROS Repositories
    if not fabfiles.exists("/etc/apt/sources.list.d/ros-latest.list"):

        # Raspbian Stretch does not have dirmngr installed by default. This
        # is needed for apt-key
        sudo("apt-get update")
        sudo("apt-get -y install dirmngr")
        sudo("sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'")
        sudo("sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116") #apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116")
        sudo("apt-get update")
        sudo("apt-get -y upgrade")
    else:
        _fp("ros-lastest.list already exists... skipping set up")
        sudo("apt-get update")
        sudo("apt-get -y upgrade")
    

    # Install Bootstrap Dependencies
    sudo("apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake")
    
    # Initializing rosdep
    if not fabfiles.exists("/etc/ros/rosdep/sources.list.d/20-default.list"):
        sudo("rosdep init")
        run("rosdep update")

    home_path = run("pwd")
    ws_dir = home_path + WS_DIR

    # Create catkin workspace
    if not fabfiles.exists(ws_dir):
        run("mkdir -p " + ws_dir)
        
    # Compile
    with cd(ws_dir):
        if not fabfiles.exists("kinetic-ros_comm-wet.rosinstall"):
            run("rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall")
        
        if not fabfiles.exists("src"):
            _fp("The following wstool downloads the source code needed")
            _pp("If wstool init fails or is interrupted, you can resume the download by running:\n wstool update -j 2 -t src\n BTW, the -j 2 option downloads 2 packages in parallel")
        
            run("wstool init -j 2 src kinetic-ros_comm-wet.rosinstall")
        else:
            _pp("Looks like you had already tried 'wstool init...', so continuing with 'wstool update...'")
            run("wstool update --delete-changed-uris -j 2 -t src")

        rval = _get_input("Did wstool download everything ok?\n(NO to quit & resolve, ENTER to continue)")
        if rval == "NO":
            return

        # Resolve dependencies
        run("rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:jessie")

        install_dir = home_path + INSTALL_DIR 
        
        _fp("All dependencies have been resolved, going to start compiling and install into: " + install_dir)
        
        if not fabfiles.exists(install_dir):
            run("mkdir -p " + install_dir)

        rval = _get_input("Continue with compile or skip? SKIP to skip compile, ENTER to continue...")
        if rval != "SKIP":
            run("./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space " + install_dir + " -j2")


            _rval = _get_input("Did the compile succeed?\n(NO to quit and fix, ENTER to continue)")
            if rval == "NO":
                return

        
        src_cmd = "source " + install_dir + "/setup.bash"
        if run("grep '" + src_cmd + "' ~/.bashrc", warn_only=True).succeeded:
            _fp("Sourcing of ROS env setup is already in your bashrc")
        else:
            _pp("Going to add ROS source setup into your bashrc")
            run("echo '" + src_cmd + "\n' >> ~/.bashrc")
            run("echo 'export ROSBOTS_MASTER=1\n' >> ~/.bashrc")

            # Add some custom python library paths
            run("echo 'export PYTHONPATH=/home/pi/lib/python:${PYTHONPATH}\n' >> ~/.bashrc") 

            # Add other setups for rosbots
            put("./sourceme_rosbots.bash", "~/")
            run("echo 'source ~/sourceme_rosbots.bash' >> ~/.bashrc")





    # Create a separate rosbots_catkin_ws outside of core ROS
    rosbots_ws_dir = home_path + "/rosbots_catkin_ws"
    if not fabfiles.exists(rosbots_ws_dir):
        _fp("Need to create and init rosbots catkin workspace")
        run("mkdir -p " + rosbots_ws_dir + "/src")
        
        old_shell = env.shell
        env.shell = '/bin/bash -l -c -i'
        with cd(rosbots_ws_dir + "/src"):
            run(ws_dir + "/src/catkin/bin/catkin_init_workspace")

        with cd(rosbots_ws_dir):
            run(ws_dir + "/src/catkin/bin/catkin_make")
            run(ws_dir + "/src/catkin/bin/catkin_make install")
        env.shell = old_shell
        
        src_cmd = "source " + rosbots_ws_dir + "/devel/setup.bash"
        if run("grep '" + src_cmd + "' ~/.bashrc", warn_only=True).succeeded:
            _fp("Sourcing of ROSbots catkin ws env setup.bash is already in your bashrc")
        else:
            _pp("Going to add ROSbots catkin ws source setup into your bashrc")
            run("echo '" + src_cmd + "\n' >> ~/.bashrc")


            
            

    _pp("All ROS components should be compiled and installed. Going to set up init.d to run ROSBots as a service.")

    # Copy over the rosbots init script - which is kicked off by the init.d
    # service framework
    put("./rosbots_startup.sh", "~/rosbots_startup.sh")
    run("chmod +x ~/rosbots_startup.sh")
    put("./rosbots_shutdown.sh", "~/rosbots_shutdown.sh")
    run("chmod +x ~/rosbots_shutdown.sh")

    # Set up and install the init.d service which will fork and call
    # the rosbots startup script above
    put("./rosbots_service_template.bash", "~/rosbots_template")
    run("cat rosbots_template | sed 's/_TEMPLATE_HOME/" + home_path.replace("/", "\/") + "/' | sed 's/_TEMPLATE_WS_PATH/" + ws_dir.replace("/", "\/") + "/' > rosbots")
    run("rm rosbots_template")

    sudo("mv rosbots /etc/init.d/")
    sudo("chown root:root /etc/init.d/rosbots")
    sudo("chmod 755 /etc/init.d/rosbots")
    sudo("update-rc.d rosbots defaults")
    sudo("systemctl daemon-reload")
    sudo("systemctl stop rosbots")
    sudo("systemctl start rosbots")

    _fp("To get IP address of Pi, from a linux system - 'arp -a'")
    
    _fp("Done...")
    
