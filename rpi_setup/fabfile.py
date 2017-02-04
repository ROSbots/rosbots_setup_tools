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

def main_setup_ros_opencv_for_rosbots():
    step_1_setup_ros_for_pi()
    step_2_setup_ros_robot_packages()
    step_3_setup_ros_rosbots_packages()
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
    
    if run("grep 'country=GB' " + supplicant_fn, warn_only=True).succeeded:
        pass
    else:
        _fp("")
        _pp("You should probably set 'country=US' in your supplicant file " + \
            supplicant_fn + " when you get a chance...")

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
        network_config = "\n\n" + \
                         "network={\n" + \
                         "    ssid=\"" + ssid_name + "\"\n" + \
                         "    psk=\"" + wpa_pwd + "\"\n" + \
                         "    id_str=\"" + name + "\"\n" + \
                         "}\n"
        sudo("cp " + supplicant_fn + " " + supplicant_fn + ".old")
        sudo("echo '" + network_config + "' >> " + supplicant_fn)

    _fp("To get IP address of Pi, from a linux system - 'arp -a'")

def step_5_setup_ros_robot_image_common_package():
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

def step_3_setup_ros_rosbots_packages():
    run("echo 'Starting...'")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"
    rosbots_path = git_path + "/rosbots_driver"
    ws_dir = home_path + "/rosbots_catkin_ws" # home_path + WS_DIR
    install_dir = home_path + INSTALL_DIR
    main_ros_ws_dir = home_path + WS_DIR

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
        if not fabfiles.exists("RPIO"):
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

    # Rerun the init script
    sudo("systemctl stop rosbots")
    sudo("systemctl start rosbots")
    

def _setup_ros_other_packages(rospkg):
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
        
        run("wstool merge -t src " + fn)

        _pp("Did the wstool merge correctly?  If so, we are going to update on the install file for the workspace.")
        
        run("wstool update -t src")

        
        _pp("Did the wstool update correctly?  If so, we are going to update dependencies.")

        run("rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie")

        _pp("Did the dependencies update ok?  If so, let's compile the new packages.")

        run("./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space " + home_path + INSTALL_DIR + " -j1")


def step_4_setup_opencv_for_pi():
    _pp("Roughly following http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/")

    _fp("Update system first")
    sudo("apt-get update")
    sudo("apt-get -y upgrade")

    _fp("Installing dependencies for OpenCV")
    sudo("apt-get install -y build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libatlas-base-dev gfortran python2.7-dev python3-dev")

    sudo("apt-get install -y python-pip")
    sudo("sudo pip install numpy")
    sudo("sudo pip install --upgrade numpy")

    home_path = run("pwd")
    git_path = home_path + "/gitspace"

    _fp("Git cloning OpenCV if need be")
    if not fabfiles.exists(git_path + "/opencv"):
        with cd(git_path):
            run("git clone https://github.com/opencv/opencv.git")
        with cd(git_path + "/opencv"):
            run("git tag -l")
            _pp("We are compiling 3.2.0 - make sure this is the latest from the tag list printed above")
            run("git checkout -b 3.2.0_branch tags/3.2.0")

    opencv_contrib_path = git_path + "/opencv_contrib"
    if not fabfiles.exists(opencv_contrib_path):
        with cd(git_path):
            run("git clone https://github.com/opencv/opencv_contrib.git")
        with cd(opencv_contrib_path):
            run("git tag -l")
            _pp("We are compiling 3.2.0 - make sure this is the latest from the tag list printed above")
            run("git checkout -b 3.2.0_branch tags/3.2.0")
            
    _fp("Setting up OpenCV cmake if need be")
    if not fabfiles.exists(git_path + "/opencv/build"):
        with cd(git_path + "/opencv"):
            run("mkdir build")

        # Set up compile
        with cd(git_path + "/opencv/build"):
            run("cmake -D CMAKE_BUILD_TYPE=RELEASE " + \
                "-D CMAKE_INSTALL_PREFIX=/usr/local " + \
                "-D INSTALL_PYTHON_EXAMPLES=ON " + \
                "-D OPENCV_EXTRA_MODULES_PATH=" + \
                opencv_contrib_path + "/modules " + \
                "-D BUILD_EXAMPLES=ON ..")

    # Compile
    _fp("Compiling OpenCV...")
    with cd(git_path + "/opencv/build"):
        run("make -j2")
        sudo("make install")
        sudo("ldconfig")

def step_1_setup_ros_for_pi():
    global WS_DIR
    global INSTALL_DIR

    run("echo 'Roughly following http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi'")

    _fp("Set up / compile ROS on Rasbian Jessie Lite 2016-05-27")
    _pp("* If you need to do raspi-config stuff, CTRL-C out and do that before running this script")
    
    # Setup ROS Repositories
    if not fabfiles.exists("/etc/apt/sources.list.d/ros-latest.list"):
        sudo("sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'")
        sudo("apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116")
        sudo("apt-get update")
        sudo("apt-get -y upgrade")
    else:
        _fp("ros-lastest.list already exists... skipping set up")
    

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
            run("wstool update -j 2 -t src")

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
    
