ROSbots Setup Tools
====================

More details to come... the current not-so-short instructions:

## Use our existing ROSbots Raspbian+ROS+OpenCV image (after you've downloaded it)
1. Download the [ROBsots ROS+OpenCV Raspbian Image](https://medium.com/@rosbots/ready-to-use-image-raspbian-stretch-ros-opencv-324d6f8dcd96). Unzip the image file.
1. On a Windows, Linux or Mac machine, install the image on to your micro-sd card which you plan to use for your Raspberry Pi.
   1. You can use the same [installation instructions as described by the Raspberry Pi folks](https://www.raspberrypi.org/documentation/installation/installing-images/README.md) - instead of their Raspbian image, you'll be using ours.
      1. ie for MacOS - sudo dd if=rosbots-xxx.img of=/dev/rdiskN conv=sync
1. Connect your Pi to your network using an ethernet cable. (We'll set up Wifi after).
1. Insert the micro-sd into your RPi then plug in the power
   1. If you are using our [ROSbots build steps](https://www.rosbots.com/build_it), that would mean [plugging in the red/black Powerboost Lipo battery wires into the Powerboost board](https://www.rosbots.com/static/landing/img/build_it_v02/step_09_powerboost_top/IMG_20180221_083340.jpg) on the top platform.
1. Find the IP address assigned to you Raspberry Pi's ethernet module:
   1. On a Mac or Linux system, that's easily discoverable via "sudo nmap -sP x.x.x.0/24" where you replace x.x.x.0 with your network subnet. 
1. ssh pi@xxx.xxx.xxx.xxx (IP address assigned to your RPi)
   1. password - rosbots!
1. Once logged in, run the following commands in the ssh terminal:
   1. **update_rosbots_git_repos**
      1. This will retrieve our latest setup routines
   1. **initialize_rosbots_image**
      1. This will:
         1. Set up new SSH keys
         1. Ask you for a new password
         1. Run raspi-config where you should enable your Pi Camera and expand your filesystem to use entire SD card space.
            1. Select *Interfacing Options* -> *Camera* -> Enable
            1. Select *Advanced Options* -> *Expand Filesystems*
            1. Finish and Reboot
1. Count to 60, ssh back into your Pi. To make sure ROS is running, type:
   1. **rosnode list**
      1. You should see - /rosout 
1. If you are using a ROSbots robot, complete the ROSbots software setup. From an ssh terminal into your RPi, type:
   1. **setup_rosbots_code**
      1. After setting up some other packages (about 30 seconds), it'll ask you for your RPi password. Once you finished that, you can go grab a cup of coffee. When you return, setup should be complete.
   1. When complete, type - rosnode list
      1. You should see /rosout and /uno_serial_node outputted



## Set up Wifi on Your Raspberry Pi

From your host machine (ie laptop):

1. cd into the rpi_setup/ folder directory in the repository.
1. Type "fab -H *ipaddressforyourpi* setup\_wifi\_on\_pi" - enter your password for your Pi
1. Answer the questions about your WiFi network (select a 2.4 GHz network for greater range)
1. With your ethernet still connected into your Pi, SSH into your Pi. While ssh'd into your Pi, turn on then off the wlan0 interface:
   1. sudo ip link set wlan0 down
   1. sudo ip link set wlan0 up
   1. (wait about 1-2 minutes for wifi to connect to your access point)
1. If above doesn't work, then try reboot via:
   1. sudo shutdown -r now
   1. Count to 60 to make sure the Pi has rebooted.
1. Unplug your Pi's ethernet cable.
1. Discover your Pi's new IP address assigned to its Wifi module:
   1. "sudo nmap -sP x.x.x.0/24" where you replace x.x.x.0 with your network subnet.
1. Now you can SSH into your Pi with the new IP Address assigned to your Pi's Wifi.
  

## With our pre-build Raspbian ROS+OpenCV image, set up ROSbots modules

From your host machine (ie laptop):

1. cd into the rpi_setup/ folder directory in the repository.
1. Type "fab -H *ipaddressforyourpi* main\_setup\_only\_rosbots\_components" - enter your password for your Pi
1. ... wait for completion (about 30 minutes)
1. rosnode list and you should see a /uno_serial_node running
1. Try our first tutorial on Applying [Coursera's Control of Mobile Robots using ROS and ROSbots](https://medium.com/@rosbots/apply-coursera-control-of-mobile-robots-with-ros-and-rosbots-part-1-777a51f63617).



## Setting up a new Raspberry Pi

1. Create an SD card with the latest Raspbian image per the [instructions on Raspberrypi.org](https://www.raspberrypi.org/downloads/raspbian/).
1. Create a stub empty file named "ssh" in the boot main directory of the SD card to enable SSH headless booting.
1. Plug in your newly created Raspbian image SD card into your Pi
1. Plug your Pi into an ethernet jack on your router
1. On a laptop / host machine type "arp -a" to see current IP addresses on your subnet
1. Power on your Pi
1. On a laptop / host machine type "arp -a" again to see what IP address your Pi was allocated
1. ALTERNATIVELY, you can use "sudo nmap -sP x.x.x.0/24" where you replace x.x.x.0 with your network subnet
1. ssh pi@<theipaddress> - default password "raspberry"
1. Change your password via "passwd"
1. Expand your filesystem via "sudo raspi-config" then selecting the expand your filesystem step.  Restart
1. Upon restart, update your raspbian image:
  1. "sudo apt-get update"
  1. "sudo apt-get upgrade"
  

## Set up ROS, OpenCV, ROSbots modules

From your host machine:

1. cd rpi_setup
1. Type "fab -H <wifiipaddressforyourpi> main_setup_ros_opencv_for_rosbots" - enter your password for Pi

This step will take a long while - about 4-6 hours - so leave this running and check in every now and then to make sure everything is ok.

When the step completes:

1. SSH into your Pi
1. Type "rosnode list" to see the current ROS nodes running.



## Shrinking ROSbots image, burning a shrunken ROSbots Raspbian Image to file
1. Use gparted to shrink on Linux machine
  1. If there's an error, then resize2fs manually with "-f" flag
  1. May need to go back to gparted to shrink again to get the partition table updated
1. Burn sd card image (only first 9GB in this case) to file
  1. sudo dd bs=1m of=rosbots-ros-opencv-stretch-lite-2017-11-29.img if=/dev/rdisk2 count=9000 conv=sync


================

[ROSbots Website](www.rosbots.com)

Jack Pien - jack@rosbots.com