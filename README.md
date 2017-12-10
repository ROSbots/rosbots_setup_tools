ROSbots Setup Tools
====================

More details to come... the current not-so-short instructions:

## Using existing ROSbots image (after you've downloaded it)
1. Unzip the image file
1. On a Linux or Mac machine, dd the image to your micro-sd card mapped to rdiskN (replace N with the actual number):
  1. sudo dd if=rosbots-xxx.img of=/dev/rdiskN conv=sync
1. Insert the micro-sd into your RPi then power up
1. ssh pi@xxx.xxx.xxx.xxx (IP address assigned to your RPi)
1. password - rosbots!
1. Set up new ssh keys
 1. sudo rm /etc/ssh/ssh_host_*
 1. sudo dpkg-reconfigure openssh-server
1. sudo raspi-config - expand your filesystem to use the entire SD card.
1. reboot
1. rosnode list (to make sure all is working)

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

## Set up ROSbots setup tools
You will drive the ROSbots setup from a host laptop (not your Raspberry Pi) using a [Python Fabric](http://www.fabfile.org/) recipe fabfile.py file.  

The fabfile "pushes" setup commands to your Pi via ssh to setup and compile the ROS, OpenCV and ROSbots source code.

1. On your host machine (not your Pi but say your Macbook), set up the ROSbots setup tool environment:
  1. Install python and pip (preferably using [virtualenvwrapper](https://pypi.python.org/pypi/virtualenvwrapper)) 
  1. Clone this repo
  1. cd rpi_setup
  1. pip install -r requirement.txt

## To Set up Wifi on Your Raspberry Pi

From your host machine:

1. cd rpi_setup
1. Type "fab -H <ipaddressforyourpi> setup\_wifi\_on\_pi" - enter your password for your Pi
1. Answer the questions about your WiFi network (select a 2.4 GHz network for greater range)
1. Type in "arp -a" again to get current list of IP's on your subnet
1. SSH into your Pi to restart via "sudo shutdown -r now"
1. Wait for your Pi to boot and type "arp -a" again to see what IP address your Pi's Wifi was allocated

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