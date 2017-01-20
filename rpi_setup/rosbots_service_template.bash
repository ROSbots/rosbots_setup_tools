#! /bin/sh

### BEGIN INIT INFO
# Provides:		rosbots
# Required-Start:	$all
# Required-Stop:	$remote_fs $syslog
# Default-Start:	5
# Default-Stop:		0 1 6
# Short-Description:	ROSCORE and other ros launch scripts
### END INIT INFO

export ROSBOTS_HOME=_TEMPLATE_HOME
export ROSBOTS_WS_PATH=_TEMPLATE_WS_PATH

start_ros () { 
    nohup ${ROSBOTS_HOME}/rosbots_startup.sh > ${ROSBOTS_HOME}/rosbots_startup_service.log 2>&1 &
}

stop_ros () {
    #killall nodes
    #for i in $( rosnode list ); do
    #rosnode kill $i;
    #done

    #stop roscore
    #killall roscore

    ${ROSBOTS_HOME}/rosbots_shutdown.sh > ${ROSBOTS_HOME}/rosbots_shutdown_service.log 2>&1
}

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    start_ros
    ;;
  stop)
    stop_ros
    ;;
  *)
    echo "Usage: /etc/init.d/rosbots {start|stop}"
    exit 1
    ;;
esac

exit 0
