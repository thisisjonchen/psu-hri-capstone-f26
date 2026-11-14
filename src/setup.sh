# Sourced by the drone container's interactive shell from pidrone_pkg.

source ~/catkin_ws/devel/setup.bash
[ -n "`hostname`" ] && my_hostname="`hostname`" || my_hostname='defaultdrone'
#export ROS_IP=192.168.42.1
export ROS_HOSTNAME="${my_hostname}"
export ROS_MASTER_URI="http://${my_hostname}:11311"
export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] \w$ "
export PYTHONPATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)${PYTHONPATH:+:$PYTHONPATH}"
