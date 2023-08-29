# How to install Robot Operating System (ROS)

Ubuntu 20.04 is required. The following commands will install ROS Noetic, create a catkin workspace and build it.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

## Install ROS
sudo apt install ros-noetic-desktop-full
 
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init
rosdep update

## Install catkin_tools
sudo apt-get install python3-catkin-tools

## Create a ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build

## Add source catkin workspace to bashrc file
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

## Test by running
roscore
```