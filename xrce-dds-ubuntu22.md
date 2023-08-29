# How to setup PX4 SITL with ROS2 and XRCE-DDS Gazebo simulation on Ubuntu 22

```bash
sudo apt update
sudo apt upgrade
sudo apt install git
```

## Install the PX4 development toolchain to use the simulator

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
```

## Install ROS2 Humble from the link below
* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## Install some dependencies for ROS2
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
pip3 install --user -U empy pyros-genmsg setuptools
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-desktop python3-argcomplete
sudo apt install ros-dev-tools
```
## Install XRCE-DDS Agent
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
## Create a ROS2 workspace and clone the example code, and then build it
```bash
mkdir -p ~/ws_ros2/src/
cd ~/ws_ros2/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
colcon build
## If failed, re-run the build command
```

## Build PX4 firmware for SITL, and run it
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
## For Ubuntu20 run the following: make px4_sitl gazebo-classic
```
## Run the XRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```
## Run the example offboard code
```bash
cd ~/ws_ros2
source install/local_setup.bash
ros2 run px4_ros_com offboard_control
```