# How to setup PX4 SITL simulation with ROS2 and Gazebo on Ubuntu 20

## You need to install the PX4 development toolchain in order to use the simulator
```bash
sudo apt update
sudo apt upgrade
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
```

## ROS2 Foxy installation
```bash
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
pip3 install --user -U empy pyros-genmsg setuptools
```

## Setup XRCE-DDS Agent
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
## Run the simulation
### In a new tab
```bash
MicroXRCEAgent udp4 -p 8888
```


### In a new tab
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

### In a new tab
```bash
mkdir -p ~/ws_ros2/src/
cd ~/ws_ros2/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
colcon build
##repeat last command if failed
source install/local_setup.bash
ros2 run px4_ros_com offboard_control
```