# How to install Robot Operating System 2 (ROS2) Foxy on Ubuntu 20.04
## Download and run the installation script
```bash
wget https://gitlab.kaust.edu.sa/telegek/scripts/-/blob/22d96e7ffcd69a86f329502be30ee50fa59617c4/foxy.sh
bash foxy.sh
```

## or run the following commands in terminal
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade

## Install ROS
sudo apt install ros-foxy-desktop python3-argcomplete
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Run example talker
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal run example listener
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

## Reference
[Installation - ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/Installation.html)