#!/bin/sh

# General stup NUC for marsupial agents

echo "Calling UPDATE \n\n"

sudo apt update
## Install Google Chrome
echo "\n\n INSTALLING GOOGLE CHROME \n\n"
wget --version
sudo apt install wget
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo dpkg -i google-chrome-stable_current_amd64.deb

## Install Terminator
echo "\n\n INSTALLINS TERMINATOR \n\n"
sudo apt-get update
sudo apt-get install terminator -y

## Install Visual Studio Code
echo "\n\n INSTALLING VISUAL STUDIO CODE \n\n"
#sudo snap install --classic code
sudo apt update
sudo apt install software-properties-common apt-transport-https wget
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"	
sudo apt install code

## Install ROS Noetic
echo "\n\n INSTALLING ROS_NOETIC \n\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
