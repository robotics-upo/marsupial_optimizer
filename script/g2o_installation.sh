read -p "Installing g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash
#####################################################################################################################################################################################
sudo apt-get install cmake libeigen3-dev
sudo apt-get install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev libcholmod3.0.6 liblapack-dev libblas-dev

cd ~
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
#cmake ..
cmake .. -DG2O_BUILD_EXAMPLES=yes
# cmake -DBUILD_WITH_MARCH_NATIVE=ON ..
make
# make -j4
sudo make install

## Run unit_test

# Then to make changes, we first enter the unit_test folder under g2o, and then run the following command:

# mkdir build
# cd build
# cmake ..
# make

# You will find that it will report an error. I have forgotten what the error report is, but it is generally divided into these categories. According to these types of modifications, 
# basically they are generally revised out.

# /usr/local/include/g2o/core There is a problem with the "xx.h"   that you said , then it is generally pointed out that <Eigen/core>there is no such file. In fact, it is a file path 
# problem. My most stupid way is, of course, I have insufficient capacity. I do n’t know how to modify the original g2o CMakelist.txt , And then I just went to the 
# /usr/local/include/g2o/coredirectory and modified it to <Eigen/core> be <eigen3/Eigen/core> that there are so many wrong files for this reason, so it is more troublesome, and I did 
# not think of a better solution.

# When you have finished the above modification, it will be shown DSO missing from command line that there are several problems of this kind. The main thing is that, for example, a 
# certain one libstuff.socannot be found. This is actually a new version of g2o. This will not be generated at all, but g2o_stuff. This type of related problem mainly appears in several 
# sub-file projects, such as stuffand slam3d, slam2detc., all you need to modify is their  corresponding CMakelist.txt, here is an example to illustrate.

# source : https://blog.csdn.net/YuYunTan/article/details/85217619 



#####################################################################################################################################################################################

# Installation G2O Ubuntu 18.04 - ROS Melodic
cd ~
# sudo apt remove ros-melodic-libg2o
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20170730_git
mkdir build
cd build
cmake -DBUILD_WITH_MARCH_NATIVE=ON ..
make -j4
sudo make install

#####################################################################################################################################################################################

# Deleting G2O 

# In doing visual Gao Xiang told slam fourteen when, in order g2o with his version of the same, I removed their equipment before g2o library, the command is as follows:
# 1: Remove g2o header file, located in / usr / local / include / g2o down
    sudo rm -r /usr/local/include/g2o
# 2: Remove g2o library file, located in / usr / local / lib
    sudo rm -r /usr/local/lib/libg2o*
# 3: Remove g2o executable file, located in / usr / local / bin
    sudo rm -r /usr/local/bin/g2o*

# source: https://programmersought.com/article/88062933076/ 