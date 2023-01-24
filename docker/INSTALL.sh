#!/bin/sh

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

apt-get update && apt-get install -y wget && apt-get install -y zip && apt-get install -y vim
cd ~ && wget https://choreonoid.org/_downloads/82c0099aa3492a273519e1906ea56e54/choreonoid-1.7.0.zip && unzip choreonoid-1.7.0.zip && rm choreonoid-1.7.0.zip

cd ~/choreonoid-1.7.0/ext && git clone https://github.com/xinyiz0931/grasp-plugin-choreonoid-1.7.git graspPlugin

cd ~/choreonoid-1.7.0/ext/graspPlugin && INSTALL-REQUISITES.sh 