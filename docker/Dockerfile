FROM osrf/ros:melodic-desktop-full

SHELL [ "/bin/bash" , "-c" ]
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN apt-get update && apt-get install -y wget && apt-get install -y zip && apt-get install -y vim
RUN cd ~ && wget https://choreonoid.org/_downloads/82c0099aa3492a273519e1906ea56e54/choreonoid-1.7.0.zip && unzip choreonoid-1.7.0.zip && rm choreonoid-1.7.0.zip

RUN cd ~/choreonoid-1.7.0/ext && git clone https://github.com/xinyiz0931/grasp-plugin-choreonoid-1.7.git graspPlugin

RUN cd ~/choreonoid-1.7.0/ext/graspPlugin && ./INSTALL-REQUISITES.sh 

RUN echo "All Done!"