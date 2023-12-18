FROM ros:kinetic
LABEL maintainer "Stefan Siegler <stefan.siegler@stud.hslu.ch>"

SHELL ["/bin/bash","-c"]
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src

# copy data/pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148 to /root/documents
COPY data/pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148 /pynaoqi-python2.7-pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148

# add some lines to bashrc
RUN echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc && \
    echo "export PYTHONPATH=/pynaoqi-python2.7-pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148/lib/python2.7/site-packages:${PYTHONPATH}" >> ~/.bashrc && \
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 

# install some apt packages
RUN apt-get update && apt-get install -y \
    git\
    net-tools \
    iputils-ping \
    nano

# clone naoqi_driver and install dependencies
RUN cd ~/catkin_ws/src \
    && git clone https://github.com/ros-naoqi/naoqi_driver.git \
    && rosdep install -i -y --from-paths ./naoqi_driver 

# clone pepper_dcm_robot and install dependencies
RUN cd ~/catkin_ws/src \
    && git clone https://github-com.translate.goog/ros-naoqi/pepper_dcm_robot \
    && rosdep install -i -y --from-paths ./pepper_dcm_robot

# clone pepper_moveit_config and install dependencies
RUN cd ~/catkin_ws/src \
    && git clone https://github-com.translate.goog/ros-naoqi/pepper_moveit_config \
    && rosdep install -i -y --from-paths ./pepper_moveit_config

# clone pepper_virtual and install dependencies
RUN cd ~/catkin_ws/src \
    && git clone https://github-com.translate.goog/ros-naoqi/pepper_virtual \
    && rosdep install -i -y --from-paths ./pepper_virtual 

# build catkin_ws
RUN cd ${CATKIN_WS}/src \
	&& source /opt/ros/${ROS_DISTRO}/setup.bash \
	&& cd ${CATKIN_WS}  && catkin_make

# copy start_ros.sh to /
ADD data/start_ros.sh /start_ros.sh
RUN chmod +x /start_ros.sh

