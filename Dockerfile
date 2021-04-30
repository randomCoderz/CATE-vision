FROM ros:noetic

# Install all apt related packages
RUN apt-get update && \
    apt-get install -y \ 
    ros-${ROS_DISTRO}-cv-bridge \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install Python3 tensorflow cpu package
RUN pip3 install tensorflow

RUN mkdir -p /home/rospkg/src
COPY vision/ /home/rospkg/src/vision/
SHELL [ "/bin/bash", "-c", "source /opt/ros/noetic/setup.bash"]
WORKDIR /home/rospkg/
RUN catkin_make clean install