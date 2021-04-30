FROM ros:noetic

# Install all apt related packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \ 
    ros-${ROS_DISTRO}-cv-bridge \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install Python3 tensorflow cpu package
RUN pip3 install tensorflow

RUN mkdir -p /home/rospkg/src
COPY vision /home/rospkg/src
WORKDIR /home/rospkg/
SHELL [ "/bin/bash", "-c", "source /opt/ros/noetic/setup.bash"]
RUN catkin_make clean install
SHELL [ "/bin/bash", "-c", "source /home/rospkg/devel/setup.bash" ]