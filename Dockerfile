FROM ros:noetic

# By default, do everything with bash
SHELL ["/bin/bash", "-c"]

# Install all apt related packages
RUN apt-get update && \
    apt-get install -y \ 
    ros-${ROS_DISTRO}-cv-bridge \
    python3-pip \ 
    vim

# Install Python3 tensorflow cpu package
RUN pip3 install tensorflow

RUN mkdir -p /ws/src
COPY vision/ /ws/src/vision/
ENV ROS_PACKAGE_PATH=/ws/src/vision
RUN rosdep install -y -r --from-path /ws/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash; \
    cd /ws/src; \
    catkin_init_workspace; \
    cd ..; \
    catkin_make clean install

RUN rm -rf /var/lib/apt/lists/*

RUN echo source /opt/ros/noetic/setup.bash >> /root/.bashrc
RUN echo export ROS_MASTER_URI=http://127.0.0.1:11311 >> /root/.bashrc
RUN echo export ROS_HOSTNAME=127.0.0.1 >> /root/.bashrc
