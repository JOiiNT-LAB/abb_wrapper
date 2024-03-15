# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-focal

ENV TZ=Europe/Rome

ARG DEBIAN_FRONTEND=noninteractive

ARG HOME
ARG ROS_WS
ENV ROS_WS $ROS_WS

# LOCALE and LANGUAGE settings
RUN apt update \ 
    && apt install -y -q tzdata locales \
    && rm -rf /var/lib/apt/lists/* \    
    && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen \
    && locale-gen \
    && ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime \
    && dpkg-reconfigure tzdata 

ENV LANG=en_US.UTF-8  
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# install ros packages
RUN apt update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt update && apt upgrade -y && apt install -y \
    gdb \
    git \
    htop \
    cmake \
    mlocate \
    openssl \
    net-tools \
    libssl-dev \
    iputils-ping \
    libprotobuf-dev \
    build-essential \
    libboost-all-dev \
    protobuf-compiler \
    libmysqlclient-dev \ 
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p $ROS_WS/src && mv /root/.bashrc /home/ros_user

ENV HOME $HOME

ADD . /home/ros_user/catkin_ws/src

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ${ROS_WS}/devel/setup.bash" >> ~/.bashrc
RUN echo "source ~/.bashrc" 

WORKDIR $ROS_WS

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws/; catkin_make'

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

CMD ["/bin/bash"]