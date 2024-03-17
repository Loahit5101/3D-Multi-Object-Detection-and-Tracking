FROM ros:noetic

VOLUME /home/loahit/Downloads/projects/perception_project/

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

COPY . /catkin_ws/src

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl tree \
    build-essential \
    python3-pip \
    python3-dev
    
RUN apt-get update && apt-get install -y \
    libvtk7-dev \
    libusb-1.0-0-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-roscpp \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-pcl-conversions \
    libpcl-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  cd /catkin_ws && \
                  catkin_make"

