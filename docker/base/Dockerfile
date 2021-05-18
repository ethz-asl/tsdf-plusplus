FROM ros:melodic-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update -y
RUN apt upgrade -y
RUN apt install -y build-essential tree vim \
    git sudo python-pip ros-melodic-catkin \
    autoconf libtool python-catkin-tools \
    ros-melodic-eigen-conversions \
    ros-melodic-pcl-ros ros-melodic-image-transport \
    ros-melodic-codec-image-transport ros-melodic-pcl-msgs \
    ros-melodic-pcl-conversions ros-melodic-cv-bridge \
    ros-melodic-tf-conversions ros-melodic-rviz && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean && \
    apt autoclean
RUN pip install -U osrf-pycommon wstool setuptools