FROM ros:melodic-ros-core
ENV DEBIAN_FRONTEND=noninteractive

ARG USER_ID
ARG GROUP_ID

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


# Create 'ros' user with sudo powers
ARG USERNAME=ros
RUN groupadd --gid $GROUP_ID $USERNAME
RUN useradd -s /bin/bash --uid $USER_ID --gid $GROUP_ID -m $USERNAME
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME
RUN echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
