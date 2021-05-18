FROM tsdf-plusplus-ros-base:v0.1

# Create 'ros' user with sudo powers
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Create workspace
USER $USERNAME
WORKDIR /home/$USERNAME
ARG ROS_VERSION=melodic
ARG CATKIN_WS=/home/ros/catkin_ws
RUN bash -c 'source /opt/ros/melodic/setup.bash && \
    mkdir -p $CATKIN_WS/src && \
    cd $CATKIN_WS && \
    catkin init && \
    catkin config --extend /opt/ros/melodic --merge-devel && \
    catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release && \
    wstool init src && \
    cd $CATKIN_WS/src && \
    git clone https://github.com/ethz-asl/tsdf-plusplus.git && \
    wstool merge -t . tsdf-plusplus/tsdf_plusplus_https.rosinstall && \
    wstool update'
