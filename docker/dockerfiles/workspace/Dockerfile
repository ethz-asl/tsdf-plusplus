FROM tsdf-plusplus-ros-base:v0.1

# Create workspace
ARG USERNAME=ros
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
    wstool init src'
