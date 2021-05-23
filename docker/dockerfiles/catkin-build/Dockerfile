FROM tsdf-plusplus-ros-workspace:v0.1

ARG USERNAME=ros
USER $USERNAME
WORKDIR /home/$USERNAME
ARG ROS_VERSION=melodic
ARG CATKIN_WS=/home/ros/catkin_ws
RUN bash -c 'source /opt/ros/melodic/setup.bash && \
    cd $CATKIN_WS/src && \
    git clone https://github.com/ethz-asl/tsdf-plusplus.git && \
    wstool merge -t . tsdf-plusplus/tsdf_plusplus_https.rosinstall && \
    wstool update'
RUN bash -c 'source /opt/ros/melodic/setup.bash && \
    cd $CATKIN_WS && \
    catkin build -j$(($(nproc) / 2)) -l1 tsdf_plusplus_ros rgbd_segmentation mask_rcnn_ros cloud_segmentation'