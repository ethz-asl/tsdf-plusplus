#!/bin/bash
docker container run --rm -it \
  --user 1001 \
  --workdir /home/ros/catkin_ws \
  tsdf-plusplus-ros-catkin-build:v0.1 \
  bash