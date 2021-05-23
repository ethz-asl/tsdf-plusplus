#!/bin/bash
mode=$1
if [ -z "$1" ]
  then
    mode="it"
fi
docker container run --rm -$mode \
  --user 1001 \
  --name tsdf-plusplus \
  --workdir /home/ros/catkin_ws \
  tsdf-plusplus-ros-catkin-build:v0.1 \
  bash