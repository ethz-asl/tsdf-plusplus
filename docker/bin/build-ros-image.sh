#!/bin/bash
docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t tsdf-plusplus-ros-base:v0.1 dockerfiles/base
docker build -t tsdf-plusplus-ros-workspace:v0.1 dockerfiles/workspace
docker build -t tsdf-plusplus-ros-catkin-build:v0.1 dockerfiles/catkin-build