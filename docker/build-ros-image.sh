#!/bin/bash
docker build -t tsdf-plusplus-ros-base:v0.1 base
docker build -t tsdf-plusplus-ros-workspace:v0.1 workspace
docker build -t tsdf-plusplus-ros-catkin-build:v0.1 catkin-build