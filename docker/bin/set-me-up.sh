#!/bin/bash
./bin/run-ros-image.sh dt
sleep 5
container_id=$(docker ps -aqf "name=tsdf-plusplus" | tr -d '\n')
echo "Docker container $container_id"
echo "Write absolute path of the folder on your computer, you want the entire '/home/ros/catkin_ws' folder on the docker machine to be copied into:"
read destination_folder
docker cp $container_id:/home/ros/catkin_ws $destination_folder
docker stop -t 1 $container_id