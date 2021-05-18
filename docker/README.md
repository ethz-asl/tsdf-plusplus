# Introduction

To build the docker image ( `tsdf-plusplus-ros-catkin-build` ) run:
```bash
./build-ros-image.sh
```

To run the docker image use:
```bash
./run-ros-image.sh
```

The workspace is located in `/home/ros/tsdf-plusplus_ws` and the `docker` user is `ros`.

TODO:
 - Allow mounted local workspaced in docker
 - Automatically build the current (remote) branch
 - Entrypoint with all nodes running