# Building image and running container

To build the docker image ( `tsdf-plusplus-ros-catkin-build` ) run:
```bash
./build-ros-image.sh
```

To run the docker image use:
```bash
./run-ros-image.sh
```

# Getting environment up and running

## Copy workspace from docker machine

If you want to get quicky up and runing, keeping sources on your computer
and using docker as a as a perdictable build system you can run:
```bash
./build-ros-image.sh
./set-me-up.sh
```

You can skip running `build-ros-image.sh` if the image is already built.

This `set-me-up.sh` script will do the following:
- Run a docker container
- Copy the built workspace to the destination folder of choice
- Stop the container

Note that if you want to change the remote to SSH or your fork you'll need to edit the origin in the cloned repos.
For instance, in the `tsdf-plusplus` project use something like:
```bash
git remote set-url origin git@github.com:ethz-asl/tsdf-plusplus.git
```

You can confirm you have the right origin URL by running:
```bash
git remote show origin
```

## Use computer workspace on docker

If you have your workspace locally and want to use the docker machine to build
and/or run nodes, you can use the scrip `docker-develop.sh`.
This script will discard the sources from the remote repository and instead
use the sources you have locally on your computer that will be mounted
on the docker machine.

For example, you can use the following command:
```bash
docker-develop.sh /home/<YOUR_USERNAME>/catkin_ws
```

If you want to run it as a daemon you can use:
```bash
docker-develop.sh /home/<YOUR_USERNAME>/catkin_ws dt
```

Once you are in the docker machine you can, for instance, build
from sources doing
```bash
 catkin build -j$(($(nproc) / 2)) -l1 tsdf_plusplus_ros rgbd_segmentation mask_rcnn_ros cloud_segmentation
```

By doing so, the docker machine will build the sources on your computer and
keep the binaries on your computer as well.


TODO:
 - Automatically build the current (remote) branch
 - Entrypoint with all nodes running