# Building image and running container

To build the docker image run:
```bash
./bin/build-ros-image.sh
```

To run the docker image use:
```bash
./bin/run-ros-image.sh
```

# Getting environment up and running

## Set up local development environment

To get the a local development environment up and running quickly using docker run:
```bash
./bin/build-ros-image.sh
./bin/set-me-up.sh
```

You can skip running `./bin/build-ros-image.sh` if the image is already built.

The `./bin/set-me-up.sh` script will do the following:
- Run a docker container
- Copy the built workspace to the destination folder of choice
- Stop the container

Note that if you want to change the remote to SSH (or your fork) you'll need to edit the `origin` in the cloned repos.

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
and/or run nodes, you can use the script `./bin/docker-develop.sh`.
You can get such workspace locally following the previous section.

This script will discard the workspace from the remote repository and instead
use the worksapce you have locally on your computer, which will be mounted
onto the docker machine.

This means that the docker will use the sources on your machine as well
as build binaries onto your machine. This way you can use docker as a
build tool and code locally on your computer while keeping the binaries.

For example, you can use the following command:
```bash
./bin/docker-develop.sh /home/<YOUR_USERNAME>/catkin_ws
```

Once you are in the docker machine you can, for instance, build
from sources doing
```bash
catkin build -j$(($(nproc) / 2)) -l1 tsdf_plusplus_ros rgbd_segmentation mask_rcnn_ros cloud_segmentation
```

By doing so, the docker machine will build the sources on your computer and
keep the binaries on your computer as well.