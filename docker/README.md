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

TODO:
 - Allow mounted local workspaced in docker
 - Automatically build the current (remote) branch
 - Entrypoint with all nodes running