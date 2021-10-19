## TSDF++: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction

TSDF++ is a novel multi-object TSDF formulation that can encode multiple object surfaces at each voxel. In a multiple dynamic object tracking and reconstruction scenario, a TSDF++ map representation allows maintaining accurate reconstruction of surfaces even while they become temporarily occluded by other objects moving in their proximity. At the same time, the representation allows maintaining a single volume for the entire scene and all the objects therein, thus solving the fundamental challenge of scalability with respect to the number of objects in the scene and removing the need for an explicit occlusion handling strategy.

## Citing 

When using **TSDF++** in your research, please cite the following publication:

Margarita Grinvald, Federico Tombari, Roland Siegwart, and Juan Nieto, **TSDF++: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction**, in _2021 IEEE International Conference on Robotics and Automation (ICRA)_, 2021. [[Paper](https://arxiv.org/abs/2105.07468)] [[Video](https://youtu.be/dSJmoeVasI0)]

```bibtex
@INPROCEEDINGS{grinvald2021tsdf,
  author={Grinvald, Margarita and Tombari, Federico and Siegwart, Roland and Nieto, Juan},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={{TSDF++: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction}}, 
  year={2021},
  volume={},
  number={},
  pages={14192-14198},
  doi={10.1109/ICRA48506.2021.9560923}}
```

## Installation

The installation has been tested on Ubuntu 16.04 and Ubutnu 20.04.

### Requirements
- ROS 
- C++14 for [PCL 1.10](https://github.com/PointCloudLibrary/pcl)

### Install dependencies
Install ROS following the instructions at the [ROS installation page](http://wiki.ros.org/ROS/Installation). The full install (`ros-kinetic-desktop-full`, `ros-melodic-desktop-full`) are recommended. 

Make sure to source your ROS _setup.bash_ script by following the instructions on the ROS installation page.


### Installation on Ubuntu
In your terminal, define the installed ROS version and name of the catkin workspace to use:
```bash
export ROS_VERSION=kinetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic)
export CATKIN_WS=~/catkin_ws
```

If you don't have a [catkin](http://wiki.ros.org/catkin) workspace yet, create a new one:
```bash
mkdir -p $CATKIN_WS/src && cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel 
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release
wstool init src
```

Clone the `tsdf-plusplus` repository over HTTPS (no Github account required) and automatically fetch dependencies:
```bash
cd $CATKIN_WS/src
git clone https://github.com/ethz-asl/tsdf-plusplus.git
wstool merge -t . tsdf-plusplus/tsdf_plusplus_https.rosinstall
wstool update
```

Alternatively, clone over SSH (Github account required):
```bash
cd $CATKIN_WS/src
git clone git@github.com:ethz-asl/tsdf-plusplus.git
wstool merge -t . tsdf-plusplus/tsdf_plusplus_ssh.rosinstall
wstool update
```

Build and source the TSDF++ packages:
```bash
catkin build tsdf_plusplus_ros rgbd_segmentation mask_rcnn_ros cloud_segmentation
source ../devel/setup.bash # (bash shell: ../devel/setup.bash,  zsh shell: ../devel/setup.zsh)
```


## Troubleshooting
### Compilation freeze
By default `catkin build` on a computer with `N` CPU cores will run `N` `make` jobs simultaneously. If compilation seems to hang forever, it might be running low on RAM. Try limiting the number of maximum parallel build jobs through the `-jN` flag to a value way lower than your CPU count, i.e.
```bash
catkin build tsdf_plusplus_ros rgbd_segmentation mask_rcnn_ros cloud_segmentation -j4
```
If it still freezes at compilation time, you can go as far as limiting the maximum number of parallel build jobs and max load to `1` through the `-lN` flag:
```bash
catkin build tsdf_plusplus_ros rgbd_segmentation mask_rcnn_ros cloud_segmentation -j1 -l1
```

## License
The code is available under the [MIT license](https://github.com/ethz-asl/tsdf-plusplus/blob/master/LICENSE).
