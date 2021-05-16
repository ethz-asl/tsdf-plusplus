## TSDF++: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction

TSDF++ is a novel multi-object TSDF formulation that can encode multiple object surfaces at each voxel. In a multiple dynamic object tracking and reconstruction scenario, a TSDF++ map representation allows maintaining accurate reconstruction of surfaces even while they become temporarily occluded by other objects moving in their proximity. At the same time, the representation allows maintaining a single volume for the entire scene and all the objects therein, thus solving the fundamental challenge of scalability with respect to the number of objects in the scene and removing the need for an explicit occlusion handling strategy.

## Citing 

When using **TSDF++** in your research, please cite the following publication:

Margarita Grinvald, Federico Tombari, Roland Siegwart, and Juan Nieto, **TSDF++: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction**, in _2021 IEEE International Conference on Robotics and Automation (ICRA)_, 2021. [[Video](https://youtu.be/dSJmoeVasI0)]

```bibtex
@article{grinvald2021tsdf,
  author={M. {Grinvald} and F. {Tombari} and R. {Siegwart} and J. {Nieto}},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  title={{TSDF++}: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction},
  year={2021},
}
```

## Installation


### Prerequisites
Install ROS following the instructions at the [ROS installation page](http://wiki.ros.org/ROS/Installation). The full install (`ros-kinetic-desktop-full`, `ros-melodic-desktop-full`) are recommended. 

Make sure to source your ROS _setup.bash_ script by following the instructions on the ROS installation page.

### Installation on Ubuntu
In your terminal, define the installed ROS version and name of the catkin workspace to use:
```bash
export ROS_VERSION = kinetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic)
export CATKIN_WS = ~/catkin_ws
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

Build and source the voxblox++ packages:
```bash
catkin build 
source ../devel/setup.bash # (bash shell: ../devel/setup.bash,  zsh shell: ../devel/setup.zsh)
```

## License
The code is available under the [MIT license](https://github.com/ethz-asl/tsdf-plusplus/blob/master/LICENSE).
