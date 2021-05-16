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

## License
The code is available under the [MIT license](https://github.com/ethz-asl/tsdf-plusplus/blob/master/LICENSE).
