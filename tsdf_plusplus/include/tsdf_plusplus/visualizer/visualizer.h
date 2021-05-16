// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_VISUALIZER_VISUALIZER_H_
#define TSDF_PLUSPLUS_VISUALIZER_VISUALIZER_H_

#include <mutex>

#include <pcl/visualization/pcl_visualizer.h>
#include <voxblox/mesh/mesh_layer.h>

class Visualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config {
    std::string window_name = "TSDF++ Map";
    double background_rgb[3] = {0.15, 0.15, 0.15};
  };

  Visualizer(std::shared_ptr<voxblox::MeshLayer> mesh_layer,
             std::shared_ptr<std::mutex> mesh_layer_mutex,
             std::shared_ptr<bool> mesh_layer_updated,
             std::shared_ptr<Eigen::Matrix4f> camera_extrinsics,
             const Eigen::Matrix3f& camera_intrinsics,
             const std::string export_path, bool frames_as_mask_images);

  void run();

  void triggerScreenshot(uint32_t frame_number);

 protected:
  void saveScreenshot();

  Config config_;

  // The main viewer.
  pcl::visualization::PCLVisualizer* viewer_;

  // Mesh layer to visualize.
  voxblox::MeshLayer* mesh_layer_;

  // Mutex to prevent reading from mesh layer while it is being updated.
  std::mutex* mesh_layer_mutex_;

  // Flag to keep track of when the mesh layer
  // has been updated and needs to be redrawn.
  bool* mesh_layer_updated_;

  // Camera parameters used to fit the visualizer
  // view to the current camera viewpoint.
  Eigen::Matrix4f* camera_extrinsics_;
  Eigen::Matrix3f camera_intrinsics_;

  // Accessories for storing the sequence of visualizer frames as PNG.
  uint32_t frame_number_;
  bool save_screenshot_;
  std::string export_path_;
};

#endif  // TSDF_PLUSPLUS_VISUALIZER_VISUALIZER_H_
