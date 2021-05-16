// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "tsdf_plusplus/visualizer/visualizer.h"

#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <tsdf_plusplus/utils/file_utils.h>
#include "tsdf_plusplus/utils/conversions.h"
#include "tsdf_plusplus/visualizer/visualizer_utils.h"

Visualizer::Visualizer(std::shared_ptr<voxblox::MeshLayer> mesh_layer,
                       std::shared_ptr<std::mutex> mesh_layer_mutex,
                       std::shared_ptr<bool> mesh_layer_updated,
                       std::shared_ptr<Eigen::Matrix4f> camera_extrinsics,
                       const Eigen::Matrix3f& camera_intrinsics,
                       const std::string export_path,
                       bool frames_as_mask_images)
    : mesh_layer_(mesh_layer.get()),
      mesh_layer_mutex_(mesh_layer_mutex.get()),
      mesh_layer_updated_(mesh_layer_updated.get()),
      camera_extrinsics_(camera_extrinsics.get()),
      camera_intrinsics_(camera_intrinsics),
      export_path_(export_path),
      save_screenshot_(false) {}

void Visualizer::run() {
  viewer_ = new pcl::visualization::PCLVisualizer();
  viewer_->setWindowName(config_.window_name.c_str());
  viewer_->setBackgroundColor(config_.background_rgb[0],
                              config_.background_rgb[1],
                              config_.background_rgb[2]);
  viewer_->setSize(800, 600);
  viewer_->setShowFPS(false);

  bool needs_refresh = false;
  pcl::PolygonMesh polygon_mesh;

  while (!viewer_->wasStopped()) {
    viewer_->spinOnce();

    if (mesh_layer_mutex_->try_lock()) {
      if (*mesh_layer_updated_) {
        voxblox::timing::Timer convert_mesh_timer("visualizer/convert_mesh");

        // TODO(margaritaG): check whether simplified or not mesh.
        convertMeshLayerToPCLPolygonMesh(*mesh_layer_, &polygon_mesh);

        convert_mesh_timer.Stop();

        needs_refresh = true;
        *mesh_layer_updated_ = false;
      }

      mesh_layer_mutex_->unlock();
    }

    if (needs_refresh) {
      voxblox::timing::Timer draw_mesh_timer("visualizer/draw_mesh");

      vtkSmartPointer<vtkPolyData> poly_data;
      pcl::VTKUtils::mesh2vtk(polygon_mesh, poly_data);

      viewer_->removeShape("map_mesh");
      viewer_->addModelFromPolyData(poly_data, "map_mesh", 0);

      draw_mesh_timer.Stop();

      pcl::visualization::Camera camera;
      computeCameraParams(camera_intrinsics_, *camera_extrinsics_, &camera);
      viewer_->setCameraParameters(camera);

      needs_refresh = false;
    }

    if (save_screenshot_) {
      saveScreenshot();
      save_screenshot_ = false;
    }
  }
}

void Visualizer::triggerScreenshot(uint32_t frame_number) {
  save_screenshot_ = true;
  frame_number_ = frame_number;
}

void Visualizer::saveScreenshot() {
  std::string export_type = "map";

  std::string path = export_path_ + "/" + export_type;
  CHECK_EQ(makePath(path, 0777), 0);

  std::stringstream ss;
  ss << std::setw(4) << std::setfill('0')
     << static_cast<unsigned>(frame_number_);
  viewer_->saveScreenshot(path + "/frame_" + ss.str() + ".png");
}
