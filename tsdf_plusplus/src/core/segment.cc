// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "tsdf_plusplus/core/segment.h"

#include <pcl/common/centroid.h>

Segment::Segment(const pcl::PointCloud<InputPointType>& pointcloud_pcl,
                 const voxblox::Transformation& T_G_C)
    : T_G_C_(T_G_C), semantic_class_(pointcloud_pcl.points[0].semantic_class) {
  pointcloud_ = pointcloud_pcl;
  convertPointcloud();
}

Segment::Segment(const pcl::PointCloud<GTInputPointType>& pointcloud_pcl,
                 const voxblox::Transformation& T_G_C)
    : T_G_C_(T_G_C),
      object_id_(pointcloud_pcl.points[0].label + 1),
      semantic_class_(BackgroundClass) {
  pcl::copyPointCloud(pointcloud_pcl, pointcloud_);
  convertPointcloud();
}

void Segment::convertPointcloud() {
  points_C_.clear();
  colors_.clear();

  points_C_.reserve(pointcloud_.points.size());
  colors_.reserve(pointcloud_.points.size());

  for (size_t i = 0u; i < pointcloud_.points.size(); ++i) {
    if (!std::isfinite(pointcloud_.points[i].x) ||
        !std::isfinite(pointcloud_.points[i].y) ||
        !std::isfinite(pointcloud_.points[i].z)) {
      continue;
    }

    points_C_.push_back(voxblox::Point(pointcloud_.points[i].x,
                                       pointcloud_.points[i].y,
                                       pointcloud_.points[i].z));

    colors_.push_back(
        voxblox::Color(pointcloud_.points[i].r, pointcloud_.points[i].g,
                       pointcloud_.points[i].b, pointcloud_.points[i].a));
  }

  Eigen::Vector4f centroid_c;
  pcl::compute3DCentroid(pointcloud_, centroid_c);

  centroid_ =
      T_G_C_ * voxblox::Point(centroid_c.x(), centroid_c.y(), centroid_c.z());
}
