// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_CORE_SEGMENT_H_
#define TSDF_PLUSPLUS_CORE_SEGMENT_H_

#include <voxblox/core/common.h>

#include <pcl_conversions/pcl_conversions.h>

#include "tsdf_plusplus/core/common.h"

class Segment {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Segment(const pcl::PointCloud<InputPointType>& pointcloud_pcl,
          const voxblox::Transformation& T_G_C);

  Segment(const pcl::PointCloud<GTInputPointType>& pointcloud_pcl,
          const voxblox::Transformation& T_G_C);

  // Populate a voxblox::Pointcloud from the pcl::PointCloud data.
  void convertPointcloud();

  voxblox::Transformation T_G_C_;
  voxblox::Pointcloud points_C_;
  voxblox::Point centroid_;
  voxblox::Colors colors_;
  ObjectID object_id_;
  SemanticClass semantic_class_;

  pcl::PointCloud<InputPointType> pointcloud_;
};

#endif  // TSDF_PLUSPLUS_CORE_SEGMENT_H_
