// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_CORE_COMMON_H_
#define TSDF_PLUSPLUS_CORE_COMMON_H_

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdint.h>

// Types.
typedef uint16_t ObjectID;
typedef uint16_t Confidence;
typedef uint8_t SemanticClass;

struct Object {
  ObjectID object_id = 0u;
  Confidence confidence = 0u;
};

// Constants.
const ObjectID EmptyID = 0u;
const ObjectID BackgroundID = 1u;
const SemanticClass BackgroundClass = 0u;

// Input pointcloud type.
struct PointXYZRGBCNormal {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  SemanticClass semantic_class;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBCNormal,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, rgb, rgb)(
        SemanticClass, semantic_class, semantic_class))

// Point types.
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeNormal;
typedef pcl::PointXYZRGBL GTInputPointType;
typedef PointXYZRGBCNormal InputPointType;

#endif  // TSDF_PLUSPLUS_CORE_COMMON_H_
