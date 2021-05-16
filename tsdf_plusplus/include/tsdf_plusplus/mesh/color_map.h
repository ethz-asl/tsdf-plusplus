// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_MESH_COLOR_MAP_H_
#define TSDF_PLUSPLUS_MESH_COLOR_MAP_H_

#include <shared_mutex>

#include <voxblox/core/common.h>

#include "tsdf_plusplus/core/common.h"

class ColorMap {
 public:
  ColorMap() {}

  void getColor(const ObjectID& object_id, voxblox::Color* color);

 protected:
  voxblox::Color randomColor();

  std::map<ObjectID, voxblox::Color> color_map_;
  std::shared_timed_mutex color_map_mutex_;
};

#endif  // TSDF_PLUSPLUS_MESH_COLOR_MAP_H_
