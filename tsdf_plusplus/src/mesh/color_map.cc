// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "tsdf_plusplus/mesh/color_map.h"

#include <voxblox/core/color.h>

void ColorMap::getColor(const ObjectID& object_id, voxblox::Color* color) {
  CHECK_NOTNULL(color);
  CHECK_NE(object_id, EmptyID);

  std::map<ObjectID, voxblox::Color>::iterator color_map_it;
  {
    std::shared_lock<std::shared_timed_mutex> readerLock(color_map_mutex_);
    color_map_it = color_map_.find(object_id);
  }

  if (color_map_it != color_map_.end()) {
    *color = color_map_it->second;
  } else {
    *color = voxblox::randomColor();

    std::lock_guard<std::shared_timed_mutex> writerLock(color_map_mutex_);
    color_map_.insert(std::pair<ObjectID, voxblox::Color>(object_id, *color));
  }
}
