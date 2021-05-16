// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_CORE_VOXEL_H_
#define TSDF_PLUSPLUS_CORE_VOXEL_H_

#include "tsdf_plusplus/core/common.h"

// Multi-object TSDF++ voxel.
struct MOVoxel {
  Object active_object;
  Object inactive_object;
};

#endif  // TSDF_PLUSPLUS_CORE_VOXEL_H_
