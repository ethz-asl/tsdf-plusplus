// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_CORE_OBJECT_VOLUME_H_
#define TSDF_PLUSPLUS_CORE_OBJECT_VOLUME_H_

#include <mutex>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "tsdf_plusplus/core/voxel.h"

using namespace voxblox;

class ObjectVolume {
 public:
  ObjectVolume(float voxel_size, size_t voxels_per_side, const Point centroid,
               const SemanticClass& semantic_class);

  inline SemanticClass getSemanticClass() { return semantic_class_; }

  inline void setSemanticClass(SemanticClass semantic_class) {
    semantic_class_ = semantic_class;
  }

  inline Layer<TsdfVoxel>* getTsdfLayerPtr() { return tsdf_layer_.get(); }

  inline Transformation getPose() { return pose_; }

  void accumulateTransform(Transformation transform);

  // Thread safe.
  // Returns a pointer to a TSDF block located at block_idx in the TSDF layer.
  // A block in temp_block_map_ is created/accessed and returned. Accessing
  // temp_block_map_ is controlled via a mutex allowing it to grow in a thread
  // safe manner during integration. These temporary blocks can be merged into
  // the TSDF layer by calling updateLayerWithStoredBlocks().
  Block<TsdfVoxel>::Ptr allocateStorageAndGetBlockPtr(
      const BlockIndex& block_idx);

  // Merges temporarily stored blocks into the TSDF layer.
  // NOT thread safe, see allocateStorageAndGetBlockPtr() for more details.
  void updateLayerWithStoredBlocks();

 protected:
  // TSDF layer of the object.
  std::shared_ptr<Layer<TsdfVoxel>> tsdf_layer_;

  SemanticClass semantic_class_;

  // Temporary storage and mutex for blocks that need
  // to be created while integrating a segment.
  Layer<TsdfVoxel>::BlockHashMap temp_block_map_;
  std::mutex temp_block_mutex_;

  // G_T_G_O i.e. transformation from object to global frame
  // expressed in global frame.
  Transformation pose_;
};

#endif  // TSDF_PLUSPLUS_CORE_OBJECT_VOLUME_H_
