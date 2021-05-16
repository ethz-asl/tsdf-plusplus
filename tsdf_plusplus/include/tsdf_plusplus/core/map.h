// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_CORE_MAP_H_
#define TSDF_PLUSPLUS_CORE_MAP_H_

#include <shared_mutex>

#include <voxblox/core/layer.h>

#include "tsdf_plusplus/core/object_volume.h"
#include "tsdf_plusplus/core/voxel.h"

using namespace voxblox;

class Map {
 public:
  struct Config {
    float voxel_size = 0.2;
    size_t voxels_per_side = 16u;

    std::string print() const;
  };

  Map(const Config& config);

  float block_size() const { return block_size_; }

  inline ObjectID* getHighestObjectIdPtr() { return highest_object_id_.get(); }

  inline Layer<MOVoxel>* getMapLayerPtr() { return map_layer_.get(); }

  inline std::map<ObjectID, ObjectVolume*>* getObjectVolumesPtr() {
    return object_volumes_.get();
  }

  ObjectVolume* getObjectVolumePtrById(const ObjectID& object_id);

  // Get the object volume by its object_id if
  // it already exists, else allocate a new one.
  ObjectVolume* allocateObjectVolumePtrById(const Point centroid,
                                            const SemanticClass& semantic_class,
                                            const ObjectID& object_id);

  // Thread safe.
  // Returns a pointer to the TSDF voxels located at global_voxel_idx in the
  // specified object_id volume.
  // Takes in the last_object_id, last_tsdf_block and last_tsdf_block_idx to
  // prevent unneeded lookups. If the voxel belongs to a block that has not been
  // allocated, a block in the corresponding object's temp_block_map_ is
  // created/accessed and a voxel from this map is returned.
  // These temporary blocks can be merged into the layer later by calling
  // updateLayerWithStoredBlocks of each object volume.
  TsdfVoxel* allocateStorageAndGetVoxelPtr(
      const Point centroid, const SemanticClass& semantic_class,
      const ObjectID& object_id, const GlobalIndex& global_voxel_idx,
      ObjectVolume** last_object_volume, ObjectID* last_object_id,
      Block<TsdfVoxel>::Ptr* last_tsdf_block, BlockIndex* last_tsdf_block_idx);

  // Gets the TSDF voxel of an object volume at the specified voxel_index.
  TsdfVoxel* getTsdfVoxelPtrByVoxelIndex(const ObjectID& object_id,
                                         const BlockIndex& block_index,
                                         const VoxelIndex& voxel_index,
                                         ObjectVolume** last_object_volume,
                                         ObjectID* last_object_id,
                                         Block<TsdfVoxel>::Ptr* last_tsdf_block,
                                         BlockIndex* last_tsdf_block_idx);

  // Gets the TSDF voxel of an object volume at the specified voxel_index.
  TsdfVoxel* getTsdfVoxelPtrByLinearIndex(
      const ObjectID& object_id, const BlockIndex& block_index,
      const IndexElement& voxel_index, ObjectVolume** last_object_volume,
      ObjectID* last_object_id, Block<TsdfVoxel>::Ptr* last_tsdf_block,
      BlockIndex* last_tsdf_block_idx);

  void transformLayer(const ObjectID& object_id,
                      const Transformation& T_out_in);

  void removeObject(const ObjectID& object_id);

 protected:
  Config config_;

  float voxels_per_side_inv_;
  float block_size_;

  // Global map volume.
  std::shared_ptr<Layer<MOVoxel>> map_layer_;

  // List of the TSDF object volumes in the map.
  std::shared_ptr<std::map<ObjectID, ObjectVolume*>> object_volumes_;

  // Mutex for accessing and creating new object volumes.
  std::shared_timed_mutex object_volumes_mutex_;

  // Field to keep track of the highest object_id generated in the map.
  std::shared_ptr<ObjectID> highest_object_id_;
};

#endif  // TSDF_PLUSPLUS_CORE_MAP_H_
