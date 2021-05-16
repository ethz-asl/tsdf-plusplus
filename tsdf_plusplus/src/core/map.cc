// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "tsdf_plusplus/core/map.h"

#include <shared_mutex>

#include <voxblox/interpolator/interpolator.h>

using namespace voxblox;

Map::Map(const Config& config)
    : config_(config),
      map_layer_(new Layer<MOVoxel>(config.voxel_size, config.voxels_per_side)),
      highest_object_id_(new ObjectID()) {
  voxels_per_side_inv_ =
      1.0f / static_cast<FloatingPoint>(config.voxels_per_side);
  block_size_ = config.voxel_size * config.voxels_per_side;
  object_volumes_.reset(new std::map<ObjectID, ObjectVolume*>());
}

ObjectVolume* Map::getObjectVolumePtrById(const ObjectID& object_id) {
  std::shared_lock<std::shared_timed_mutex> object_volumes_reader_lock(
      object_volumes_mutex_);
  auto object_volume_it = object_volumes_->find(object_id);

  if (object_volume_it != object_volumes_->end()) {
    return object_volume_it->second;
  } else {
    return nullptr;
  }
}

ObjectVolume* Map::allocateObjectVolumePtrById(
    const Point centroid, const SemanticClass& semantic_class,
    const ObjectID& object_id) {
  std::lock_guard<std::shared_timed_mutex> object_volumes_writer_lock(
      object_volumes_mutex_);

  auto object_volume_it = object_volumes_->find(object_id);

  if (object_volume_it != object_volumes_->end()) {
    return object_volume_it->second;
  }

  ObjectVolume* object_volume = new ObjectVolume(
      config_.voxel_size, config_.voxels_per_side, centroid, semantic_class);
  auto insert_status = object_volumes_->emplace(object_id, object_volume);
  CHECK(insert_status.second)
      << "Object volume " << object_id << " already exists when allocating.";
  return insert_status.first->second;
}

TsdfVoxel* Map::allocateStorageAndGetVoxelPtr(
    const Point centroid, const SemanticClass& semantic_class,
    const ObjectID& object_id, const GlobalIndex& global_voxel_idx,
    ObjectVolume** last_object_volume, ObjectID* last_object_id,
    Block<TsdfVoxel>::Ptr* last_tsdf_block, BlockIndex* last_tsdf_block_idx) {
  CHECK_NOTNULL(last_object_volume);
  CHECK_NOTNULL(last_object_id);
  CHECK_NOTNULL(last_tsdf_block);
  CHECK_NOTNULL(last_tsdf_block_idx);

  const BlockIndex block_idx =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);

  if ((object_id != *last_object_id) || (*last_object_volume == nullptr)) {
    *last_object_volume = getObjectVolumePtrById(object_id);

    // If an object volume with ID object_id exists, allocate it.
    if (*last_object_volume == nullptr) {
      *last_object_volume =
          allocateObjectVolumePtrById(centroid, semantic_class, object_id);
    }

    *last_object_id = object_id;

    *last_tsdf_block =
        (*last_object_volume)->getTsdfLayerPtr()->getBlockPtrByIndex(block_idx);

    // If no block at this location currently exists, we allocate it.
    if (*last_tsdf_block == nullptr) {
      *last_tsdf_block =
          (*last_object_volume)->allocateStorageAndGetBlockPtr(block_idx);
    }

    *last_tsdf_block_idx = block_idx;
  } else {
    if ((block_idx != *last_tsdf_block_idx) || (*last_tsdf_block == nullptr)) {
      *last_tsdf_block = (*last_object_volume)
                             ->getTsdfLayerPtr()
                             ->getBlockPtrByIndex(block_idx);

      // If no block at this location currently exists, we allocate it.
      if (*last_tsdf_block == nullptr) {
        *last_tsdf_block =
            (*last_object_volume)->allocateStorageAndGetBlockPtr(block_idx);
      }

      *last_tsdf_block_idx = block_idx;
    }
  }

  // Get the corresponding voxel by 3D position in world frame.
  const VoxelIndex local_voxel_idx =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, config_.voxels_per_side);

  return &((*last_tsdf_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

TsdfVoxel* Map::getTsdfVoxelPtrByVoxelIndex(
    const ObjectID& object_id, const BlockIndex& block_idx,
    const VoxelIndex& voxel_index, ObjectVolume** last_object_volume,
    ObjectID* last_object_id, Block<TsdfVoxel>::Ptr* last_tsdf_block,
    BlockIndex* last_tsdf_block_idx) {
  CHECK_NOTNULL(last_object_volume);
  CHECK_NOTNULL(last_object_id);
  CHECK_NOTNULL(last_tsdf_block);
  CHECK_NOTNULL(last_tsdf_block_idx);

  if ((object_id != *last_object_id) || (*last_object_volume == nullptr)) {
    auto object_volume_it = object_volumes_->find(object_id);

    if (object_volume_it == object_volumes_->end()) {
      // This should not happen, however one other thing that could be
      // done for extra safety is a reader lock, or directly calling
      // getObjectVolumePtrById().
      LOG(ERROR) << "Trying to access index: " << block_idx.transpose()
                 << " of a non-existent object volume "
                 << static_cast<unsigned>(object_id);
      return nullptr;
    }

    *last_object_volume = object_volume_it->second;
    *last_object_id = object_id;

    *last_tsdf_block =
        (*last_object_volume)->getTsdfLayerPtr()->getBlockPtrByIndex(block_idx);
    *last_tsdf_block_idx = block_idx;
  } else {
    if ((block_idx != *last_tsdf_block_idx) || (*last_tsdf_block == nullptr)) {
      *last_tsdf_block = (*last_object_volume)
                             ->getTsdfLayerPtr()
                             ->getBlockPtrByIndex(block_idx);

      if (*last_tsdf_block == nullptr) {
        LOG(ERROR) << "Trying to acces a non-existent block of object "
                   << static_cast<unsigned>(object_id)
                   << " at index: " << block_idx.transpose();
        return nullptr;
      }

      *last_tsdf_block_idx = block_idx;
    }
  }

  return &((*last_tsdf_block)->getVoxelByVoxelIndex(voxel_index));
}

TsdfVoxel* Map::getTsdfVoxelPtrByLinearIndex(
    const ObjectID& object_id, const BlockIndex& block_idx,
    const IndexElement& voxel_index, ObjectVolume** last_object_volume,
    ObjectID* last_object_id, Block<TsdfVoxel>::Ptr* last_tsdf_block,
    BlockIndex* last_tsdf_block_idx) {
  CHECK_NOTNULL(last_object_volume);
  CHECK_NOTNULL(last_object_id);
  CHECK_NOTNULL(last_tsdf_block);
  CHECK_NOTNULL(last_tsdf_block_idx);

  if ((object_id != *last_object_id) || (*last_object_volume == nullptr)) {
    auto object_volume_it = object_volumes_->find(object_id);

    if (object_volume_it == object_volumes_->end()) {
      // This should not happen, however one other thing that could be
      // done for extra safety is a reader lock, or directly calling
      // getObjectVolumePtrById().
      LOG(ERROR) << "Trying to access index: " << block_idx.transpose()
                 << " of a non-existent object volume "
                 << static_cast<unsigned>(object_id);
      return nullptr;
    }

    *last_object_volume = object_volume_it->second;
    *last_object_id = object_id;

    *last_tsdf_block =
        (*last_object_volume)->getTsdfLayerPtr()->getBlockPtrByIndex(block_idx);
    *last_tsdf_block_idx = block_idx;
  } else {
    if ((block_idx != *last_tsdf_block_idx) || (*last_tsdf_block == nullptr)) {
      *last_tsdf_block = (*last_object_volume)
                             ->getTsdfLayerPtr()
                             ->getBlockPtrByIndex(block_idx);

      if (*last_tsdf_block == nullptr) {
        LOG(ERROR) << "Trying to acces a non-existent block of object "
                   << static_cast<unsigned>(object_id)
                   << " at index: " << block_idx.transpose();
        return nullptr;
      }

      *last_tsdf_block_idx = block_idx;
    }
  }

  return &((*last_tsdf_block)->getVoxelByLinearIndex(voxel_index));
}

void Map::transformLayer(const ObjectID& object_id,
                         const Transformation& T_out_in) {
  ObjectVolume* object_volume = getObjectVolumePtrById(object_id);
  Layer<TsdfVoxel>* object_layer = object_volume->getTsdfLayerPtr();

  // First, deactivate all voxels in the map layer
  // corresponding to the object_volume.
  BlockIndexList all_object_blocks;
  object_layer->getAllAllocatedBlocks(&all_object_blocks);

  for (const BlockIndex& block_index : all_object_blocks) {
    Block<MOVoxel>::Ptr mo_block = map_layer_->getBlockPtrByIndex(block_index);

    for (IndexElement voxel_idx = 0;
         voxel_idx < static_cast<IndexElement>(mo_block->num_voxels());
         ++voxel_idx) {
      MOVoxel& mo_voxel = mo_block->getVoxelByLinearIndex(voxel_idx);

      if (mo_voxel.active_object.object_id == object_id) {
        mo_voxel.active_object = mo_voxel.inactive_object;
        mo_voxel.inactive_object = Object();
      } else if (mo_voxel.inactive_object.object_id == object_id) {
        mo_voxel.inactive_object = Object();
      }
    }

    mo_block->updated().set();
  }

  // Next, transform the object_volume.
  Layer<TsdfVoxel>* layer_out = new Layer<TsdfVoxel>(
      object_layer->voxel_size(), object_layer->voxels_per_side());

  // Mark all the blocks in the output layer that may be filled by the
  // input layer (we are conservative here approximating the input blocks as
  // spheres of diameter sqrt(3)*block_size).
  IndexSet block_idx_set;

  for (const BlockIndex& block_index : all_object_blocks) {
    const Point c_in =
        getCenterPointFromGridIndex(block_index, object_layer->block_size());

    // Forwards transform of block center.
    const Point c_out = T_out_in * c_in;

    // Furthest center point of neighboring blocks.
    float kUnitCubeDiagonalLength = std::sqrt(3.0);
    float offset = kUnitCubeDiagonalLength * object_layer->block_size() * 0.5;

    // Add index of all blocks in range to set.
    for (float x = c_out.x() - offset; x < c_out.x() + offset;
         x += layer_out->block_size()) {
      for (float y = c_out.y() - offset; y < c_out.y() + offset;
           y += layer_out->block_size()) {
        for (float z = c_out.z() - offset; z < c_out.z() + offset;
             z += layer_out->block_size()) {
          const Point current_center_out = Point(x, y, z);
          BlockIndex current_idx = getGridIndexFromPoint<BlockIndex>(
              current_center_out, 1.0f / layer_out->block_size());
          block_idx_set.insert(current_idx);
        }
      }
    }
  }

  // Get inverse transform.
  const Transformation T_in_out = T_out_in.inverse();

  Interpolator<TsdfVoxel> interpolator(object_layer);

  ObjectID last_object_id;
  ObjectVolume* last_object_volume = nullptr;
  Block<TsdfVoxel>::Ptr last_tsdf_block = nullptr;
  BlockIndex last_tsdf_block_idx;

  // We now go through all the blocks in the output layer and interpolate the
  // input layer at the center of each output voxel position. For each
  // interpolated voxel, activate the object_id in the corresponding map layer
  // voxel.
  for (const BlockIndex& block_idx : block_idx_set) {
    typename Block<TsdfVoxel>::Ptr block =
        layer_out->allocateBlockPtrByIndex(block_idx);

    // Fetch corresponding block in the map layer.
    typename Block<MOVoxel>::Ptr mo_block =
        getMapLayerPtr()->allocateBlockPtrByIndex(block_idx);

    for (IndexElement voxel_idx = 0;
         voxel_idx < static_cast<IndexElement>(block->num_voxels());
         ++voxel_idx) {
      TsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_idx);
      // Map voxel in which to activate the object being moved.
      MOVoxel& mo_voxel = mo_block->getVoxelByLinearIndex(voxel_idx);

      // Find voxel centers location in the input.
      const Point voxel_center =
          T_in_out * block->computeCoordinatesFromLinearIndex(voxel_idx);

      // Interpolate voxel.
      if (interpolator.getVoxel(voxel_center, &voxel, true)) {
        block->has_data() = true;

        // Deactivate previous object, activate the interpolated one.
        mo_voxel.inactive_object = mo_voxel.active_object;
        mo_voxel.active_object.object_id = object_id;
        // TODO(margaritaG): parametrize this.
        mo_voxel.active_object.confidence = 4u;
      }
    }

    if (!block->has_data()) {
      layer_out->removeBlock(block_idx);
    } else {
      mo_block->updated().set();
    }
  }

  // TODO(margaritaG): potential memory leak,
  // delete the previous tsdf_layer?
  *object_layer = *layer_out;
}

void Map::removeObject(const ObjectID& object_id) {
  ObjectVolume* object_volume = getObjectVolumePtrById(object_id);
  Layer<TsdfVoxel>* object_layer = object_volume->getTsdfLayerPtr();

  // Deactivate all voxels in the map layer
  // corresponding to the object_volume.
  BlockIndexList all_object_blocks;
  object_layer->getAllAllocatedBlocks(&all_object_blocks);

  for (const BlockIndex& block_index : all_object_blocks) {
    Block<MOVoxel>::Ptr mo_block = map_layer_->getBlockPtrByIndex(block_index);

    for (IndexElement voxel_idx = 0;
         voxel_idx < static_cast<IndexElement>(mo_block->num_voxels());
         ++voxel_idx) {
      MOVoxel& mo_voxel = mo_block->getVoxelByLinearIndex(voxel_idx);

      if (mo_voxel.active_object.object_id == object_id) {
        mo_voxel.active_object = mo_voxel.inactive_object;
        mo_voxel.inactive_object = Object();
      } else if (mo_voxel.inactive_object.object_id == object_id) {
        mo_voxel.inactive_object = Object();
      }
    }

    mo_block->updated().set();
  }
}
