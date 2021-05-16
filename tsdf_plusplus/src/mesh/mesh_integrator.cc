// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "tsdf_plusplus/mesh/mesh_integrator.h"

#include <voxblox/mesh/marching_cubes.h>
#include <voxblox/utils/meshing_utils.h>

using namespace voxblox;

MOMeshIntegrator::MOMeshIntegrator(const Config& config,
                                   std::shared_ptr<Map> map,
                                   std::shared_ptr<MeshLayer> mesh_layer)
    : config_(config), map_(map.get()), mesh_layer_(mesh_layer.get()) {
  initFromLayer(*map_->getMapLayerPtr());

  cube_index_offsets_ << 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
      0, 0, 1, 1, 1, 1;

  if (config_.integrator_threads == 0) {
    LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
    config_.integrator_threads = 1;
  }
}

void MOMeshIntegrator::initFromLayer(const Layer<MOVoxel>& map_layer) {
  voxel_size_ = map_layer.voxel_size();
  block_size_ = map_layer.block_size();
  voxels_per_side_ = map_layer.voxels_per_side();

  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_inv_ = 1.0 / block_size_;
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

bool MOMeshIntegrator::generateMesh(bool only_mesh_updated_blocks,
                                    bool clear_updated_flag) {
  BlockIndexList all_map_blocks;
  if (only_mesh_updated_blocks) {
    map_->getMapLayerPtr()->getAllUpdatedBlocks(Update::kMesh, &all_map_blocks);
    if (all_map_blocks.size() == 0u) {
      return false;
    }
  } else {
    map_->getMapLayerPtr()->getAllAllocatedBlocks(&all_map_blocks);
  }

  // Allocate all the mesh memory.
  for (const BlockIndex& block_index : all_map_blocks) {
    mesh_layer_->allocateMeshPtrByIndex(block_index);
  }

  std::unique_ptr<ThreadSafeIndex> index_getter(
      new MixedThreadSafeIndex(all_map_blocks.size()));

  std::list<std::thread> integration_threads;

  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(
        &MOMeshIntegrator::generateMeshBlocksFunction, this,
        std::cref(all_map_blocks), clear_updated_flag, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  return true;
}

void MOMeshIntegrator::generateMeshBlocksFunction(
    const BlockIndexList& all_map_blocks, bool clear_updated_flag,
    ThreadSafeIndex* index_getter) {
  CHECK(index_getter != nullptr);

  ObjectID last_object_id;
  ObjectVolume* last_object_volume = nullptr;

  size_t list_idx;
  while (index_getter->getNextIndex(&list_idx)) {
    const BlockIndex& block_idx = all_map_blocks[list_idx];
    updateMeshForBlock(block_idx, &last_object_volume, &last_object_id);
    if (clear_updated_flag) {
      typename Block<MOVoxel>::Ptr block =
          map_->getMapLayerPtr()->getBlockPtrByIndex(block_idx);
      block->updated().reset(Update::kMesh);
    }
  }
}

void MOMeshIntegrator::updateMeshForBlock(const BlockIndex& block_index,
                                          ObjectVolume** last_object_volume,
                                          ObjectID* last_object_id) {
  Mesh::Ptr mesh = mesh_layer_->getMeshPtrByIndex(block_index);
  mesh->clear();
  // This block should already exist, otherwise it makes no sense to update
  // the mesh for it.
  typename Block<MOVoxel>::ConstPtr block =
      map_->getMapLayerPtr()->getBlockPtrByIndex(block_index);

  if (!block) {
    LOG(ERROR) << "Trying to mesh a non-existent block at index: "
               << block_index.transpose();
    return;
  }
  //  TODO(margaritaG): do not pass both block_index and block, as duplicate.
  extractBlockMesh(block_index, block, last_object_volume, last_object_id,
                   mesh);
  // Update colors if needed.
  if (config_.use_color) {
    updateMeshColor(*block, mesh.get());
  }

  mesh->updated = true;
}

void MOMeshIntegrator::extractBlockMesh(const BlockIndex& block_index,
                                        typename Block<MOVoxel>::ConstPtr block,
                                        ObjectVolume** last_object_volume,
                                        ObjectID* last_object_id,
                                        Mesh::Ptr mesh) {
  CHECK(block != nullptr);
  CHECK(mesh != nullptr);

  IndexElement vps = block->voxels_per_side();
  VertexIndex next_mesh_index = 0;

  VoxelIndex voxel_index;
  for (voxel_index.x() = 0; voxel_index.x() < vps - 1; ++voxel_index.x()) {
    for (voxel_index.y() = 0; voxel_index.y() < vps - 1; ++voxel_index.y()) {
      for (voxel_index.z() = 0; voxel_index.z() < vps - 1; ++voxel_index.z()) {
        Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
        extractMeshInsideBlock(block_index, *block, voxel_index, coords,
                               last_object_volume, last_object_id,
                               &next_mesh_index, mesh.get());
      }
    }
  }

  // Max X plane
  // takes care of edge (x_max, y_max, z),
  // takes care of edge (x_max, y, z_max).
  voxel_index.x() = vps - 1;
  for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
    for (voxel_index.y() = 0; voxel_index.y() < vps; voxel_index.y()++) {
      Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(block_index, *block, voxel_index, coords,
                          last_object_volume, last_object_id, &next_mesh_index,
                          mesh.get());
    }
  }

  // Max Y plane.
  // takes care of edge (x, y_max, z_max),
  // without corner (x_max, y_max, z_max).
  voxel_index.y() = vps - 1;
  for (voxel_index.z() = 0; voxel_index.z() < vps; voxel_index.z()++) {
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
      Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(block_index, *block, voxel_index, coords,
                          last_object_volume, last_object_id, &next_mesh_index,
                          mesh.get());
    }
  }

  // Max Z plane.
  voxel_index.z() = vps - 1;
  for (voxel_index.y() = 0; voxel_index.y() < vps - 1; voxel_index.y()++) {
    for (voxel_index.x() = 0; voxel_index.x() < vps - 1; voxel_index.x()++) {
      Point coords = block->computeCoordinatesFromVoxelIndex(voxel_index);
      extractMeshOnBorder(block_index, *block, voxel_index, coords,
                          last_object_volume, last_object_id, &next_mesh_index,
                          mesh.get());
    }
  }
}

void MOMeshIntegrator::extractMeshInsideBlock(
    const BlockIndex& block_index, const Block<MOVoxel>& block,
    const VoxelIndex& index, const Point& coords,
    ObjectVolume** last_object_volume, ObjectID* last_object_id,
    VertexIndex* next_mesh_index, Mesh* mesh) {
  CHECK(next_mesh_index != nullptr);
  CHECK(mesh != nullptr);

  Block<TsdfVoxel>::Ptr last_tsdf_block = nullptr;
  BlockIndex last_tsdf_block_idx;

  Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
      cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
  Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
  Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
  bool all_neighbors_observed = true;

  for (unsigned int i = 0; i < 8; ++i) {
    VoxelIndex corner_index = index + cube_index_offsets_.col(i);
    const MOVoxel& voxel = block.getVoxelByVoxelIndex(corner_index);

    // Get the id of the object currently active at this voxel.
    ObjectID object_id = voxel.active_object.object_id;

    if (object_id == EmptyID) {
      all_neighbors_observed = false;
      break;
    }

    // Get the corresponding TSDF voxel of the object.
    TsdfVoxel* tsdf_voxel = map_->getTsdfVoxelPtrByVoxelIndex(
        object_id, block_index, corner_index, last_object_volume,
        last_object_id, &last_tsdf_block, &last_tsdf_block_idx);

    // TODO(margaritaG): this should not happen so remove,
    // originally was not here.
    if (!tsdf_voxel) {
      all_neighbors_observed = false;
      break;
    }

    if (!utils::getSdfIfValid(*tsdf_voxel, config_.min_weight,
                              &(corner_sdf(i)))) {
      all_neighbors_observed = false;
      break;
    }

    corner_coords.col(i) = coords + cube_coord_offsets.col(i);
  }

  if (all_neighbors_observed) {
    MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index, mesh);
  }
}

void MOMeshIntegrator::extractMeshOnBorder(
    const BlockIndex& block_index, const Block<MOVoxel>& block,
    const VoxelIndex& index, const Point& coords,
    ObjectVolume** last_object_volume, ObjectID* last_object_id,
    VertexIndex* next_mesh_index, Mesh* mesh) {
  CHECK(mesh != nullptr);

  Block<TsdfVoxel>::Ptr last_tsdf_block = nullptr;
  BlockIndex last_tsdf_block_idx;

  Eigen::Matrix<FloatingPoint, 3, 8> cube_coord_offsets =
      cube_index_offsets_.cast<FloatingPoint>() * voxel_size_;
  Eigen::Matrix<FloatingPoint, 3, 8> corner_coords;
  Eigen::Matrix<FloatingPoint, 8, 1> corner_sdf;
  bool all_neighbors_observed = true;
  corner_coords.setZero();
  corner_sdf.setZero();

  for (unsigned int i = 0; i < 8; ++i) {
    VoxelIndex corner_index = index + cube_index_offsets_.col(i);

    if (block.isValidVoxelIndex(corner_index)) {
      const MOVoxel& voxel = block.getVoxelByVoxelIndex(corner_index);
      // Get the id of the object currently active at this voxel.
      ObjectID object_id = voxel.active_object.object_id;

      if (object_id == EmptyID) {
        all_neighbors_observed = false;
        break;
      }

      // TODO(margaritaG): investigate occasional segfault here.
      // Get the corresponding TSDF voxel of the object.
      TsdfVoxel* tsdf_voxel = map_->getTsdfVoxelPtrByVoxelIndex(
          object_id, block_index, corner_index, last_object_volume,
          last_object_id, &last_tsdf_block, &last_tsdf_block_idx);

      // TODO(margaritaG): this should not happen so remove,
      // originally was not here.
      if (!tsdf_voxel) {
        all_neighbors_observed = false;
        break;
      }

      if (!utils::getSdfIfValid(*tsdf_voxel, config_.min_weight,
                                &(corner_sdf(i)))) {
        all_neighbors_observed = false;
        break;
      }

      corner_coords.col(i) = coords + cube_coord_offsets.col(i);
    } else {
      // We have to access a different block.
      BlockIndex block_offset = BlockIndex::Zero();

      for (unsigned int j = 0u; j < 3u; j++) {
        if (corner_index(j) < 0) {
          block_offset(j) = -1;
          corner_index(j) = corner_index(j) + voxels_per_side_;
        } else if (corner_index(j) >=
                   static_cast<IndexElement>(voxels_per_side_)) {
          block_offset(j) = 1;
          corner_index(j) = corner_index(j) - voxels_per_side_;
        }
      }

      BlockIndex neighbor_index = block.block_index() + block_offset;

      if (map_->getMapLayerPtr()->hasBlock(neighbor_index)) {
        const Block<MOVoxel>& neighbor_block =
            map_->getMapLayerPtr()->getBlockByIndex(neighbor_index);

        CHECK(neighbor_block.isValidVoxelIndex(corner_index));
        const MOVoxel& voxel =
            neighbor_block.getVoxelByVoxelIndex(corner_index);
        // Get the id of the object currently active at this voxel.
        ObjectID object_id = voxel.active_object.object_id;

        if (object_id == EmptyID) {
          all_neighbors_observed = false;
          break;
        }

        // Get the corresponding TSDF voxel of the object.
        TsdfVoxel* tsdf_voxel = map_->getTsdfVoxelPtrByVoxelIndex(
            object_id, neighbor_index, corner_index, last_object_volume,
            last_object_id, &last_tsdf_block, &last_tsdf_block_idx);

        // TODO(margaritaG): this should not happen so remove,
        // originally was not here.
        if (!tsdf_voxel) {
          all_neighbors_observed = false;
          break;
        }

        if (!utils::getSdfIfValid(*tsdf_voxel, config_.min_weight,
                                  &(corner_sdf(i)))) {
          all_neighbors_observed = false;
          break;
        }

        corner_coords.col(i) = coords + cube_coord_offsets.col(i);
      } else {
        all_neighbors_observed = false;
        break;
      }
    }
  }

  if (all_neighbors_observed) {
    MarchingCubes::meshCube(corner_coords, corner_sdf, next_mesh_index, mesh);
  }
}

void MOMeshIntegrator::updateMeshColor(const Block<MOVoxel>& block,
                                       Mesh* mesh) {
  CHECK(mesh != nullptr);

  mesh->colors.clear();
  mesh->colors.resize(mesh->indices.size());

  // Use nearest-neighbor search.
  for (size_t i = 0; i < mesh->vertices.size(); i++) {
    const Point& vertex = mesh->vertices[i];
    VoxelIndex voxel_index = block.computeVoxelIndexFromCoordinates(vertex);
    if (block.isValidVoxelIndex(voxel_index)) {
      const MOVoxel& voxel = block.getVoxelByVoxelIndex(voxel_index);
      ObjectID object_id = voxel.active_object.object_id;
      ObjectVolume* object_volume = map_->getObjectVolumePtrById(object_id);
      if (config_.using_ground_truth_segmentation) {
        if (object_id != 0u) {
          color_map_.getColor(object_id, &(mesh->colors[i]));
        } else {
          mesh->colors[i] = Color(200u, 200u, 200u);
        }
      } else {
        if (object_volume->getSemanticClass() != BackgroundClass) {
          color_map_.getColor(object_id, &(mesh->colors[i]));
        } else {
          mesh->colors[i] = Color(200u, 200u, 200u);
        }
      }
    } else {
      const typename Block<MOVoxel>::ConstPtr neighbor_block =
          map_->getMapLayerPtr()->getBlockPtrByCoordinates(vertex);
      const MOVoxel& voxel = neighbor_block->getVoxelByCoordinates(vertex);
      ObjectID object_id = voxel.active_object.object_id;
      ObjectVolume* object_volume = map_->getObjectVolumePtrById(object_id);
      if (config_.using_ground_truth_segmentation) {
        if (object_id != 0u) {
          color_map_.getColor(object_id, &(mesh->colors[i]));
        } else {
          mesh->colors[i] = Color(200u, 200u, 200u);
        }
      } else {
        if (object_volume->getSemanticClass() != BackgroundClass) {
          color_map_.getColor(object_id, &(mesh->colors[i]));
        } else {
          mesh->colors[i] = Color(200u, 200u, 200u);
        }
      }
      // TODO(margaritaG): Should check or not whether valid SDF? If not
      // valid, then let the voxel be black?
      // utils::getColorIfValid(voxel, config_.min_weight,
      // &(mesh->colors[i]));
    }
  }
}
