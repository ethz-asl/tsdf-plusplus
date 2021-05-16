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

#ifndef TSDF_PLUSPLUS_MESH_MESH_INTEGRATOR_H_
#define TSDF_PLUSPLUS_MESH_MESH_INTEGRATOR_H_

#include <thread>

#include <voxblox/core/layer.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/mesh/mesh_layer.h>

#include "tsdf_plusplus/core/map.h"
#include "tsdf_plusplus/mesh/color_map.h"

using namespace voxblox;

class MOMeshIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    bool use_color = true;
    float min_weight = 1e-4;

    bool using_ground_truth_segmentation = false;

    size_t integrator_threads = std::thread::hardware_concurrency();
  };

  MOMeshIntegrator(const Config& config, std::shared_ptr<Map> map,
                   std::shared_ptr<MeshLayer> mesh_layer);

  // Generates a mesh from the map_layer, returns a boolean
  // whether parts of the layer had to be re-meshed.
  bool generateMesh(bool only_mesh_updated_blocks, bool clear_updated_flag);

 protected:
  void initFromLayer(const Layer<MOVoxel>& map_layer);

  void generateMeshBlocksFunction(const BlockIndexList& all_map_blocks,
                                  bool clear_updated_flag,
                                  ThreadSafeIndex* index_getter);

  void updateMeshForBlock(const BlockIndex& block_index,
                          ObjectVolume** last_object_volume,
                          ObjectID* last_object_id);

  void extractBlockMesh(const BlockIndex& block_index,
                        typename Block<MOVoxel>::ConstPtr block,
                        ObjectVolume** last_object_volume,
                        ObjectID* last_object_id, Mesh::Ptr mesh);

  void extractMeshInsideBlock(const BlockIndex& block_index,
                              const Block<MOVoxel>& block,
                              const VoxelIndex& index, const Point& coords,
                              ObjectVolume** last_object_volume,
                              ObjectID* last_object_id,
                              VertexIndex* next_mesh_index, Mesh* mesh);

  void extractMeshOnBorder(const BlockIndex& block_index,
                           const Block<MOVoxel>& block, const VoxelIndex& index,
                           const Point& coords,
                           ObjectVolume** last_object_volume,
                           ObjectID* last_object_id,
                           VertexIndex* next_mesh_index, Mesh* mesh);

  void updateMeshColor(const Block<MOVoxel>& block, Mesh* mesh);

  Config config_;

  // Map containing the TSDF++ global map layer and the object volumes.
  Map* map_;

  MeshLayer* mesh_layer_;

  // Cached map config.
  FloatingPoint voxel_size_;
  size_t voxels_per_side_;
  FloatingPoint block_size_;

  // Derived types.
  FloatingPoint voxel_size_inv_;
  FloatingPoint voxels_per_side_inv_;
  FloatingPoint block_size_inv_;

  // Cached index map.
  Eigen::Matrix<int, 3, 8> cube_index_offsets_;

  ColorMap color_map_;
};

#endif  // TSDF_PLUSPLUS_MESH_MESH_INTEGRATOR_H_
