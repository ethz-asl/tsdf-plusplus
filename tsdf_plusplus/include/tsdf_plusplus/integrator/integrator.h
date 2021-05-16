// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_INTEGRATOR_INTEGRATOR_H_
#define TSDF_PLUSPLUS_INTEGRATOR_INTEGRATOR_H_

#include <memory>
#include <thread>

#include <voxblox/core/common.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/approx_hash_array.h>

#include "tsdf_plusplus/core/map.h"
#include "tsdf_plusplus/core/segment.h"

using namespace voxblox;

typedef std::map<Segment*, size_t> SegmentHistogram;

class Integrator {
 public:
  struct Config {
    // Segment to object matching.
    float min_overlap_ratio = 0.2f;

    float truncation_distance = 0.1;
    float max_weight = 10000.0;
    bool voxel_carving_enabled = true;
    float min_ray_length_m = 0.1;
    float max_ray_length_m = 5.0;
    bool use_const_weight = false;
    bool allow_clear = true;
    bool use_weight_dropoff = true;
    bool use_sparsity_compensation_factor = false;
    float sparsity_compensation_factor = 1.0f;

    size_t integrator_threads = std::thread::hardware_concurrency();

    // ThreadSafeInfex mode results in rays being integrated
    // in sorted or mixed order. Options: "mixed", "sorted"
    std::string integration_order_mode = "mixed";

    /// Merge integrator specific
    bool enable_anti_grazing = false;
  };

  Integrator(const Config& config, std::shared_ptr<Map> map);

  // Compute the pairwise overlap (expressed as the number of points) between
  // a segment in the current frame and the corresponding objects in the map.
  void computeObjectOverlap(
      Segment* segment,
      std::map<ObjectID, SegmentHistogram>* object_segment_overlap);

  // Assign to each segment either the object_id of one of the objects
  // in the map it overlaps with, or a new, previously unseen object_id.
  void assignObjectIds(
      std::vector<Segment*>* current_frame_segments,
      std::map<ObjectID, SegmentHistogram>* object_segment_overlap,
      std::map<ObjectID, Segment*>* object_merged_segments);

  void integrateSegment(const Segment& segment);

 protected:
  // Returns a new unique object_id, that has not been previously
  // used, to initialize new objects in the map.
  inline ObjectID getFreshObjectId() {
    CHECK_LT(*highest_object_id_, std::numeric_limits<ObjectID>::max());
    return ++(*highest_object_id_);
  }

  // Thread safe.
  inline bool isPointValid(const Point& point_C, bool* is_clearing) const {
    CHECK(is_clearing != nullptr);
    const FloatingPoint ray_distance = point_C.norm();
    if (ray_distance < config_.min_ray_length_m) {
      return false;
    } else if (ray_distance > config_.max_ray_length_m) {
      if (config_.allow_clear) {
        *is_clearing = true;
        return true;
      } else {
        return false;
      }
    } else {
      *is_clearing = false;
      return true;
    }
  }

  void increaseOverlapCount(
      const ObjectID object_id, Segment* segment, size_t count,
      std::map<ObjectID, SegmentHistogram>* object_segment_overlap);

  bool nextSegmentObjectPair(
      const std::map<ObjectID, SegmentHistogram>& object_segment_overlap,
      const std::set<Segment*>& assigned_segments,
      std::pair<Segment*, ObjectID>* segment_object_pair);

  void bundleRays(const Transformation& T_G_C, const Pointcloud& points_C,
                  ThreadSafeIndex* index_getter,
                  LongIndexHashMapType<AlignedVector<size_t>>::type* voxel_map,
                  LongIndexHashMapType<AlignedVector<size_t>>::type* clear_map);

  void integrateRays(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Point centroid, const ObjectID& object_id,
      const SemanticClass& semantic_class, const Colors& colors,
      bool enable_anti_grazing, bool clearing_ray,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map);

  void integrateVoxels(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Point centroid, const ObjectID& object_id,
      const SemanticClass& semantic_class, const Colors& colors,
      bool enable_anti_grazing, bool clearing_ray,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
      size_t thread_idx);

  void integrateVoxel(
      const Transformation& T_G_C, const Pointcloud& points_C,
      const Point centroid, const ObjectID& object_id,
      const SemanticClass& semantic_class, const Colors& colors,
      bool enable_anti_grazing, bool clearing_ray,
      const std::pair<GlobalIndex, AlignedVector<size_t>>& kv,
      const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map);

  // Thread safe.
  // Will return a pointer to a voxel located at global_voxel_idx in the map
  // layer. Takes in the last_block_idx and last_block to prevent unneeded map
  // lookups. If this voxel belongs to a block that has not been allocated, a
  // block in temp_block_map_ is created/accessed and a voxel from this map is
  // returned instead. Unlike the layer, accessing temp_block_map_ is controlled
  // via a mutex allowing it to grow during integration. These temporary blocks
  // can be merged into the layer later by calling updateLayerWithStoredBlocks.
  MOVoxel* allocateStorageAndGetVoxelPtr(const GlobalIndex& global_voxel_idx,
                                         Block<MOVoxel>::Ptr* last_block,
                                         BlockIndex* last_block_idx);

  // Merges temporarily stored blocks into the main layer.
  // NOT thread safe, see allocateStorageAndGetVoxelPtr for more details.
  void updateLayerWithStoredBlocks();

  // Updates mo_voxel, thread safe.
  void updateMOVoxel(const Point centroid, const SemanticClass& semantic_class,
                     const Point& origin, const Point& point_G,
                     const ObjectID& object_id,
                     const GlobalIndex& global_voxel_idx, const Color& color,
                     const float weight, MOVoxel* mo_voxel,
                     ObjectVolume** last_object_volume,
                     ObjectID* last_object_id,
                     Block<TsdfVoxel>::Ptr* last_tsdf_block,
                     BlockIndex* last_tsdf_block_idx);

  // Thread safe.
  // Figure out whether the voxel is behind or in front of the surface.
  // To do this, project the voxel_center onto the ray from origin to point G.
  // Then check if the the magnitude of the vector is smaller or greater than
  // the original distance...
  float computeDistance(const Point& origin, const Point& point_G,
                        const Point& voxel_center) const;

  // Thread safe.
  float getVoxelWeight(const Point& point_C) const;

  Config config_;

  // Map containing the global map layer and the object volumes.
  Map* map_;

  // Reference to the highest object_id generated in the map.
  ObjectID* highest_object_id_;

  // Cached map config.
  float voxel_size_;
  size_t voxels_per_side_;
  float block_size_;

  // Derived types.
  float voxel_size_inv_;
  float voxels_per_side_inv_;
  float block_size_inv_;

  // Temporary storage and mutex for blocks that need
  // to be created while integrating a segment.
  Layer<MOVoxel>::BlockHashMap temp_block_map_;
  std::mutex temp_block_mutex_;

  // We need to prevent simultaneous access to the voxels in the map. We could
  // put a single mutex on the map or on the blocks, but as voxel updating is
  // the most expensive operation in integration and most voxels are close
  // together, both strategies would bottleneck the system. We could make a
  // mutex per voxel, but this is too ram heavy as one mutex = 40 bytes.
  // Because of this we create an array that is indexed by the first n bits of
  // the voxels hash. Assuming a uniform hash distribution, this means the
  // chance of two threads needing the same lock for unrelated voxels is
  // (num_threads / (2^n)). For 8 threads and 12 bits this gives 0.2%.
  ApproxHashArray<12, std::mutex, GlobalIndex, LongIndexHash> mutexes_;
};

#endif  // TSDF_PLUSPLUS_INTEGRATOR_INTEGRATOR_H_
