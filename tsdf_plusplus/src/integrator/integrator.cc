// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "tsdf_plusplus/integrator/integrator.h"

#include <pcl/common/centroid.h>
#include <voxblox/core/voxel.h>

Integrator::Integrator(const Config& config, std::shared_ptr<Map> map)
    : config_(config),
      map_(map.get()),
      highest_object_id_(map->getHighestObjectIdPtr()) {
  voxel_size_ = map_->getMapLayerPtr()->voxel_size();
  block_size_ = map_->getMapLayerPtr()->block_size();
  voxels_per_side_ = map_->getMapLayerPtr()->voxels_per_side();

  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_inv_ = 1.0 / block_size_;
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;

  if (config_.integrator_threads == 0) {
    LOG(WARNING) << "Automatic core count failed, defaulting to 1 thread.";
    config_.integrator_threads = 1;
  }
  // Clearing rays have no utility if voxel_carving is disabled.
  if (config_.allow_clear && !config_.voxel_carving_enabled) {
    config_.allow_clear = false;
  }
}

void Integrator::computeObjectOverlap(
    Segment* segment,
    std::map<ObjectID, SegmentHistogram>* object_segment_overlap) {
  CHECK_NOTNULL(segment);
  CHECK_NOTNULL(object_segment_overlap);

  bool exists_overlapping_object = false;

  for (const Point& point_C : segment->points_C_) {
    const Point point_G = segment->T_G_C_ * point_C;

    // Get the corresponding voxel by 3D position in world frame.
    Layer<MOVoxel>::BlockType::ConstPtr mo_block_ptr =
        map_->getMapLayerPtr()->getBlockPtrByCoordinates(point_G);

    if (mo_block_ptr) {
      // Get the id of the currently active object at this 3D position.
      const MOVoxel& mo_voxel = mo_block_ptr->getVoxelByCoordinates(point_G);
      ObjectID object_id = mo_voxel.active_object.object_id;

      if (object_id != EmptyID) {
        exists_overlapping_object = true;

        // Increase the overlap point count for this object-segment pair.
        constexpr size_t count = 1u;
        increaseOverlapCount(object_id, segment, count, object_segment_overlap);
      }
    }
  }

  // Segments that did not overlap with any object in the map (i.e. are
  // previously unobserved) are assigned a new, previously unseen object_id.
  if (!exists_overlapping_object) {
    ObjectID object_id = getFreshObjectId();
    const size_t count = segment->points_C_.size();
    increaseOverlapCount(object_id, segment, count, object_segment_overlap);
  }
}

void Integrator::increaseOverlapCount(
    const ObjectID object_id, Segment* segment, size_t count,
    std::map<ObjectID, SegmentHistogram>* object_segment_overlap) {
  CHECK_NOTNULL(segment);
  CHECK_NOTNULL(object_segment_overlap);

  auto object_it = object_segment_overlap->find(object_id);

  if (object_it != object_segment_overlap->end()) {
    auto segment_it = object_it->second.find(segment);
    if (segment_it != object_it->second.end()) {
      ++segment_it->second;
    } else {
      object_it->second.emplace(segment, count);
    }
  } else {
    SegmentHistogram segment_histogram;
    segment_histogram.emplace(segment, count);
    object_segment_overlap->emplace(object_id, segment_histogram);
  }
}

void Integrator::assignObjectIds(
    std::vector<Segment*>* current_frame_segments,
    std::map<ObjectID, SegmentHistogram>* object_segment_overlap,
    std::map<ObjectID, Segment*>* object_merged_segments) {
  CHECK_NOTNULL(object_segment_overlap);

  std::set<Segment*> assigned_segments;
  std::pair<Segment*, ObjectID> segment_object_pair;

  while (nextSegmentObjectPair(*object_segment_overlap, assigned_segments,
                               &segment_object_pair)) {
    Segment* segment = segment_object_pair.first;
    CHECK_NOTNULL(segment);
    ObjectID object_id = segment_object_pair.second;

    auto it = object_merged_segments->find(object_id);
    if (it != object_merged_segments->end()) {
      it->second->pointcloud_ += segment->pointcloud_;
    } else {
      segment->object_id_ = object_id;
      object_merged_segments->emplace(object_id, segment);
    }

    // The segment has been assigned an object_id from the map.
    assigned_segments.emplace(segment);

    ObjectVolume* object_volume = map_->getObjectVolumePtrById(object_id);

    if (object_volume) {
      if (segment->semantic_class_ == BackgroundClass &&
          object_volume->getSemanticClass() == BackgroundClass) {
        // This object_id can no longer be assigned
        // to another segment in the current frame.
        object_segment_overlap->erase(object_id);
      }
    }
  }

  // All segments which have not been assigned an object_id among their
  // overlapping map objects are assigned a new, previously unseen object_id.
  for (Segment* segment : *current_frame_segments) {
    if (assigned_segments.find(segment) == assigned_segments.end()) {
      segment->object_id_ = getFreshObjectId();
      assigned_segments.emplace(segment);
      object_merged_segments->emplace(segment->object_id_, segment);
    }
  }
}

bool Integrator::nextSegmentObjectPair(
    const std::map<ObjectID, SegmentHistogram>& object_segment_overlap,
    const std::set<Segment*>& assigned_segments,
    std::pair<Segment*, ObjectID>* segment_object_pair) {
  size_t max_overlap_count = 0u;
  ObjectID max_object_id;
  Segment* max_segment;

  for (auto const& object_pair : object_segment_overlap) {
    ObjectID object_id = object_pair.first;
    for (auto const& segment_pair : object_pair.second) {
      Segment* segment = segment_pair.first;
      size_t overlap_count = segment_pair.second;

      float overlap_ratio =
          (float)overlap_count / max_segment->points_C_.size();

      bool is_assigned =
          assigned_segments.find(segment) != assigned_segments.end();
      bool is_greater_than_max = overlap_count > max_overlap_count;
      bool is_greater_than_min = overlap_ratio > config_.min_overlap_ratio;
      is_greater_than_min = true;

      if (!is_assigned && is_greater_than_max && is_greater_than_min) {
        max_overlap_count = overlap_count;
        max_object_id = object_id;
        max_segment = segment;
      }
    }
  }

  if (!max_overlap_count) {
    return false;
  }

  segment_object_pair->first = max_segment;
  segment_object_pair->second = max_object_id;

  return true;
}

void Integrator::integrateSegment(const Segment& segment) {
  timing::Timer integrate_segment_timer("integrate/segment");
  CHECK_EQ(segment.points_C_.size(), segment.colors_.size());

  // Pre-compute a list of unique voxels to end on.
  // Create a hashmap: VOXEL INDEX -> index in original cloud.
  LongIndexHashMapType<AlignedVector<size_t>>::type voxel_map;
  // This is a hash map (same as above) to all the indices that need to be
  // cleared.
  LongIndexHashMapType<AlignedVector<size_t>>::type clear_map;

  std::unique_ptr<ThreadSafeIndex> index_getter(ThreadSafeIndexFactory::get(
      config_.integration_order_mode, segment.points_C_));

  timing::Timer bundle_timer("integrate/1_bundle_rays");

  bundleRays(segment.T_G_C_, segment.points_C_, index_getter.get(), &voxel_map,
             &clear_map);

  bundle_timer.Stop();

  timing::Timer integrate_rays_timer("integrate/2_integrate_rays");

  bool is_clearing_ray = false;
  integrateRays(segment.T_G_C_, segment.points_C_, segment.centroid_,
                segment.object_id_, segment.semantic_class_, segment.colors_,
                config_.enable_anti_grazing, is_clearing_ray, voxel_map,
                clear_map);

  integrate_rays_timer.Stop();

  timing::Timer clear_timer("integrate/3_clear");

  is_clearing_ray = true;
  integrateRays(segment.T_G_C_, segment.points_C_, segment.centroid_,
                segment.object_id_, segment.semantic_class_, segment.colors_,
                config_.enable_anti_grazing, is_clearing_ray, voxel_map,
                clear_map);

  clear_timer.Stop();

  integrate_segment_timer.Stop();
}

void Integrator::bundleRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    ThreadSafeIndex* index_getter,
    LongIndexHashMapType<AlignedVector<size_t>>::type* voxel_map,
    LongIndexHashMapType<AlignedVector<size_t>>::type* clear_map) {
  CHECK(voxel_map != nullptr);
  CHECK(clear_map != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point& point_C = points_C[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, &is_clearing)) {
      continue;
    }

    const Point point_G = T_G_C * point_C;

    GlobalIndex voxel_index =
        getGridIndexFromPoint<GlobalIndex>(point_G, voxel_size_inv_);

    if (is_clearing) {
      (*clear_map)[voxel_index].push_back(point_idx);
    } else {
      (*voxel_map)[voxel_index].push_back(point_idx);
    }
  }

  VLOG(3) << "Went from " << points_C.size() << " points to "
          << voxel_map->size() << " raycasts  and " << clear_map->size()
          << " clear rays.";
}

void Integrator::integrateRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Point centroid, const ObjectID& object_id,
    const SemanticClass& semantic_class, const Colors& colors,
    bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map) {
  // If only 1 thread just do function call, otherwise spawn threads.
  if (config_.integrator_threads == 1) {
    constexpr size_t thread_idx = 0u;
    integrateVoxels(T_G_C, points_C, centroid, object_id, semantic_class,
                    colors, enable_anti_grazing, clearing_ray, voxel_map,
                    clear_map, thread_idx);
  } else {
    std::list<std::thread> integration_threads;

    for (size_t i = 0u; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &Integrator::integrateVoxels, this, T_G_C, std::cref(points_C),
          centroid, object_id, semantic_class, std::cref(colors),
          enable_anti_grazing, clearing_ray, std::cref(voxel_map),
          std::cref(clear_map), i);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  timing::Timer insertion_timer("integrate/insert_blocks");
  updateLayerWithStoredBlocks();

  insertion_timer.Stop();
}

void Integrator::integrateVoxels(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Point centroid, const ObjectID& object_id,
    const SemanticClass& semantic_class, const Colors& colors,
    bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
    size_t thread_idx) {
  LongIndexHashMapType<AlignedVector<size_t>>::type::const_iterator it;
  size_t map_size;
  if (clearing_ray) {
    it = clear_map.begin();
    map_size = clear_map.size();
  } else {
    it = voxel_map.begin();
    map_size = voxel_map.size();
  }

  for (size_t i = 0u; i < map_size; ++i) {
    if (((i + thread_idx + 1u) % config_.integrator_threads) == 0u) {
      integrateVoxel(T_G_C, points_C, centroid, object_id, semantic_class,
                     colors, enable_anti_grazing, clearing_ray, *it, voxel_map);
    }
    ++it;
  }
}

void Integrator::integrateVoxel(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Point centroid, const ObjectID& object_id,
    const SemanticClass& semantic_class, const Colors& colors,
    bool enable_anti_grazing, bool clearing_ray,
    const std::pair<GlobalIndex, AlignedVector<size_t>>& kv,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map) {
  if (kv.second.empty()) {
    return;
  }

  const Point& origin = T_G_C.getPosition();
  Color merged_color;
  Point merged_point_C = Point::Zero();
  FloatingPoint merged_weight = 0.0;
  // TODO(margaritaG): could implement a histogram here
  // and select the max occurring object_id.
  ObjectID merged_object_id = object_id;

  for (const size_t pt_idx : kv.second) {
    const Point& point_C = points_C[pt_idx];
    const Color& color = colors[pt_idx];

    const float point_weight = getVoxelWeight(point_C);
    if (point_weight < kEpsilon) {
      continue;
    }
    merged_point_C = (merged_point_C * merged_weight + point_C * point_weight) /
                     (merged_weight + point_weight);
    merged_color =
        Color::blendTwoColors(merged_color, merged_weight, color, point_weight);
    merged_weight += point_weight;

    // Only take the first point when clearing.
    if (clearing_ray) {
      break;
    }
  }

  const Point merged_point_G = T_G_C * merged_point_C;

  RayCaster ray_caster(origin, merged_point_G, clearing_ray,
                       config_.voxel_carving_enabled, config_.max_ray_length_m,
                       voxel_size_inv_, config_.truncation_distance);

  Block<MOVoxel>::Ptr block = nullptr;
  BlockIndex block_idx;

  ObjectID last_object_id;
  ObjectVolume* last_object_volume = nullptr;

  // The pair block_idx and block of either layer must be updated together,
  // else it is invalid. Therefore, keep a separate pair for the TsdfVoxel.
  Block<TsdfVoxel>::Ptr tsdf_block = nullptr;
  BlockIndex last_tsdf_block_idx;

  GlobalIndex global_voxel_idx;
  while (ray_caster.nextRayIndex(&global_voxel_idx)) {
    if (enable_anti_grazing) {
      // Check if this one is already the the block hash map for this
      // insertion. Skip this to avoid grazing.
      if ((clearing_ray || global_voxel_idx != kv.first) &&
          voxel_map.find(global_voxel_idx) != voxel_map.end()) {
        continue;
      }
    }

    MOVoxel* voxel =
        allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

    updateMOVoxel(centroid, semantic_class, origin, merged_point_G,
                  merged_object_id, global_voxel_idx, merged_color,
                  merged_weight, voxel, &last_object_volume, &last_object_id,
                  &tsdf_block, &last_tsdf_block_idx);
  }
}

MOVoxel* Integrator::allocateStorageAndGetVoxelPtr(
    const GlobalIndex& global_voxel_idx, Block<MOVoxel>::Ptr* last_block,
    BlockIndex* last_block_idx) {
  CHECK_NOTNULL(last_block);
  CHECK_NOTNULL(last_block_idx);

  const BlockIndex block_idx =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);

  if ((block_idx != *last_block_idx) || (*last_block == nullptr)) {
    *last_block = map_->getMapLayerPtr()->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  // If no block at this location currently exists, we allocate a temporary
  // voxel that will be merged into the map later
  if (*last_block == nullptr) {
    // To allow temp_block_map_ to grow we can only let one thread in at once
    std::lock_guard<std::mutex> lock(temp_block_mutex_);

    typename Layer<MOVoxel>::BlockHashMap::iterator it =
        temp_block_map_.find(block_idx);
    if (it != temp_block_map_.end()) {
      *last_block = it->second;
    } else {
      auto insert_status = temp_block_map_.emplace(
          block_idx, std::make_shared<Block<MOVoxel>>(
                         voxels_per_side_, voxel_size_,
                         getOriginPointFromGridIndexD(block_idx, block_size_)));

      CHECK(insert_status.second) << "Block already exists when allocating at "
                                  << block_idx.transpose();

      *last_block = insert_status.first->second;
    }
  }

  (*last_block)->updated().set();

  const VoxelIndex local_voxel_idx =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

  return &((*last_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

// NOT thread safe.
void Integrator::updateLayerWithStoredBlocks() {
  BlockIndex last_block_idx;
  Block<MOVoxel>::Ptr block = nullptr;

  for (const std::pair<const BlockIndex, Block<MOVoxel>::Ptr>& temp_block_pair :
       temp_block_map_) {
    map_->getMapLayerPtr()->insertBlock(temp_block_pair);
  }

  temp_block_map_.clear();

  auto object_volumes = map_->getObjectVolumesPtr();

  for (auto pair : *object_volumes) {
    ObjectVolume* object_volume = pair.second;
    object_volume->updateLayerWithStoredBlocks();
  }
}

// Updates map layer voxel and corresponding object volume voxel. Thread safe.
void Integrator::updateMOVoxel(
    const Point centroid, const SemanticClass& semantic_class,
    const Point& origin, const Point& point_G, const ObjectID& object_id,
    const GlobalIndex& global_voxel_idx, const Color& color, const float weight,
    MOVoxel* mo_voxel, ObjectVolume** last_object_volume,
    ObjectID* last_object_id, Block<TsdfVoxel>::Ptr* last_tsdf_block,
    BlockIndex* last_tsdf_block_idx) {
  CHECK(mo_voxel != nullptr);

  const Point voxel_center =
      getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

  const float sdf = computeDistance(origin, point_G, voxel_center);

  float updated_weight = weight;
  // Compute updated weight in case we use weight dropoff. It's easier here
  // that in getVoxelWeight as here we have the actual SDF for the voxel
  // already computed.
  const FloatingPoint dropoff_epsilon = voxel_size_;
  if (config_.use_weight_dropoff && sdf < -dropoff_epsilon) {
    updated_weight = weight * (config_.truncation_distance + sdf) /
                     (config_.truncation_distance - dropoff_epsilon);
    updated_weight = std::max(updated_weight, 0.0f);
  }

  // Compute the updated weight in case we compensate for sparsity. By
  // multiplicating the weight of occupied areas (|sdf| < truncation distance)
  // by a factor, we prevent to easily fade out these areas with the free
  // space parts of other rays which pass through the corresponding voxels.
  // This can be useful for creating a TSDF map from sparse sensor data (e.g.
  // visual features from a SLAM system). By default, this option is disabled.
  if (config_.use_sparsity_compensation_factor) {
    if (std::abs(sdf) < config_.truncation_distance) {
      updated_weight *= config_.sparsity_compensation_factor;
    }
  }

  // Lookup the mutex that is responsible for this voxel and lock it.
  std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));

  if (mo_voxel->active_object.object_id == 0u &&
      mo_voxel->active_object.confidence != 0u) {
    LOG(FATAL)
        << "Encountered voxel which is set to object 0 but has confidence "
        << mo_voxel->active_object.confidence;
  }

  CHECK(mo_voxel->active_object.object_id != 0u ||
        mo_voxel->active_object.confidence == 0u);

  // Do the confidence increase/decrease and decide which
  // object_id this voxel is assigned after this update.
  if (mo_voxel->active_object.confidence == 0u) {
    mo_voxel->active_object.object_id = object_id;
    mo_voxel->active_object.confidence = 1;
  } else {
    if (mo_voxel->active_object.object_id == object_id) {
      ++mo_voxel->active_object.confidence;
    } else {
      --mo_voxel->active_object.confidence;
    }
  }

  if (mo_voxel->active_object.object_id == 0u) {
    LOG(FATAL) << "Confidence got to: " << mo_voxel->active_object.confidence;
  }

  // TODO(margaritaG): experiment with mo_voxel->active_object.object_id for
  // real-world segmentation.
  TsdfVoxel* tsdf_voxel = map_->allocateStorageAndGetVoxelPtr(
      centroid, semantic_class, object_id, global_voxel_idx, last_object_volume,
      last_object_id, last_tsdf_block, last_tsdf_block_idx);

  const float new_weight = tsdf_voxel->weight + updated_weight;

  // It is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  if (new_weight < kFloatEpsilon) {
    return;
  }

  const float new_sdf =
      (sdf * updated_weight + tsdf_voxel->distance * tsdf_voxel->weight) /
      new_weight;

  // Color blending is expensive only do it close to the surface
  if (std::abs(sdf) < config_.truncation_distance) {
    tsdf_voxel->color = Color::blendTwoColors(
        tsdf_voxel->color, tsdf_voxel->weight, color, updated_weight);
  }
  tsdf_voxel->distance = (new_sdf > 0.0)
                             ? std::min(config_.truncation_distance, new_sdf)
                             : std::max(-config_.truncation_distance, new_sdf);
  tsdf_voxel->weight = std::min(config_.max_weight, new_weight);
}

// Thread safe.
// Figure out whether the voxel is behind or in front of the surface.
// To do this, project the voxel_center onto the ray from origin to point G.
// Then check if the the magnitude of the vector is smaller or greater than
// the original distance.
float Integrator::computeDistance(const Point& origin, const Point& point_G,
                                  const Point& voxel_center) const {
  const Point v_voxel_origin = voxel_center - origin;
  const Point v_point_origin = point_G - origin;

  const float dist_G = v_point_origin.norm();
  // Projection of a (v_voxel_origin) onto b (v_point_origin).
  const float dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

  const float sdf = static_cast<float>(dist_G - dist_G_V);
  return sdf;
}

// Thread safe.
float Integrator::getVoxelWeight(const Point& point_C) const {
  if (config_.use_const_weight) {
    return 1.0f;
  }
  const FloatingPoint dist_z = std::abs(point_C.z());
  if (dist_z > kEpsilon) {
    return 1.0f / (dist_z * dist_z);
  }
  return 0.0f;
}
