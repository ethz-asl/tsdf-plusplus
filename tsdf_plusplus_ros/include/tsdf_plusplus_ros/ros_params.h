// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_ROS_PARAMS_H_
#define TSDF_PLUSPLUS_ROS_PARAMS_H_

inline Map::Config getMapConfigFromRosParam(const ros::NodeHandle& nh_private) {
  Map::Config map_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = map_config.voxel_size;
  int voxels_per_side = map_config.voxels_per_side;
  nh_private.param("voxel_size", voxel_size, voxel_size);
  nh_private.param("voxels_per_side", voxels_per_side, voxels_per_side);

  if (!voxblox::isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR(
        "voxels_per_side must be a power of 2, setting to default value.");
    voxels_per_side = map_config.voxels_per_side;
  }

  map_config.voxel_size = static_cast<float>(voxel_size);
  map_config.voxels_per_side = voxels_per_side;

  return map_config;
}

inline Integrator::Config getIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Integrator::Config integrator_config;

  const Map::Config map_config = getMapConfigFromRosParam(nh_private);
  float truncation_distance_factor = 4.0f;

  double max_weight = integrator_config.max_weight;
  nh_private.param("voxel_carving_enabled",
                   integrator_config.voxel_carving_enabled,
                   integrator_config.voxel_carving_enabled);
  nh_private.param("truncation_distance_factor", truncation_distance_factor,
                   truncation_distance_factor);
  nh_private.param("max_ray_length_m", integrator_config.max_ray_length_m,
                   integrator_config.max_ray_length_m);
  nh_private.param("min_ray_length_m", integrator_config.min_ray_length_m,
                   integrator_config.min_ray_length_m);
  nh_private.param("max_weight", max_weight, max_weight);
  nh_private.param("use_const_weight", integrator_config.use_const_weight,
                   integrator_config.use_const_weight);
  nh_private.param("use_weight_dropoff", integrator_config.use_weight_dropoff,
                   integrator_config.use_weight_dropoff);
  nh_private.param("allow_clear", integrator_config.allow_clear,
                   integrator_config.allow_clear);
  nh_private.param("anti_grazing", integrator_config.enable_anti_grazing,
                   integrator_config.enable_anti_grazing);
  nh_private.param("use_sparsity_compensation_factor",
                   integrator_config.use_sparsity_compensation_factor,
                   integrator_config.use_sparsity_compensation_factor);
  nh_private.param("sparsity_compensation_factor",
                   integrator_config.sparsity_compensation_factor,
                   integrator_config.sparsity_compensation_factor);
  nh_private.param("integration_order_mode",
                   integrator_config.integration_order_mode,
                   integrator_config.integration_order_mode);

  integrator_config.truncation_distance =
      static_cast<float>(truncation_distance_factor) * map_config.voxel_size;
  integrator_config.max_weight = static_cast<float>(max_weight);

  return integrator_config;
}

inline ICP::Config getICPConfigFromRosParam(const ros::NodeHandle& nh_private) {
  ICP::Config icp_config;
  nh_private.param("object_tracking/icp_use_reciprocal_correspondences",
                   icp_config.use_reciprocal_correspondences,
                   icp_config.use_reciprocal_correspondences);
  nh_private.param("object_tracking/icp_max_correspondence_distance",
                   icp_config.max_correspondence_distance,
                   icp_config.max_correspondence_distance);
  nh_private.param("object_tracking/icp_use_symmetric_objective",
                   icp_config.use_symmetric_objective,
                   icp_config.use_symmetric_objective);
  nh_private.param("object_tracking/icp_max_iterations",
                   icp_config.max_iterations, icp_config.max_iterations);
  nh_private.param("object_tracking/icp_transformation_epsilon",
                   icp_config.transformation_epsilon,
                   icp_config.transformation_epsilon);
  nh_private.param("object_tracking/icp_absolute_mse", icp_config.absolute_mse,
                   icp_config.absolute_mse);
  nh_private.param("object_tracking/icp_euclidean_fitness_epsilon",
                   icp_config.euclidean_fitness_epsilon,
                   icp_config.euclidean_fitness_epsilon);

  return icp_config;
}

inline MOMeshIntegrator::Config getMeshIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  MOMeshIntegrator::Config mesh_integrator_config;

  nh_private.param("mesh_min_weight", mesh_integrator_config.min_weight,
                   mesh_integrator_config.min_weight);
  nh_private.param("mesh_use_color", mesh_integrator_config.use_color,
                   mesh_integrator_config.use_color);

  nh_private.param("using_ground_truth_segmentation",
                   mesh_integrator_config.using_ground_truth_segmentation,
                   mesh_integrator_config.using_ground_truth_segmentation);

  return mesh_integrator_config;
}

#endif  // TSDF_PLUSPLUS_ROS_PARAMS_H_
