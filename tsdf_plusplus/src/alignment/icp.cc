#include "tsdf_plusplus/alignment/icp.h"

#include <glog/logging.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include "tsdf_plusplus/alignment/icp_utils.h"

ICP::ICP(Config config) : config_(config) {}

pcl::IterativeClosestPointWithNormals<PointTypeNormal, PointTypeNormal>
ICP::init() {
  pcl::IterativeClosestPointWithNormals<PointTypeNormal, PointTypeNormal> icp;

  icp.setUseReciprocalCorrespondences(config_.use_reciprocal_correspondences);
  icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);

  pcl::registration::TransformationEstimationPointToPlane<PointTypeNormal,
                                                          PointTypeNormal>::Ptr
      transformation_estimation(
          new pcl::registration::TransformationEstimationPointToPlane<
              PointTypeNormal, PointTypeNormal>);
  icp.setTransformationEstimation(transformation_estimation);

  icp.setUseSymmetricObjective(config_.use_symmetric_objective);

  icp.setMaximumIterations(config_.max_iterations);
  icp.getConvergeCriteria()->setAbsoluteMSE(config_.absolute_mse);
  icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  icp.setTransformationEpsilon(config_.transformation_epsilon);

  return icp;
}

// Point-to-plane ICP alignment with normals.
bool ICP::align(const pcl::PointCloud<PointTypeNormal>::Ptr source_cloud,
                const pcl::PointCloud<PointTypeNormal>::Ptr target_cloud,
                const Eigen::Matrix4f& guess,
                Eigen::Matrix4f* transformation_matrix_float) {
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  pcl::IterativeClosestPointWithNormals<PointTypeNormal, PointTypeNormal> icp_ =
      init();

  icp_.setMaximumIterations(config_.max_iterations);
  icp_.setInputSource(source_cloud);
  icp_.setInputTarget(target_cloud);

  int iterations = config_.max_iterations;

  pcl::PointCloud<PointTypeNormal>::Ptr aligned_source(
      new pcl::PointCloud<PointTypeNormal>);

  bool success = false;

  icp_.align(*aligned_source, guess);
  success =
      checkConvergenceState(icp_.getConvergeCriteria()->getConvergenceState());

  transformation_matrix = icp_.getFinalTransformation().cast<double>();

  *transformation_matrix_float = transformation_matrix.cast<float>();
  *source_cloud = *aligned_source;
  return success;
}
