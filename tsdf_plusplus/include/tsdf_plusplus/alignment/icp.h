#ifndef TSDF_PLUSPLUS_ALIGNMENT_ICP_H_
#define TSDF_PLUSPLUS_ALIGNMENT_ICP_H_

#include <pcl/registration/gicp.h>

#include "tsdf_plusplus/core/common.h"

class ICP {
 public:
  struct Config {
    bool point_to_plane = false;

    bool use_reciprocal_correspondences = false;
    double max_correspondence_distance =
        std::sqrt(std::numeric_limits<double>::max());

    bool use_symmetric_objective = false;

    int max_iterations = 10;
    double absolute_mse = 1e-12;
    double euclidean_fitness_epsilon = -std::numeric_limits<double>::max();
    double transformation_epsilon = 0.0;
  };

  ICP(Config config);

  pcl::IterativeClosestPointWithNormals<PointTypeNormal, PointTypeNormal>
  init();

  void setMaximumIterations(int max_iterations);

  bool align(const pcl::PointCloud<PointTypeNormal>::Ptr source_cloud,
             const pcl::PointCloud<PointTypeNormal>::Ptr target_cloud,
             const Eigen::Matrix4f& guess,
             Eigen::Matrix4f* transformation_matrix);

 protected:
  Config config_;
};

#endif  // TSDF_PLUSPLUS_ALIGNMENT_ICP_H_
