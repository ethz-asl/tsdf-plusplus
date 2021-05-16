#ifndef TSDF_PLUSPLUS_ALIGNMENT_ICP_UTILS_H_
#define TSDF_PLUSPLUS_ALIGNMENT_ICP_UTILS_H_

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>

using ConvergenceState =
    pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState;

inline bool checkConvergenceState(ConvergenceState state) {
  bool success = false;

  if (state == ConvergenceState::CONVERGENCE_CRITERIA_NOT_CONVERGED ||
      state == ConvergenceState::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES ||
      state == ConvergenceState::CONVERGENCE_CRITERIA_ITERATIONS) {
    LOG(INFO) << "\nICP has NOT CONVERGED. ";
  } else {
    success = true;
  }

  return success;
}

#endif  // TSDF_PLUSPLUS_ALIGNMENT_ICP_UTILS_H_
