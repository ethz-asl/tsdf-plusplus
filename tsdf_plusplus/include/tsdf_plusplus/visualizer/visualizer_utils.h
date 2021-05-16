/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TSDF_PLUSPLUS_VISUALIZER_VISUALIZER_UTILS_H_
#define TSDF_PLUSPLUS_VISUALIZER_VISUALIZER_UTILS_H_

inline void computeCameraParams(const Eigen::Matrix3f& intrinsics,
                                const Eigen::Matrix4f& extrinsics,
                                pcl::visualization::Camera* camera) {
  // Position = extrinsic translation.
  Eigen::Vector3f pos_vec = extrinsics.block<3, 1>(0, 3);

  // Rotate the view vector.
  Eigen::Matrix3f rotation = extrinsics.block<3, 3>(0, 0);
  Eigen::Vector3f y_axis(0.f, 1.f, 0.f);
  Eigen::Vector3f up_vec(rotation * y_axis);

  // Compute the new focal point.
  Eigen::Vector3f z_axis(0.f, 0.f, 1.f);
  Eigen::Vector3f focal_vec = pos_vec + rotation * z_axis;

  // Get the width and height of the image - assume the
  // calibrated centers are at the center of the image.
  Eigen::Vector2i window_size;
  window_size[0] = 2 * static_cast<int>(intrinsics(0, 2));
  window_size[1] = 2 * static_cast<int>(intrinsics(1, 2));

  // Compute the vertical field of view based
  // on the focal length and image  height.
  double fovy = 2 * std::atan(window_size[1] / (2. * intrinsics(1, 1)));

  camera->pos[0] = pos_vec[0];
  camera->pos[1] = pos_vec[1];
  camera->pos[2] = pos_vec[2];
  camera->focal[0] = focal_vec[0];
  camera->focal[1] = focal_vec[1];
  camera->focal[2] = focal_vec[2];

  // The camera coordinate frame in the PCL visualizer
  // are rotated around the z axis by 180 degrees.
  camera->view[0] = -up_vec[0];
  camera->view[1] = -up_vec[1];
  camera->view[2] = -up_vec[2];

  camera->fovy = fovy;
  camera->clip[0] = 0.01;
  camera->clip[1] = 1000.01;
  camera->window_size[0] = window_size[0];
  camera->window_size[1] = window_size[1];
}

#endif  // TSDF_PLUSPLUS_VISUALIZER_VISUALIZER_UTILS_H_
