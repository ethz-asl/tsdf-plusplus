// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef TSDF_PLUSPLUS_ROS_CONTROLLER_H_
#define TSDF_PLUSPLUS_ROS_CONTROLLER_H_

#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tsdf_plusplus/alignment/icp.h>
#include <tsdf_plusplus/core/segment.h>
#include <tsdf_plusplus/integrator/integrator.h>
#include <tsdf_plusplus/mesh/mesh_integrator.h>
#include <tsdf_plusplus/visualizer/visualizer.h>
#include <voxblox/core/common.h>
#include <voxblox/utils/timing.h>

class Controller {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Controller(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  Controller(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const Map::Config& map_config,
             const Integrator::Config& integrator_config,
             const ICP::Config& icp_config,
             const MOMeshIntegrator::Config& mesh_config);

  virtual ~Controller();

  void getConfigFromRosParam(const ros::NodeHandle& nh_private);

 protected:
  void processSegmentPointcloud(
      const sensor_msgs::PointCloud2::Ptr& segment_pcl_msg);

  bool lookupTransformTF(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp, Transformation* transform);

  void integrateFrame();

  void integrateSemanticClasses();

  void trackObjects();

  void exportPoses();

  void publishPointclouds();

  void clearFrame();

  void updateMeshEvent(const ros::TimerEvent& event);

  void segmentPointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& segment_pcl_msg);

  bool generateMeshCallback(std_srvs::Empty::Request& /*request*/,
                            std_srvs::Empty::Response& /*response*/);

  bool saveObjectsCallback(std_srvs::Empty::Request& /*request*/,
                           std_srvs::Empty::Response& /*response*/);

  // Simulate removal of foreground objects, only used
  // for experimental evaluation in the publication.
  bool removeObjectsCallback(std_srvs::Empty::Request& /*request*/,
                             std_srvs::Empty::Response& /*response*/);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Data subscribers.
  ros::Subscriber pointcloud_sub_;

  // Will throttle to this message rate.
  ros::Duration min_time_between_msgs_;

  // Last message time for throttling input and detecting when all segments
  // of the current frame have been received and thus can be integrated.
  ros::Time last_segment_msg_time_;

  uint32_t frame_number_;

  // Global coordinate frame, i.e.the to_frame for all TF transform lookups.
  std::string world_frame_;

  // If set, overwrites the sensor frame of incoming segment pointclouds.
  std::string sensor_frame_;

  // TF listener to lookup TF transforms.
  tf::TransformListener tf_listener_;

  // Last camera-to-global coordinate frame transform.
  voxblox::Transformation T_G_C_;

  // Flag whether ground truth or real-world per-frame segmentation is used.
  bool using_ground_truth_segmentation_;

  // List of segments observed in the current frame.
  std::vector<Segment*> current_frame_segments_;

  // Pairwise overlap (number of points) between segments
  // in the current frame and objects in the map.
  std::map<ObjectID, SegmentHistogram> object_segment_overlap_;

  // Segments assigned to the same object_id during the
  // segmentation propagation step are merged together.
  std::map<ObjectID, Segment*> object_merged_segments_;

  // ICP.
  bool object_tracking_enabled_;
  std::shared_ptr<ICP> icp_;

  // Maps and integrators.
  std::shared_ptr<Map> map_;
  std::unique_ptr<Integrator> integrator_;

  // Mutex to prevent reading from the map layer while it is being updated.
  std::mutex map_mutex_;

  // Semantic classes.
  std::vector<std::string> semantic_classes_;

  // Meshing.
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::unique_ptr<MOMeshIntegrator> mesh_integrator_;
  ros::Timer update_mesh_timer_;
  bool publish_mesh_;

  // Mutex to prevent reading from the mesh_layer while it is being updated.
  std::shared_ptr<std::mutex> mesh_layer_mutex_;
  std::shared_ptr<bool> mesh_layer_updated_;

  // The mesh is written to a file only if mesh_filename_ is non-empty.
  std::string mesh_filename_;

  // PCL visualizer running in a separate thread.
  std::unique_ptr<Visualizer> visualizer_;
  std::thread vizualizer_thread_;
  bool write_frames_to_file_;
  std::string export_path_;

  // Camera parameters used to fit the visualizer
  // window to the current camera view.
  Eigen::Matrix3f camera_intrinsics_;
  std::shared_ptr<Eigen::Matrix4f> camera_extrinsics_;

  // Services.
  ros::ServiceServer generate_mesh_srv_;
  ros::ServiceServer save_objects_srv_;
  ros::ServiceServer move_object_srv_;
  ros::ServiceServer remove_objects_srv_;

  // Publishers.
  ros::Publisher mesh_pub_;
};

#endif  // TSDF_PLUSPLUS_ROS_CONTROLLER_H_
