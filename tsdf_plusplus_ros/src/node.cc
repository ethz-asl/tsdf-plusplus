#include <gflags/gflags.h>
#include <glog/logging.h>

#include "tsdf_plusplus_ros/controller.h"

int main(int argc, char** argv) {
  std::cout << std::endl
            << "TSDF++ Copyright (c) 2020- Margarita Grinvald, Autonomous "
               "Systems Lab, ETH Zurich."
            << std::endl
            << std::endl;

  ros::init(argc, argv, "tsdf_plusplus_node");
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  Controller controller(node_handle, node_handle_private);

  // Spinner with a number of threads equal to the number of cores.
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
