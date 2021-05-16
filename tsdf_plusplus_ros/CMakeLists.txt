cmake_minimum_required(VERSION 2.8.3)
project(tsdf_plusplus_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_library
  src/controller.cc
)
target_link_libraries(${PROJECT_NAME}_library ${catkin_LIBRARIES} )

cs_add_executable(tsdf_plusplus_node
  src/node.cc
)
target_link_libraries(tsdf_plusplus_node ${PROJECT_NAME}_library)

cs_install()
cs_export()
