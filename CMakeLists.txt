cmake_minimum_required(VERSION 3.14)
project(pcl_div)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/${PROJECT_NAME}_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "pcl_div::PclNode"
  EXECUTABLE ${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
