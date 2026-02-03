cmake_minimum_required(VERSION 3.8)
project(hdl_people_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters kdtree search sample_consensus segmentation extract)
find_package(OpenCV REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cluster.msg"
  "msg/ClusterArray.msg"
  "msg/Track.msg"
  "msg/TrackArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(hdl_people_tracking_component SHARED
  apps/hdl_people_detection_nodelet.cpp
  apps/hdl_people_tracking_nodelet.cpp
  src/kidono_human_classifier.cpp
  src/people_detector.cpp
)
target_compile_features(hdl_people_tracking_component PUBLIC cxx_std_14)
target_link_libraries(hdl_people_tracking_component
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(hdl_people_tracking_component
  rclcpp
  rclcpp_components
  pcl_conversions
  pcl_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hdl_people_tracking_component "${cpp_typesupport_target}")

rclcpp_components_register_nodes(hdl_people_tracking_component
  "hdl_people_tracking::HdlPeopleDetectionNode"
  "hdl_people_tracking::HdlPeopleTrackingNode"
)

install(TARGETS hdl_people_tracking_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hdl_people_tracking_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  pcl_conversions
  pcl_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
)

ament_package()