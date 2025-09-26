cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS date_time)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ggdb")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(NLopt REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  include/plugin
  include/utils
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

set(msg_files
  "msg/Bspline.msg"
)

set(
  PLANNER_INTERFACE
  "planner"
)

rosidl_generate_interfaces(
  ${PLANNER_INTERFACE}
  ${msg_files}
  DEPENDENCIES
  std_msgs
  geometry_msgs
  builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  builtin_interfaces
)

add_executable(
  planner_manager
  src/utils/utility.cpp
  src/plugin/raycast.cpp
  src/plugin/sdfMap.cpp
  src/plugin/astar.cpp
  src/plugin/topo_prm.cpp
  src/plugin/planning_visualization.cpp
  src/plugin/edt_environment.cpp
  src/plugin/OctomapManager.cpp
  src/plugin/polynomial_traj.cpp
  src/plugin/non_uniform_bspline.cpp
  src/plugin/bspline_optimizer_2d.cpp
  src/plugin/bspline_optimizer.cpp
  src/plugin/obj_predictor.cpp
  src/outside_graph.cpp
  src/planner_node.cpp
  src/planner_process.cpp
  src/room_graph.cpp
)

target_link_libraries(
  planner_manager
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${NLOPT_LIBRARIES}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(
  planner_manager
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
  octomap
  octomap_ros
  visualization_msgs  
)

rosidl_get_typesupport_target(
    cpp_typesupport_target 
    ${PLANNER_INTERFACE} 
    "rosidl_typesupport_cpp"
)
target_link_libraries(
  planner_manager 
  "${cpp_typesupport_target}"
)

add_executable(
  test_octomap
  test/test_octomap.cpp
  src/plugin/OctomapManager.cpp
)

target_link_libraries(
  test_octomap
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(
  test_octomap
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  octomap
  cv_bridge
  octomap_ros
  visualization_msgs  
)

add_executable(
  test_room_graph
  test/test_graph.cpp
  src/room_graph.cpp
)

target_link_libraries(
  test_room_graph
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(
  test_room_graph
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  octomap
  octomap_ros
  cv_bridge
  visualization_msgs  
)

add_executable(
  traj_server
  src/traj_server.cpp
  src/plugin/non_uniform_bspline.cpp
)

target_link_libraries(
  traj_server
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

target_link_libraries(traj_server "${cpp_typesupport_target}")

ament_target_dependencies(
  traj_server
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
  octomap
  octomap_ros
  visualization_msgs  
)

add_executable(
  test_sdfmap
  test/test_sdfmap.cpp
)

target_link_libraries(
  test_sdfmap
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(
  test_sdfmap
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
  octomap
  octomap_ros
  visualization_msgs  
)

add_executable(
  test_bspline
  test/test_bspline.cpp
  src/plugin/non_uniform_bspline.cpp
  src/plugin/raycast.cpp
  src/plugin/sdfMap.cpp
  src/plugin/bspline_optimizer.cpp
  src/plugin/edt_environment.cpp
  src/plugin/OctomapManager.cpp
  src/plugin/polynomial_traj.cpp
)

target_link_libraries(
  test_bspline
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${NLOPT_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries(test_bspline "${cpp_typesupport_target}")


ament_target_dependencies(
  test_bspline
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
  octomap
  octomap_ros
  visualization_msgs  
)

add_executable(
  test_path_publisher
  test/test_path_publisher.cpp
)

target_link_libraries(
  test_path_publisher
  Boost::date_time
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(
  test_path_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
  octomap
  octomap_ros
  visualization_msgs  
)

install(
  TARGETS test_path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_octomap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS traj_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_room_graph
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_bspline
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS planner_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_sdfmap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config map launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_package()
