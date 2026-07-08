cmake_minimum_required(VERSION 3.22)
# C is required by rosidl interface generation (srv/SetVirtualCam.srv); CXX for the node.
project(micropilot_rendering_node LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Non-ROS rendering lib (resolved via CMAKE_PREFIX_PATH, not rosdep) ──────
find_package(micropilot_rendering REQUIRED)

# ── ROS2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Service interface ─────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetVirtualCam.srv)

set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  nav_msgs
  cv_bridge
  OpenCV)

# ── Node library ─────────────────────────────────────────────────────────────
add_library(rendering_node_lib SHARED
  src/rendering_node.cpp)

target_include_directories(rendering_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rendering_node_lib ${THIS_PACKAGE_DEPS})
target_link_libraries(rendering_node_lib micropilot_rendering::reprojector)

# Link this package's generated service typesupport (SetVirtualCam) into the lib.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(rendering_node_lib "${cpp_typesupport_target}")

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(rendering_node src/main.cpp)
target_link_libraries(rendering_node rendering_node_lib)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS rendering_node_lib rendering_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
