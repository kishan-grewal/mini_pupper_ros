cmake_minimum_required(VERSION 3.8)
project(mini_pupper_tracking_cpp)
find_package(tf2 REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mini_pupper_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# orientation_node ------------
add_executable(orientation_node
  src/orientation_node.cpp
  src/orientation_node_main.cpp
)

target_include_directories(orientation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(orientation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mini_pupper_interfaces
  tf2
)

install(TARGETS orientation_node
  DESTINATION lib/${PROJECT_NAME}
)

# pose_node ------------
add_executable(pose_node
  src/pose_node.cpp
  src/pose_node_main.cpp
)

target_include_directories(pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mini_pupper_interfaces
  tf2
)

install(TARGETS pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# lie_imu_node ------------
add_executable(lie_imu_node
  src/lie_imu_node.cpp
  src/lie_imu_node_main.cpp
)

target_include_directories(lie_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(lie_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mini_pupper_interfaces
  tf2
)

install(TARGETS lie_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()