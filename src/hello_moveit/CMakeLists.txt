cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Define first executable
add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Define second executable
add_executable(gen3_lite_hello_moveit src/gen3_lite_hello_moveit.cpp)
target_include_directories(gen3_lite_hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(gen3_lite_hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  gen3_lite_hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Define second executable
add_executable(pose_monitor src/pose_monitor.cpp)
target_include_directories(pose_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pose_monitor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pose_monitor
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Install both executables
install(TARGETS hello_moveit gen3_lite_hello_moveit pose_monitor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
