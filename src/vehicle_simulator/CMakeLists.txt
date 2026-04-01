cmake_minimum_required(VERSION 3.8)
project(vehicle_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(
  vehicleSimulator 
  src/main.cpp
  src/vehicleSimulator.cpp
)

ament_target_dependencies(
  vehicleSimulator
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  gazebo_msgs
  gazebo_ros
  message_filters
  OpenCV
  PCL
  pcl_conversions
  pcl_ros
)

target_link_libraries(vehicleSimulator
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  vehicleSimulator
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch mesh urdf world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
