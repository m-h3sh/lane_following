cmake_minimum_required(VERSION 3.8)
project(model_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(pcl REQUIRED)
# find_package(pcl_conversions REQUIRED)

add_executable(lane_follower src/lane_follower.cpp)
target_link_libraries(lane_follower
  ${OpenCV_LIBRARIES}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY launch model config src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lane_follower
  
  # lane_follower_2
  DESTINATION lib/${PROJECT_NAME})

# add_executable(lane_frame_transform src/lane_frame_transform.py)

# HLEOIHNOGFSISHUBDFOGI TYPE THE PACKAGES REQUIRED IN LANEFOLLOWER HERE
ament_target_dependencies(lane_follower visualization_msgs rclcpp geometry_msgs sensor_msgs OpenCV Eigen3 std_msgs sensor_msgs cv_bridge)

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
