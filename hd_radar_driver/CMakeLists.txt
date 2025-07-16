cmake_minimum_required(VERSION 3.8)
project(hd_radar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hd_radar_interfaces REQUIRED)

add_executable(
  hd_radar_node src/hd_radar_node.cpp
  src/hd_radar_driver.cpp
  src/modules/hd_radar_pcl.cpp
  src/modules/hd_radar_raw.cpp
  src/modules/hd_radar_heat.cpp
  src/modules/hd_radar_can.cpp
  )
ament_target_dependencies(
  hd_radar_node
  rclcpp
  std_srvs
  sensor_msgs
  cv_bridge
  hd_radar_interfaces
  )

ament_export_dependencies(
  rclcpp
  sensor_msgs
)

target_link_libraries(hd_radar_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

install(TARGETS 
  hd_radar_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch config rviz rqt
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
  )
ament_export_include_directories(include)

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
