cmake_minimum_required(VERSION 3.0.2)
project(lidar_curb_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  visualization_msgs
  lidar_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(curb_detection src/main.cpp src/curb_detection.cpp)
add_dependencies(curb_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(curb_detection ${catkin_LIBRARIES})
