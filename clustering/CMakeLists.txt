cmake_minimum_required(VERSION 3.5)
project(clustering)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/utils
${PCL_INCLUDE_DIRS}
${pcl_conversions_INCLUDE_DIRS}
#${rclcpp_INCLUDE_DIRS}
/usr/include/vtk-9.1/
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
#src/dbscan.cpp
src/utils/visualizer.cpp
#include/dbscan.hpp
#include/nanoflann.hpp
include/utils/visualizer.hpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs PCL pcl_conversions)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(lidar_cluster src/lidar_cluster.cpp)
ament_target_dependencies(lidar_cluster rclcpp sensor_msgs PCL pcl_conversions)
target_link_libraries(lidar_cluster ${PROJECT_NAME})

install(TARGETS
  lidar_cluster
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()