cmake_minimum_required(VERSION 3.5)
project(your_package_name)

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
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_SOURCE_DIR} SHARED
src/dbscan.cpp
src/utils/visualizer.cpp

include/dbscan.hpp
include/nanoflann.hpp
include/utils/visualizer.hpp
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(clustering src/clustering.cpp)
ament_target_dependencies(lidar_subscriber rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  lidar_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()