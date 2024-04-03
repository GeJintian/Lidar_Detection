cmake_minimum_required(VERSION 3.5)
project(clustering)

set(CMAKE_CXX_STANDARD 20)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/utils
${PROJECT_SOURCE_DIR}/include/tf_tracker
${PCL_INCLUDE_DIRS}
${pcl_conversions_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
#${rclcpp_INCLUDE_DIRS}
/usr/include/vtk-9.1/
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/dbscan.cpp
src/utils/visualizer.cpp
src/kf_tracker/CKalmanFilter.cpp
src/kf_tracker/featureDetection.cpp
include/dbscan.hpp
include/nanoflann.hpp
include/kf_tracker/CKalmanFilter.hpp
include/kf_tracker/featureDetection.hpp
include/kf_tracker/tracker.hpp
include/utils/visualizer.hpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs PCL pcl_conversions OpenCV)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# For DBSCAN clustering
add_executable(dbscan_cluster src/lidar_cluster.cpp)
ament_target_dependencies(dbscan_cluster rclcpp sensor_msgs PCL pcl_conversions)
target_link_libraries(dbscan_cluster ${PROJECT_NAME})

# For KDTree clustering + KF tracking
add_executable(kf_tracking src/kf_tracker/tracker.cpp)
ament_target_dependencies(kf_tracking rclcpp sensor_msgs PCL pcl_conversions OpenCV visualization_msgs)
target_link_libraries(kf_tracking ${PROJECT_NAME})

add_executable(merge_node src/merge_node.cpp)
ament_target_dependencies(merge_node rclcpp tf2 tf2_ros tf2_geometry_msgs pcl_conversions PCL sensor_msgs)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS dbscan_cluster kf_tracking merge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()