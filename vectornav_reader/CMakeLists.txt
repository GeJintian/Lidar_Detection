cmake_minimum_required(VERSION 3.5)
project(vectornavreader)

set(CMAKE_CXX_STANDARD 20)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/vectornav_reader
  ${vectornav_msgs_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
include/vectornav_reader/reader.hpp
)

set_target_properties(vectornavreader PROPERTIES LINKER_LANGUAGE CXX)
# ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs vectornav_msgs)
# target_link_libraries(${PROJECT_NAME})

add_executable(reader src/reader.cpp)
ament_target_dependencies(reader rclcpp vectornav_msgs sensor_msgs)
# target_link_libraries(reader ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  reader
  DESTINATION lib/${PROJECT_NAME})

ament_package()
