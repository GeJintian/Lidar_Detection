cmake_minimum_required(VERSION 3.12)
project(MyExampleProject VERSION 1.0.0 LANGUAGES CXX)

# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(vendor/)

# Add executable
add_executable(example example.cpp dbscan.cpp)
