#ifndef DBSCAN_H
#define DBSCAN_H
#pragma once

#include "nanoflann.hpp"

#include <cassert>
#include <cstddef>
#include <span>
#include <vector>
#include <cstdlib>
#include <type_traits>
#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <fstream>
#include <charconv>
#include <tuple>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


struct point2
{
    float x, y;
};

class point3
{
    public:
        float x, y, z;
        point3(float x,float y,float z):x(x),y(y),z(z){}
};

auto dbscan(const std::span<const point3>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;
void dbscan3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float eps, int min_pts);

// template<size_t dim>
// auto dbscan(const std::span<float>& data, float eps, int min_pts)
// {
//     static_assert(dim == 2 or dim == 3, "This only supports either 2D or 3D points");
//     assert(data.size() % dim == 0);
    
//     if(dim == 2)
//     {
//         auto * const ptr = reinterpret_cast<float const*> (data.data());
//         auto points = std::span<const point2> 
//     }
// }
#endif