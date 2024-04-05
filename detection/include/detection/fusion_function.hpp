#ifndef LIDAR_FUNCTION_H
#define LIDAR_FUNCTION_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>
#include <queue>
#include <mutex>
#include <patchworkpp/utils.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <cmath>
#include <algorithm>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#define Z_LOWER_BOUND -1
#define Z_UPPER_BOUND 4
#define SELF_BOUND 5


void clustering_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_radar){
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    
    for (const auto& point : cloud_lidar) {
        if (point.z >= Z_LOWER_BOUND && point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.y > 0 && point.y < 100 && point.x > -50 && point.x < 50) {
            lidar_filtered->push_back(point);
        }
    }

    for (const auto& point : )

}

#endif