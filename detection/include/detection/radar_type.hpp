#ifndef RADAR_TYPE_H
#define RADAR_TYPE_H

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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>


struct RadarPointType
{
  PCL_ADD_POINT4D;
  union
  {
    struct
    {
      float velocity;
      float snr;
      float rcs;
      float confidence;
      float velocity_interval;
    };
    float data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (RadarPointType,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, velocity, velocity)
                                    (float, snr, snr)
                                    (float, rcs, rcs)
                                    (float, confidence, confidence)
                                    (float, velocity_interval, velocity_interval))

void radar2lidar_pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar, pcl::PointCloud<RadarPointType>::Ptr cloud_radar){
  for (int idx =0; idx < cloud_radar.points.size(); idx ++) {
    pcl::PointXYZ point;
    point.x = cloud_radar->points[idx].x;
    point.y = cloud_radar->points[idx].y;
    point.z = cloud_radar->points[idx].z;
    cloud_lidar->points.push_back(point);
}
}

#endif