#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

void visualizeWithPCLOnly(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

#endif