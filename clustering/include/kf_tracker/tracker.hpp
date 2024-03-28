#ifndef TRACKER_H
#define TRACKER_H

#include "kf_tracker/CKalmanFilter.hpp"
#include "kf_tracker/featureDetection.hpp"
#include "opencv2/video/tracking.hpp"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <limits>
#include <utility>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class KFTrackerNode : public rclcpp::Node
{
public:
    KFTrackerNode() : Node("kf_tracker")
    {
        // Publishers
        pub_cluster0 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_0", 1);
        pub_cluster1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_1", 1);
        pub_cluster2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_2", 1);
        pub_cluster3 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_3", 1);
        pub_cluster4 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_4", 1);
        pub_cluster5 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_5", 1);
        // Add the rest of your publishers following the same pattern...

        objID_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("obj_id", 1);
        markerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz", 1);

        // Subscriber
        sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "filtered_cloud", 1, std::bind(&KFTrackerNode::cloud_cb, this, std::placeholders::_1));
    }

private:
    void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster0, pub_cluster1, pub_cluster2, pub_cluster3, pub_cluster4, pub_cluster5;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr objID_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
};

#endif