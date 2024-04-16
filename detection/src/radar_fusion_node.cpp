#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/filter.h> 
#include "pcl/filters/impl/filter.hpp"
#include "detection/radar_type.hpp"

#define LIDAR_FILTER_ANGLE 52


Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R = q.matrix();
    return R;
}

Eigen::Matrix4d createTransformationMatrix(Eigen::Vector3d &rpy, Eigen::Vector3d &xyz) {
    Eigen::Matrix3d rotationMatrix = rpyToRotationMatrix(rpy(0), rpy(1), rpy(2));
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix.block<3, 1>(0, 3) = xyz;
    return transformationMatrix;
}

void confidence_filter(pcl::PointCloud<RadarPointType>::Ptr cloud_in, pcl::PointCloud<RadarPointType>::Ptr cloud_out){
    for (const auto& point : *cloud_in) {
        if (point.confidence >= 0.5) {
            cloud_out->push_back(point);
        }
    }

}

class RadarFusionNode : public rclcpp::Node
{
public:
    RadarFusionNode() : Node("radar_fusion_node")
    {
        subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_front/points", 10, std::bind(&RadarFusionNode::front_pointsCallback, this, std::placeholders::_1));
        subscription_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_left/points", 10, std::bind(&RadarFusionNode::left_pointsCallback, this, std::placeholders::_1));
        subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_right/points", 10, std::bind(&RadarFusionNode::right_pointsCallback, this, std::placeholders::_1));
        subscription_back_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/radar_back/points", 10, std::bind(&RadarFusionNode::back_pointsCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/radar_fusion_points", 10);

    }

private:
    void front_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_front) {
            front_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        confidence_filter(cloud,front_cloud);

        count_front = true;
        if(count_front && count_left && count_right && count_back){
            publish();
        }
    }
    void left_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_left) {
            left_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        confidence_filter(cloud, left_cloud);

        count_left = true;
        if(count_front && count_left && count_right && count_back){
            publish();
        }
    }
    void right_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_right) {
            right_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        confidence_filter(cloud, right_cloud);

        count_right = true;
        if(count_front && count_left && count_right && count_back){
            publish();
        }
    }
    void back_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_back) {
            back_cloud->clear();
        }
        pcl::PointCloud<RadarPointType>::Ptr cloud(new pcl::PointCloud<RadarPointType>);
        pcl::fromROSMsg(*msg, *cloud);
        confidence_filter(cloud, back_cloud);

        count_back = true;
        if(count_front && count_left && count_right && count_back){
            publish();
        }
    }
    void publish(){
        pcl::PointCloud<RadarPointType>::Ptr merged_cloud_(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr f_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr l_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr r_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        pcl::PointCloud<RadarPointType>::Ptr b_transformed_cloud(new pcl::PointCloud<RadarPointType>());
        if(right_cloud->size()>0){
            pcl::transformPointCloud(*right_cloud, *r_transformed_cloud, T_right);
            *merged_cloud_ += *r_transformed_cloud;
        }
        if(left_cloud->size()>0){
            pcl::transformPointCloud(*left_cloud, *l_transformed_cloud, T_left);
            *merged_cloud_ += *l_transformed_cloud;
        }
        if(front_cloud->size()>0){
            pcl::transformPointCloud(*front_cloud, *f_transformed_cloud, T_front);
            *merged_cloud_ += *f_transformed_cloud;
        }
        if(back_cloud->size()>0){
            pcl::transformPointCloud(*back_cloud, *b_transformed_cloud, T_back);
            *merged_cloud_ += *b_transformed_cloud;
        }


        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*merged_cloud_, output);
        output.header.frame_id = "map";
        publisher_->publish(output);
        front_cloud->clear();
        left_cloud->clear();
        right_cloud->clear();
        back_cloud->clear();
        count_front = false;
        count_left = false;
        count_right = false;
        count_back = false;
    }

    //Settings from urdf
    Eigen::Vector3d rpy_front = Eigen::Vector3d(1.525, -0.000, 1.565);
    Eigen::Vector3d xyz_front = Eigen::Vector3d(0.077, 0.014, -0.240);
    Eigen::Vector3d rpy_left = Eigen::Vector3d(0.463, -1.552, 2.679);
    Eigen::Vector3d xyz_left = Eigen::Vector3d(-0.015, -0.066, -0.268);
    Eigen::Vector3d rpy_right = Eigen::Vector3d(-0.009, 1.550, -0.015);
    Eigen::Vector3d xyz_right = Eigen::Vector3d(0.003, 0.180, -0.290);
    Eigen::Vector3d rpy_back = Eigen::Vector3d(-1.562, 0.004, -1.566);
    Eigen::Vector3d xyz_back = Eigen::Vector3d(-0.315, 0.063, -3.069);
    bool count_left = false;
    bool count_right = false;
    bool count_front = false;
    bool count_back = false;
    Eigen::Matrix4d T_front = createTransformationMatrix(rpy_front,xyz_front);
    Eigen::Matrix4d T_left = createTransformationMatrix(rpy_left,xyz_left);
    Eigen::Matrix4d T_right = createTransformationMatrix(rpy_right,xyz_right);
    Eigen::Matrix4d T_back = createTransformationMatrix(rpy_back,xyz_back);

    pcl::PointCloud<RadarPointType>::Ptr front_cloud{new pcl::PointCloud<RadarPointType>()};
    pcl::PointCloud<RadarPointType>::Ptr left_cloud{new pcl::PointCloud<RadarPointType>()};
    pcl::PointCloud<RadarPointType>::Ptr right_cloud{new pcl::PointCloud<RadarPointType>()};
    pcl::PointCloud<RadarPointType>::Ptr back_cloud{new pcl::PointCloud<RadarPointType>()};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_left_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_back_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
