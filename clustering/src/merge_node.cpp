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


class LidarFusionNode : public rclcpp::Node
{
public:
    LidarFusionNode() : Node("lidar_fusion_node")
    {
        subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_front/points", 10, std::bind(&LidarFusionNode::front_pointsCallback, this, std::placeholders::_1));
        subscription_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_left/points", 10, std::bind(&LidarFusionNode::left_pointsCallback, this, std::placeholders::_1));
        subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_right/points", 10, std::bind(&LidarFusionNode::right_pointsCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_points", 10);
    }

private:
    void front_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_front) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *temp_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*temp_cloud, *transformed_cloud, T_front);
        *merged_cloud_ += *transformed_cloud;
        count_front = true;
        if(count_front && count_left && count_right){
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*merged_cloud_, output);
            output.header.frame_id = "map";
            publisher_->publish(output);
            merged_cloud_->clear();
            count_front = false;
            count_left = false;
            count_right = false;
        }
    }
    void left_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_left) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *temp_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*temp_cloud, *transformed_cloud, T_left);
        *merged_cloud_ += *transformed_cloud;
        count_left = true;
        if(count_front && count_left && count_right){
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*merged_cloud_, output);
            output.header.frame_id = "map";
            publisher_->publish(output);
            merged_cloud_->clear();
            count_front = false;
            count_left = false;
            count_right = false;
        }
    }
    void right_pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {   
        if(count_right) {return;}
        else{
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *temp_cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*temp_cloud, *transformed_cloud, T_right);
            *merged_cloud_ += *transformed_cloud;
        }
        count_right = true;
        if(count_front && count_left && count_right){
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*merged_cloud_, output);
            output.header.frame_id = "map";
            publisher_->publish(output);
            merged_cloud_->clear();
            count_front = false;
            count_left = false;
            count_right = false;
        }
    }
    void publish(){

        
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*merged_cloud_, output);
        output.header.frame_id = "map";
        publisher_->publish(output);
        merged_cloud_->clear();
        count_front = false;
        count_left = false;
        count_right = false;
    }

    //Settings from urdf
    Eigen::Vector3d rpy_front = Eigen::Vector3d(0.048645729083959824, 0.031505277472651416, -1.5728420649351478);
    Eigen::Vector3d xyz_front = Eigen::Vector3d(0.11156371743249438, -0.015148333508697098, -0.10643051836532358);
    Eigen::Vector3d rpy_left = Eigen::Vector3d(2.143873331795305, -0.003605912968916103, -1.5671401905346867);
    Eigen::Vector3d xyz_left = Eigen::Vector3d(0.05473934211418374, -0.2372464311776421, -0.6578870124268912);
    Eigen::Vector3d rpy_right = Eigen::Vector3d(-2.143873331795305, -0.003605912968916103, -1.5744524630551062);
    Eigen::Vector3d xyz_right = Eigen::Vector3d(0.16838809275080502, -0.2372464311776421, -0.6578870124268912);
    bool count_left = false;
    bool count_right = false;
    bool count_front = false;
    Eigen::Matrix4d T_front = createTransformationMatrix(rpy_front,xyz_front);
    Eigen::Matrix4d T_left = createTransformationMatrix(rpy_left,xyz_left);
    Eigen::Matrix4d T_right = createTransformationMatrix(rpy_right,xyz_right);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_left_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}