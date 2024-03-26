#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sstream>
#include <iostream>
//#include "visualizer.hpp"
#include "dbscan.hpp"
using namespace std;

class LiDARSubscriber : public rclcpp::Node
{
public:
    LiDARSubscriber()
    : Node("lidar_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_right/points", 10, std::bind(&LiDARSubscriber::topic_callback, this, std::placeholders::_1));
    }
    int count = 0;
private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a point cloud");
        // visualizeWithPCLOnly(msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        stringstream ss;
        ss << setw(9)<<setfill('0')<<count;
        string str;
        ss>>str;
        string name = "../data/"+str + ".pcd";
        pcl::io::savePCDFileASCII(name,*cloud);
        count = count + 1;
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    std::cout<<"Start listening main function."<<std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}