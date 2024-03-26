#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualizer.hpp"

class LiDARSubscriber : public rclcpp::Node
{
public:
    LiDARSubscriber()
    : Node("lidar_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_points", 10, std::bind(&LiDARSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received a point cloud");
        visualizeWithPCLOnly(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LiDARSubscriber>());
    rclcpp::shutdown();
    return 0;
}