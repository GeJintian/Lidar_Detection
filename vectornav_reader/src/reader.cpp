#include "vectornav_reader/reader.hpp"

using namespace std;

class VectorNavSubscriber : public rclcpp::Node
{
public:
    VectorNavSubscriber()
    : Node("vectornav_subscriber")
    {
        subscription_ = this->create_subscription<vectornav_msgs::msg::InsGroup>(
            "/vectornav/raw/ins", 10, std::bind(&VectorNavSubscriber::topic_callback, this, std::placeholders::_1));
        outFile.open("right_track.csv", ios::out);
        outFile<<"x,y"<<endl;
        outFile<<fixed<<setprecision(20);
    }
    ofstream outFile;
private:
    void topic_callback(const vectornav_msgs::msg::InsGroup msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a GPS message.");
        outFile<<msg.poslla.x<<","<<msg.poslla.y<<endl;
    }
    rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    std::cout<<"Start listening main function."<<std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VectorNavSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}