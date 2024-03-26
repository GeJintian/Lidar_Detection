#include "visualizer.hpp"

void visualizeWithPCLOnly(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // std::cout<<"Printing PCL"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    // }
}

//void visualizeWith