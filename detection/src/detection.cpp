#include "detection/detection.hpp"
using namespace std;

MultiVehicleDetection::MultiVehicleDetection()
: Node("detection_node")
{
    sub_lidar_cloud_.subscribe(this, "/nonground");
    sub_radar_cloud_.subscribe(this, "/radar_fused_points");

    synchronizer_.reset(new Sync(SyncPolicy(50), sub_lidar_cloud_, sub_radar_cloud_));
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration(std::chrono::duration<double>(0.3)));
    synchronizer_->registerCallback(std::bind(&MultiVehicleDetection::detection_callback, this, std::placeholders::_1, std::placeholders::_2));

    vectornav_subscriber_ = this->create_subscription<a2rl_bs_msgs::msg::VectornavIns>("/a2rl/vn/ins", 10, std::bind(&MultiVehicleDetection::vectornav_callback, this, std::placeholders::_1));

    //KalmanFilterTracking * tracker;
}

MultiVehicleDetection::~MultiVehicleDetection(){}

void MultiVehicleDetection::detection_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &lidar_msg, const sensor_msgs::msg::PointCloud2::ConstPtr &radar_msg){
    is_frozen = true;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<RadarPointType>::Ptr radar_cloud(new pcl::PointCloud<RadarPointType>);
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
    pcl::fromROSMsg(*radar_msg, *radar_cloud);
    clustering_filter(lidar_cloud, radar_cloud, latest_vn_msg_.velocity_body_ins.x, latest_vn_msg_.velocity_body_ins.y);

    is_frozen = false;
}

void MultiVehicleDetection::vectornav_callback(const a2rl_bs_msgs::msg::VectornavIns::SharedPtr msg) {
    if (!is_frozen) latest_vn_msg_ = *msg;
}

void MultiVehicleDetection::clustering_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar, pcl::PointCloud<RadarPointType>::Ptr cloud_radar, double init_x, double init_y){
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<RadarPointType>::Ptr radar_filtered(new pcl::PointCloud<RadarPointType>);
    
    for (const auto& point : *cloud_lidar) {
        if (point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -30 && point.x < 80 && point.y > -50 && point.y < 50) {
            lidar_filtered->push_back(point);
        }
    }

    for (const auto& point : *cloud_radar){
        if (point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -50 && point.x < 50 && point.y > -50 && point.y < 50) {
            radar_filtered->push_back(point);
        }
    }

    // for radar
    pcl::PointCloud<RadarPointType>::Ptr radar_outlier(new pcl::PointCloud<RadarPointType>);
    VAP(radar_filtered, radar_outlier, init_x, init_y); // consider merging ego velocity into VAP

    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    radar2lidar_(radar_xyz, radar_outlier);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr radar_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    radar_tree->setInputCloud(radar_xyz);
    std::vector<pcl::PointIndices> radar_cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1);
    ec.setMinClusterSize(3); //TODO: change this parameter
    ec.setSearchMethod(radar_tree);
    ec.setInputCloud(radar_xyz);
    ec.extract(radar_cluster_indices);

    // for lidar
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_final;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(lidar_filtered);
    sor.setLeafSize(0.1f, 0.1f, 0.05f);
    sor.filter(*lidar_final);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr lidar_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    lidar_tree->setInputCloud(lidar_final);

    std::vector<pcl::PointIndices> lidar_cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> er;
    er.setClusterTolerance(0.3);
    er.setMinClusterSize(50); //TODO: change this parameters
    er.setSearchMethod(lidar_tree);
    er.setInputCloud(lidar_final);
    er.extract(lidar_cluster_indices);

    //TODO: maybe we also need to calculate the norm vector of each cluster
    std::vector<LidarClusterProperties> lidar_cluster_properties;
    for (std::vector<pcl::PointIndices>::const_iterator it = lidar_cluster_indices.begin(); it != lidar_cluster_indices.end(); ++it)
    {
        double x=0;
        double y=0;
        double z=0;
        double minx = 1000;
        double miny = 1000;
        double maxx = -1000;
        double maxy = -1000;
        double count = 0;
        bool valid = true;
        for (const auto& idx : it->indices){
            x += (*lidar_final)[idx].x;
            y += (*lidar_final)[idx].y;
            z += (*lidar_final)[idx].z;
            if (minx > (*lidar_final)[idx].x){
                minx = (*lidar_final)[idx].x;
            }
            if (miny > (*lidar_final)[idx].y){
                miny = (*lidar_final)[idx].y;
            }
            if (maxx < (*lidar_final)[idx].x){
                maxx = (*lidar_final)[idx].x;
            }
            if (maxy < (*lidar_final)[idx].y){
                maxy = (*lidar_final)[idx].y;
            }
            count = count + 1;
            if (maxx - minx > VEHICLE_LENGTH || maxy - miny > VEHICLE_LENGTH){
                valid = false;
                break;
            }
        }
        if(!valid) continue;
        lidar_cluster_properties.push_back(LidarClusterProperties(x/count, y/count, z/count, minx, maxx, miny, maxy));
    }

    std::vector<RadarClusterProperties> radar_cluster_properties;
    for (std::vector<pcl::PointIndices>::const_iterator it = lidar_cluster_indices.begin(); it != lidar_cluster_indices.end(); ++it)
    {
        double x=0;
        double y=0;
        double z=0;
        double minx = 1000;
        double miny = 1000;
        double maxx = -1000;
        double maxy = -1000;
        double count = 0;
        double vd = 0;
        for (const auto& idx : it->indices){
            x += (*radar_outlier)[idx].x;
            y += (*radar_outlier)[idx].y;
            z += (*radar_outlier)[idx].z;
            vd += (*radar_outlier)[idx].velocity;
            if (minx > (*radar_outlier)[idx].x){
                minx = (*radar_outlier)[idx].x;
            }
            if (miny > (*radar_outlier)[idx].y){
                miny = (*radar_outlier)[idx].y;
            }
            if (maxx < (*radar_outlier)[idx].x){
                maxx = (*radar_outlier)[idx].x;
            }
            if (maxy < (*radar_outlier)[idx].y){
                maxy = (*radar_outlier)[idx].y;
            }
            count = count + 1;
        }
        radar_cluster_properties.push_back(RadarClusterProperties(x/count, y/count, z/count, vd/count));
    }

    // KNN search
    std::vector<int> associations(radar_cluster_properties.size(), -1);
    CLUSTER_ASSOCIATION_KNN(lidar_cluster_properties, radar_cluster_properties, associations);
    if(associations.size() == 0){
        return;
    }
    

}