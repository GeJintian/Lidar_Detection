#ifndef CKALMANFILTER_H
#define CKALMANFILTER_H
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace cv;
using namespace std;


class KalmanFilterTracking {
public:
    KalmanFilterTracking();
    void KFT(const std::vector<RadarClusterProperties> &property_set, vector<*CKalmanFilter> &kf_collection);
    void tracking_cb();
    std::vector<geometry_msgs::msg::Point> prevClusterCenters;
    bool firstFrame = true;
    std::vector<int> objID;
};

std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat);
#endif