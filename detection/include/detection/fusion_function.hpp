#ifndef FUSION_FUNCTION_H
#define FUSION_FUNCTION_H

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "detection/radar_type.hpp"

#define Z_UPPER_BOUND 3
#define SELF_BOUND 5
#define VELOCITY_BOUND 2
                                    

struct VelocityEstimatorResidual {
    VelocityEstimatorResidual(double azimuth, double vd) : azimuth_(azimuth), vd_(vd) {}

    template <typename T>
    bool operator()(const T* const v_ego, T* residual) const {
        residual[0] = v_ego[0] * cos(T(azimuth_)) + v_ego[1] * sin(T(azimuth_)) - T(vd_);
        return true;
    }

private:
    const double azimuth_, vd_;
};

Eigen::Vector2d EstimateEgoVelocity(pcl::PointCloud<RadarPointType>::Ptr cloud, double init_x, double init_y) {
    Eigen::VectorXd x(cloud->points.size());
    Eigen::VectorXd y(cloud->points.size());
    Eigen::VectorXd vd(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        x(i) = cloud->points[i].x;
        y(i) = cloud->points[i].y;
        vd(i) = cloud->points[i].intensity;
    }

    Eigen::VectorXd azimuth = Eigen::atan2(y.array(), x.array());
    Eigen::Vector2d v_ego(init_x, init_y);

    ceres::Problem problem;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<VelocityEstimatorResidual, 1, 2>(
                new VelocityEstimatorResidual(azimuth(i), vd(i)));
        problem.AddResidualBlock(cost_function, nullptr /* squared loss */, v_ego.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // std::cout << summary.BriefReport() << "\n";
    // std::cout << "Estimated v_ego: " << v_ego.transpose() << std::endl;

    return v_ego;
}

void VAP(pcl::PointCloud<RadarPointType>::Ptr cloud, pcl::PointCloud<RadarPointType>::Ptr cloud_out, double init_x, double init_y){
  Eigen::Vector2d v_ego = EstimateEgoVelocity(cloud, init_x, init_y);

  std::vector<int> outliers_indices;
  std::vector<double> residuals;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
      double azimuth = std::atan2(cloud->points[i].y, cloud->points[i].x);
      double predicted_vd = v_ego[0] * std::cos(azimuth) + v_ego[1] * std::sin(azimuth);
      double residual = predicted_vd - cloud->points[i].intensity;
      residuals.push_back(residual);
  }

  double mean_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
  double sq_sum = std::inner_product(residuals.begin(), residuals.end(), residuals.begin(), 0.0);
  double std_dev = std::sqrt(sq_sum / residuals.size() - mean_residual * mean_residual);

  for (size_t i = 0; i < residuals.size(); ++i) {
      if (std::abs(residuals[i]) > 0.5 * std_dev) {
          outliers_indices.push_back(i);
      }
  }

  for (int idx : outliers_indices) {
      cloud_out->points.push_back(cloud->points[idx]);
  }
}

void clustering_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar, pcl::PointCloud<RadarPointType>::Ptr cloud_radar, double init_x, double init_y){
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<RadarPointType>::Ptr radar_filtered(new pcl::PointCloud<RadarPointType>);
    
    for (const auto& point : cloud_lidar) {
        if (point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -50 && point.x < 50 && point.y > -50 && point.y < 50) {
            lidar_filtered->push_back(point);
        }
    }

    for (const auto& point : cloud_radar){
        if (point.z <= Z_UPPER_BOUND && point.y*point.y+point.x*point.x > SELF_BOUND*SELF_BOUND && point.x > -50 && point.x < 50 && point.y > -50 && point.y < 50) {
            radar_filtered->push_back(point);
        }
    }

  pcl::PointCloud<RadarPointType>::Ptr radar_outlier(new pcl::PointCloud<RadarPointType>);
  VAP(radar_filtered, radar_outlier, init_x, init_y);

}

#endif