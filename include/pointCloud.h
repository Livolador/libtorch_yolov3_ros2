#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "parameterReader.h"

using namespace std;
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class cPointCloud
{
public:
    cPointCloud(sCameraIntrinsicParameters _camera);
    cv::Point3f point2dTo3d(cv::Point3f &point);                                    // 2d点转3d点
    PointCloud::Ptr image2PointCloud(cv::Mat rgb, cv::Mat depth, vector<int> mask); //图像转到点云
    PointCloud::Ptr image2PointCloud(cv::Mat rgb, cv::Mat depth);
    void markCluster(PointCloud::Ptr cloudCluster, visualization_msgs::msg::Marker &marker);

private:
    sCameraIntrinsicParameters camera; //相机内参
};
