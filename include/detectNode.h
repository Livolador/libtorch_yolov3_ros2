#pragma once
#include "Darknet.h"
#include <iostream>
#include <chrono>
#include <time.h>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/msg/CameraInfo.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "parameterReader.h"
#include "pointCloud.h"

#define _CRT_SECURE_NO_DEPRECATE

using namespace std;
using namespace std::chrono;
using std::placeholders::_1;
using namespace std::chrono_literals;

//订阅并发布话题，数据为marker
class detectNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    detectNode(std::string name);
    void showBox(cv::Mat &images, cv::Point origin0, cv::Point origin1, std::string name);

private:
    //声明一个订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscribe;
    // 声明话题发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg) const;
    std::string parameterPath;
    //设置绘制文本的相关参数
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    // input image size for YOLO v3
    int input_image_size = 416;
};
