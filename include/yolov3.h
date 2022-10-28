#pragma once
#include <torch/torch.h>
#include <iostream>
#include <chrono>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include "Darknet.h"

#define _CRT_SECURE_NO_DEPRECATE

using namespace std; 
using namespace std::chrono; 

class yolov3
{
private:
    /**************************************************************************
     * @brief 类的名字,后期放到参数文件当中
     **************************************************************************/ 
	vector<string> classesName = {"person", "bicycle", "car", "motorcycle",	"airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",	"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
    /**************************************************************************
     * @brief 设备类型
     **************************************************************************/ 
    torch::DeviceType deviceType;
    /**************************************************************************
     * @brief 权重路径
     **************************************************************************/ 
    const char *modelPath;
    /**************************************************************************
     * @brief 使用设备
     **************************************************************************/ 
    torch::Device device(device_type);//device_type在构造函数中指定
    /**************************************************************************
     * @brief 构造yolo网络
     **************************************************************************/ 
    Darknet net(modelPath, &device);//modelPath在构造函数中指定
    /**************************************************************************
     * @brief input image size for YOLO v3,后期放到参数文件当中
     **************************************************************************/ 
    int input_image_size = 416;
    
    map<string, string> *info = net.get_net_info();

public:
    /**************************************************************************
     * @brief 构造函数
     * @param weightPath  权重路径
     **************************************************************************/ 
    yolov3(const char *weightPath, const char *_modelPath = "../models/yolov3.cfg");

    /**************************************************************************
     * @brief 检测图片
     * @param image 待检测的原始图片
     * @return 大小为(n, 6)的向量，代表检测出来的n个目标的位置、置信度、种类信息(ymin, xmin, ymax, xmax, conf, class)
     **************************************************************************/    
    vector<vector<float>> detectImage(cv::Mat image);

    /**************************************************************************
     * @brief 显示检测结果
     * @param output detectImage的输出
     * @param img 要画框的原始图像
     * @param show - 是否显示最终画了框框的图片
     **************************************************************************/   
    void show_results(vector<vector<float>> output, cv::Mat img, bool show = true);

    /**************************************************************************
     * @brief 确定使用的设备类型
     * @return 设备类型
     **************************************************************************/   
    torch::DeviceType wether_GPU(void);

    /**************************************************************************
     * @brief 在目录中读取所有图片
     * @param pattern 目录路径
     * @return 图片的向量
     **************************************************************************/
    vector<cv::Mat> readImage(cv::String pattern);

};


