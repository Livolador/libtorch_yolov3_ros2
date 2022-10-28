#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <boost/lexical_cast.hpp>

using namespace std;

//定义数据
// rgbd图像
struct sFrame
{
  cv::Mat rgb, depth;
};

// 相机内参结构
struct sCameraIntrinsicParameters
{
  double factor, cx, cy, fx, fy, d0, d1, d2, d3, d4;
};

class cParameterReader
{
public:
  cParameterReader(string fileName = "../config/parameters.txt"); //读取所有参数
  void initTum();
  template <class T>
  void getData(const string &key, T &value); //数据转换
  void getClassName();                       //读取类名
  int num_class;
  string cfgPath, weightsPath, namePath;
  string imagesPath, datasetPath;
  string rgbTopic, depthTopic, markerTopic, pointCloudTopic, imageBoxTopic;
  bool topicOrFile = false, publishPointCloud = false, publishImageBox = true;
  map<string, string> data;            //读到的参数数据
  sCameraIntrinsicParameters camera;   //相机内参
  vector<string> rgbFiles, depthFiles; //数据集存储路径
  int startIndex;                      //起始
  vector<string> className;            //类名
  bool saveImageBox = 0;
  string savePath;
};
