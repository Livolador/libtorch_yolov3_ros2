#include "detectNode.h"

//订阅并发布话题，数据为marker

detectNode::detectNode(std::string name) : Node(name)
{
  this->declare_parameter<std::string>("parameterPath", parameterPath);

  this->get_parameter("parameterPath", parameterPath);
  if (parameterPath.empty())
  {
    std::cout << "请输入参数文件路径,例如ros2 run yolov3_ros2 detect_node --ros-args -r parameterPath:=your parameter file path" << std::endl;
    return;
  }
  // std::cout << parameterPath << std::endl;

  //读取参数文件
  cParameterReader pr(parameterPath);
  RCLCPP_INFO(this->get_logger(), "Start yolo detect node.");
  //订阅图片消息
  // imageSubscribe = this->create_subscription<sensor_msgs::Image::SharedPtr>( "topic", 10, std::bind(&detectNode::imageCallback, this, _1));
  // 创建发布者
  markerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(pr.markerTopic, 10);
  if (pr.publishImageBox)
    imagePublisher = this->create_publisher<sensor_msgs::msg::Image>(pr.imageBoxTopic, 10);
  //数据集循环
  //定义yolo
  torch::DeviceType device_type;

  if (torch::cuda::is_available())
    device_type = torch::kCUDA;
  else
    device_type = torch::kCPU;
  torch::Device device(device_type);

  Darknet net(pr.cfgPath.c_str(), &device, pr.num_class);

  map<string, string> *info = net.get_net_info();

  info->operator[]("height") = to_string(input_image_size);

  cout << "loading weight ..." << endl;
  net.load_weights(pr.weightsPath.c_str());
  cout << "weight loaded ..." << endl;

  net.to(device);

  torch::NoGradGuard no_grad;
  net.eval();

  cout << "start to inference ..." << endl;
  int objNum = 0;
  cv::RNG rng;
  for (int j = 0; j < pr.rgbFiles.size(); ++j)
  {
    //定义消息
    visualization_msgs::msg::MarkerArray markerArray;
    // DELETEALL 操作，防止重影
    //用来删除某个topic前一周期中的所有的Marker显示
    //若markerArray第一个元素操作的是DELETEALL，
    //那么其他元素可以设置生命周期无限长，也就是系统不会自己删除
    //当收到新的消息时就会自动删除这个topoic之前的全部显示。
    visualization_msgs::msg::Marker marker0;
    marker0.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.emplace_back(marker0);

    //读图片
    sFrame frame;
    frame.rgb = cv::imread(pr.rgbFiles[j]);
    frame.depth = cv::imread(pr.depthFiles[j], -1);
    cv::Mat img(frame.rgb);
    cv::Mat resized_image;
    cv::cvtColor(frame.rgb, resized_image, cv::COLOR_BGR2RGB);

    cv::resize(resized_image, resized_image, cv::Size(input_image_size, input_image_size));

    cv::Mat img_float;
    resized_image.convertTo(img_float, CV_32F, 1.0 / 255);

    auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
    img_tensor = img_tensor.permute({0, 3, 1, 2});
    auto start = chrono::high_resolution_clock::now(); //开始时间
    auto output = net.forward(img_tensor);             //前向传播
    // auto duration = duration_cast<milliseconds>(end - start);

    // It should be known that it takes longer time at first time
    // cout << "inference taken : " << duration.count() << " ms" << endl;
    auto result = net.write_results(output, 0.6, 0.4);

    if (result.dim() == 1)
    {
      objNum = 0;
      cout << "no object found" << endl;
      continue;
    }
    else
    {
      objNum = result.size(0);

      cout << objNum << " objects found:" << endl;

      float w_scale = float(frame.rgb.cols) / input_image_size;
      float h_scale = float(frame.rgb.rows) / input_image_size;

      result.select(1, 1).mul_(w_scale);
      result.select(1, 2).mul_(h_scale);
      result.select(1, 3).mul_(w_scale);
      result.select(1, 4).mul_(h_scale);

      auto result_data = result.accessor<float, 2>();
      builtin_interfaces::msg::Time ros_time = this->get_clock()->now();
      for (int i = 0; i < result.size(0); i++)
      {
        visualization_msgs::msg::Marker marker;
        // shape: n * 7, left x, left y, right x, right y, object confidence, class_score, class_id
        int kinds = (int)(result_data[i][7]);
        if (kinds >= pr.num_class || kinds < 0) //超出数组范围跳过
          continue;
        //若rviz收到相同命名空间和id的marker,会覆盖掉之前的marker，所以若不是同一个marker，需要保证命名空间和id不能都相同
        marker.ns = "/yolo";
        //选择系统自带的基础坐标系
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros_time;
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.text = pr.className[kinds];

        // std::cout << marker.text << std::endl;

        cPointCloud pc(pr.camera);
        vector<int> mask(4);
        mask[0] = (int)(result_data[i][1]);
        mask[1] = (int)(result_data[i][2]);
        mask[2] = (int)(result_data[i][3]);
        mask[3] = (int)(result_data[i][4]);
        PointCloud::Ptr cloudCluster = pc.image2PointCloud(frame.rgb, frame.depth, mask);
        pc.markCluster(cloudCluster, marker); //通过点云设置位置和方向
        /*
        marker.color.r = 0.0f;
        marker.color.g = 0.2f;
        marker.color.b = 0.2f;
        */
        marker.color.r = rng.uniform(0.f, 1.f);
        marker.color.g = rng.uniform(0.f, 1.f);
        marker.color.b = rng.uniform(0.f, 1.f);
        marker.color.a = rng.uniform(0.f, 1.f); //设置透明度
        markerArray.markers.push_back(marker);
        markerArray.markers[i].text = pr.className[kinds];
        //在图片上显示检测框
        showBox(img, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), pr.className[kinds]);
      }
      //显示2d检测结果
      if (pr.publishImageBox)
      {
        sensor_msgs::msg::Image::SharedPtr imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        imageMsg->header.stamp = rclcpp::Time(ros_time);
        imageMsg->header.frame_id = "base_link";
        imagePublisher->publish(*imageMsg);
      }
      markerPublisher->publish(markerArray); //发布marker
      // 存储下图像
      if (pr.saveImageBox)
        cv::imwrite(pr.savePath + "/" + std::to_string(j) + ".png", img);
    }
  }
}

void detectNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr imageMsg) const
{
  if (imageMsg->encoding == sensor_msgs::image_encodings::BGR8)
    cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
  else
  {
    cout << "Image encoding is not BGR8, and please check the image encoding method" << endl;
  }
}

void detectNode::showBox(cv::Mat &images, cv::Point origin0, cv::Point origin1, std::string name)
{
  //绘制图像
  cv::rectangle(images, origin0, origin1, cv::Scalar(0, 0, 255), 1, 1, 0);
  origin0.y += 8;
  cv::putText(images, name, origin0, font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 1, 0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*产生一个的节点*/
  auto node = std::make_shared<detectNode>("detectNode");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}