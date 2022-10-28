#include "pointCloud.h"

cPointCloud::cPointCloud(sCameraIntrinsicParameters _camera) : camera(_camera)
{
}

cv::Point3f cPointCloud::point2dTo3d(cv::Point3f &point)
{
    cv::Point3f p;
    p.z = double(point.z) / camera.factor;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy; // opencv和pcl y周
    return p;
}

//整幅图像转到点云
PointCloud::Ptr cPointCloud::image2PointCloud(cv::Mat rgb, cv::Mat depth)
{
    PointCloud::Ptr cloud(new PointCloud);

    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.factor;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

/*
 * 图像转到点云
 * @mask, 左上角x1，y1，右下角x2,y2
 * @rgb, 彩色图
 * @depth 深度图
 */
PointCloud::Ptr cPointCloud::image2PointCloud(cv::Mat rgb, cv::Mat depth, vector<int> mask)
{
    //点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud(new PointCloud);
    //遍历深度图
    //根据mask

    for (int i = mask[1]; i < mask[3]; ++i)
    {
        for (int j = mask[0]; j < mask[2]; ++j)
        {
            //取出当前点的深度
            ushort d = depth.ptr<ushort>(i)[j];

            //若该点没有值则跳过
            if (d == 0)
                continue;

            //若该点有数值则在点云中增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.factor;
            p.x = (j - camera.cx) * p.z / camera.fx;
            p.y = (i - camera.cy) * p.z / camera.fy;

            // opencv 读图片是bgr
            p.b = rgb.ptr<uchar>(i)[j * 3];
            p.g = rgb.ptr<uchar>(i)[j * 3 + 1];
            p.r = rgb.ptr<uchar>(i)[j * 3 + 2];
            //加到点云中
            cloud->points.push_back(p);
        }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

void cPointCloud::markCluster(PointCloud::Ptr cloudCluster, visualization_msgs::msg::Marker &marker)
{
    // Eigen::Vector4f centroid;
    Eigen::Vector4f pointMin;
    Eigen::Vector4f pointMax;
    // pcl::compute3DCentroid(*cloudCluster, centroid);
    pcl::getMinMax3D(*cloudCluster, pointMin, pointMax);
    /*
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
    */
    marker.pose.position.x = (float)(pointMax[0] + pointMin[0]) / 2.0;
    marker.pose.position.y = (float)(pointMax[1] + pointMin[1]) / 2.0;
    marker.pose.position.z = (float)(pointMax[2] + pointMin[2]) / 2.0;
    /*
    std::cout << marker.pose.position.x << std::endl;
    std::cout << marker.pose.position.y << std::endl;
    std::cout << marker.pose.position.z << std::endl;
    */
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (float)(pointMax[0] - pointMin[0]);
    marker.scale.y = (float)(pointMax[1] - pointMin[1]);
    marker.scale.z = (float)(pointMax[2] - pointMin[2]);
    /*
    std::cout << marker.scale.x << std::endl;
    std::cout << marker.scale.y << std::endl;
    std::cout << marker.scale.z << std::endl;
    */
    if (marker.scale.x == 0.0)
        marker.scale.x = 0.1;
    if (marker.scale.y == 0.0)
        marker.scale.y = 0.1;
    if (marker.scale.z == 0.0)
        marker.scale.z = 0.1;
}
