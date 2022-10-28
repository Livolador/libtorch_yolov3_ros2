# libtorch_yolov3_ros

YOLO的全称是you only look once，指只需要浏览一次就可以识别出图中的物体的类别和位置，这是Redmon等人在2016年的一篇研究论文中命名的。YOLO实现了自动驾驶汽车等前沿技术中使用的实时对象检测。本项目是通过libtorch实现yolov3，并使用ros2-foxy框架下发布话题。
![Image text](/image/yolo.png)

在本地本地文件读取rgb图和深度图（以TUM的数据集为例）
依赖：
eigen-3.3.7
flann-1.9.1
libtorch
opencv-3.4.16
pcl-1.12
建立项目文件夹
```bash
rviz2#打开rviz
```
根据参数文件parameters.txt中的话题去添加话题
![运行显示](/image/display.png)
添加组件Image和MarkerArray，将Fixed Frame修改为base_link
修改各种配置文件路径为自己系统的绝对路径

```bash
colcon build --merge-install
source setup.sh
ros2 run yolov3_ros2 detectnode --ros-args -p parameterPath:=/home/l/ros2/yolov3-ros2/yolov3-1.0.0/workspace/src/yolov3_ros2/config/parameters.txt
#记得修改成自己的parameters.txt参数文件的路径
```
## TODO
1、可以使用参数文件选择话题或者本地文件
2、发布目标点云

## 借鉴项目
[libtorch-yolov3](https://github.com/walktree/libtorch-yolov3)
