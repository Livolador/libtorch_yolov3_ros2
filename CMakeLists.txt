cmake_minimum_required( VERSION 3.22 )#声明cmake最低版本


project(yolov3 VERSION 1.0.0)#声明一个工程

SET( CMAKE_CXX_COMPILER "g++") #设定编译器

SET(CMAKE_BUILD_TYPE "Release")#编译类型
SET(CMAKE_CXX_FLAGS "-std=c++14")#c++版本
SET(CMAKE_CXX_FLAGS_RELEASE "-O3  -g -fPIC -fopenmp ")#-O3选项，该选项表示优化等级；-w 不显示waring -Wall显示所有警告
#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories( ${Eigen_INCLUDE_DIR} )

#增加boost的依赖

find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(Torch_DIR ${SRC_PATH}/share/cmake/Torch)
find_package(Torch REQUIRED)

message(STATUS "Torch_DIR = ${Torch_DIR}")
find_package(PCL REQUIRED COMPONENTS common io visualization)
include_directories( ${PCL_INCLUDE_DIRS} )

set(dependencies
    rclcpp
    std_msgs
    image_transport
    cv_bridge
    sensor_msgs
    visualization_msgs
)
#add_library(parameterpeader parameterReader.cpp)
#add_library(darknet Darknet.cpp)
#add_library(pointcloud pointCloud.cpp)

aux_source_directory(./src DIR_SRCS)
add_executable(detectnode ${DIR_SRCS})
ament_target_dependencies(detectnode  ${dependencies})
target_link_libraries(detectnode 
${Eigen_LIBS}   ${TORCH_LIBRARIES}
${PCL_LIBS}  ${OpenCV_LIBS}   ${Boost_LIBRARIES})

install(TARGETS
  detectnode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
