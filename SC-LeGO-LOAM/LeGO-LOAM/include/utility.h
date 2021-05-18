//
// Created by zlc on 2021/5/13.
//

#ifndef _LEGO_LOAM_UTILITY_H_
#define _LEGO_LOAM_UTILITY_H_

#pragma once

#include <ros/ros.h>

// 传感器数据
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// 自己写的cloud_msgs消息类型
#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

// PCL相关库
#include <pcl/point_cloud.h>            // 点云
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>

#include <pcl/filters/filter.h>         // PCL滤波
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree_flann.h>    // kdtree组织点云
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>       // ICP

// 坐标变换
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>       // STL向量
#include <array>
#include <cmath>        // 数学
#include <algorithm>    // 算法
#include <queue>        // 队列
#include <deque>        // 双向队列
#include <iostream>
#include <fstream>      // 文件操作
#include <ctime>
#include <cfloat>
#include <iterator>     // STL迭代器
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>

#include <thread>       // 线程操作
#include <mutex>

#define PI 3.14159265

using namespace std;
// rosbag filter "HK-Data20190316-2 20190331_NJ_LL.bag" "lidaronly_HK-Data20190316-2 20190331_NJ_LL.bag" "topic == '/velodyne_points'"
// rosbag filter "HK-Data20190117.bag" "lidaronly_HK-Data20190117.bag" "topic == '/velodyne_points'"


typedef pcl::PointXYZI  PointType;


// extern const string pointCloudTopic = "/velodyne_points";
// extern const string pointCloudTopic = "/kitti_scan";
extern const string pointCloudTopic = "/os1_points";            // 话题
extern const string imuTopic = "/imu/data";

// disabled, it would be better to save the pcd in $(find lego_loam)/map folder
// extern const string fileDirectory = "/tmp/";

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = false;     // if true, ang_res_y and ang_bottom are not used


// 下面是不同类型雷达的不同配置
// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;      // 360°/1800 = 0.2
// extern const float ang_res_y = 2.0;      // 32°/16 = 2.0
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;      // 表示地面需要的线圈数;

// VLP-32C
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// // HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
extern const int N_SCAN = 64;                               // 激光雷达扫描线束
extern const int Horizon_SCAN = 1024;                       // 水平点个数
extern const float ang_res_x = 360.0/float(Horizon_SCAN);   // 水平角（视界角）分辨率  水平上每一度的点数
extern const float ang_res_y = 33.2/float(N_SCAN-1);        // 垂线角分辨率，显然是等距的。
extern const float ang_bottom = 16.6+0.1;                   // 最底下角的角度
extern const int groundScanInd = 15;                        // 表示地面需要的线圈数;

extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay  = 0;
extern const int imuQueLength = 200;

extern const float sensorMinimumRange = 3.0;            // x²+y²+z² = range,最小的点与雷达的深度
extern const float sensorMountAngle = 0.0;              // 安装角度 0°
extern const float segmentTheta     = 60.0/180.0*M_PI;  // decrease this value may improve accuracy. 1.0472 点云分割时的角度跨度上限（π/3）

extern const int segmentValidPointNum = 5;              // 检查上下左右连续5个点做为分割的特征依据,如果一个点没有5个与他相同标签邻域点，那么分割无效
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;


// Mapping Params  建图参数
extern const float surroundingKeyframeSearchRadius = 50.0;  // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum = 50;       // submap size (when loop closure enabled)

// history key frames (history submap for loop closure)     历史关键帧
extern const float historyKeyframeSearchRadius = 20.0;      // NOT used in Scan Context-based loop detector / default 7.0; key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum = 25;           // 2n+1 number of history key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 1.5;       // default 0.3; the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 1500.0;     // key frames with in n meters will be visualized   1500米内的关键帧会可视化


struct smoothness_t
{
    float value;
    size_t ind;
};

struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};


// △△△ 定义两种点的类型：PointXYZIR  和  PointXYZIRPYT  △△△
/*
 * A point cloud type that has "ring" channel
 */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,
                               (float, x, x) (float, y, y)
                                        (float, z, z) (float, intensity, intensity)
                                        (uint16_t, ring, ring)
)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;             // 欧拉角
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                           (float, z, z) (float, intensity, intensity)
                                           (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                           (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif // _LEGO_LOAM_UTILITY_H_
