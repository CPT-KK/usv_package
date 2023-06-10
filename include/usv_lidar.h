#pragma once

#define SENSOR0_DIVX 0.738
#define SENSOR0_DIVY 0.0
#define SENSOR0_DIVZ 1.68
#define SENSOR0_ROTANG 0.0

#define SENSOR2_DIVX 0.0
#define SENSOR2_DIVY 1.7
#define SENSOR2_DIVZ 1.14
#define SENSOR2_ROTANG -90.0

#define SENSOR3_DIVX 0.0
#define SENSOR3_DIVY -1.7
#define SENSOR3_DIVZ 1.14
#define SENSOR3_ROTANG 90.0

#include <iostream>
#include <memory>
#include <vector>

// ROS headers
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"

// PointCloud Library headers
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "usv_pose.h"

#include "usefulMathFunc.h"

using namespace std;

class LIDARPointCloud : public rclcpp::Node {
   public:
    double timeStampSecs;           // 云图的时间戳（秒）

    double USV2TargetVesselHeading; // 无人船船头指向目标船的航向角（rad，ENU系下）

    double tvPredictR = 0;  // 目标船预测范围，以(0, 0)为圆心，因为ENU的原点在目标船

    bool isLidarFindTV = false;                  // 是否找到目标船的标志位
    double tvAngle = 0;                      // 经激光雷达测量后，目标船的方向角（rad， ENU系下）
    double tvRectL = 0;                      // 经激光雷达测量后，目标船的最小外接方向矩形的长（m）
    double tvRectW = 0;                      // 经激光雷达测量后，目标船的最小外接方向矩形的宽（m）
    double tvRectLWRatio = 0;                // 经激光雷达测量后，目标船的最小外接方向矩形的长宽比
    double tvX = 0;                          // 经激光雷达测量后，目标船的最小外接方向矩形的 x 坐标（无人船船体系下）
    double tvY = 0;                          // 经激光雷达测量后，目标船的最小外接方向矩形的 y 坐标（无人船船体系下）

    vector<vector<double> > obsXYR;  // 障碍物的中心 x y 坐标信息与半径 R（m，无人船船体系下），第一列是 x，第二列是 y

    double tvRecordBestAngle = 0;       // 经激光雷达测量后，目标船在LW率最大情况下的方向角，ENU系下
    double tvRecordBestLen = 0;
    double tvRecordBestWid = 0;
    double tvRecordBestLWRatio = 0;

    // Constructor function
    LIDARPointCloud(std::shared_ptr<USVPose> usvPosePtr_, double tvPredictR_) : Node("usv_lidar_pose") {
        frontLidarSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/usv/slot0/points", 1, std::bind(&LIDARPointCloud::lidarCallback, this, std::placeholders::_1));
        leftLidarSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/usv/slot2/points", 1, std::bind(&LIDARPointCloud::lidarCallback, this, std::placeholders::_1));
        rightLidarSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/usv/slot3/points", 1, std::bind(&LIDARPointCloud::lidarCallback, this, std::placeholders::_1));

        // 初始化点云指针
        cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud0Ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud2Ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud3Ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // 写入目标船预估范围
        tvPredictR = tvPredictR_;

        // 将 USV 的位置类的地址写入到这个类的成员变量中
        usvPosePtr = usvPosePtr_;

        // 预分配内存
        tvRecord.reserve(4000);
        obsXYR.reserve(20);
    }

    ~LIDARPointCloud() {

    }

    // 打开记录功能，将会在调用 endTVRecord 前记录所有的关于目标船的信息
    void startTVRecord();

    // 停止记录功能
    void endTVRecord();

    // 计算在记录期间最好的目标船的信息量
    void calcBestRecord();

   private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr frontLidarSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr leftLidarSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rightLidarSub;

    // 定义 PCL 下的点云 pclCloud 变量
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0Ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2Ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3Ptr;

    bool isCloud0Ready = false;
    bool isCloud2Ready = false;
    bool isCloud3Ready = false;

    double timeStampSecsLidar0;
    double timeStampSecsLidar2;
    double timeStampSecsLidar3;

    bool isTVRecord = false;
    vector<vector<double> > tvRecord;

    // Ptr of USV state information
    std::shared_ptr<USVPose> usvPosePtr;

    // 成员函数
    void cloudCut(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double xLB, double xUB, double yLB, double yUB, double zLB, double zUB);
    void cloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double divFilter);
    void cloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double lidarDivX, double lidarDivY, double lidarDivZ, double lidarRotateAngle);
    void clusterDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double objVector[3], double& len, double& wid, double& lwRatio, double& cenX, double& cenY);
    void objExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
     
    // LIDAR callback function
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

