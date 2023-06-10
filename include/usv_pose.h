#pragma once

#include <iostream>
#include <memory>
#include <string>
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

#include "usefulMathFunc.h"

class USVPose : public rclcpp::Node {
   public:

    // 无人船状态量
    double x = 0;       // USV 的 x 坐标（ENU 系）
    double y = 0;       // USV 的 y 坐标（ENU 系）
    double vx = 0;      // USV 的 vx (ENU 系）
    double vy = 0;      // USV 的 vy（ENU 系）
    double axb = 0;     // USV 的 ax (USV 船体系）
    double ayb = 0;     // USV 的 ay (USV 船体系）
    double roll = 0;    // USV 的滚转角
    double pitch = 0;   // USV 的俯仰角
    double yaw = 0;     // USV 的方向角、朝向角、航向角 (ENU 系，rad）
    double yawRate = 0; // USV 的方向角速度 (ENU 系，rad/s）

    // Flag to determine if the USV pose variables are available
    bool isValid = false;           // 此 USVPose 下的数据是否有效
    bool isTVEstXYValid = false;    // 此 USVPose 下是否给了目标船的预测位置

    // Target vessel 相关
    double tvEstX = 0;  // 目标船预测的 x 坐标（这个坐标是定位系统用的坐标，此 USVPose 将以其为原点建立如ENU系）
    double tvEstY = 0;  // 目标船预测的 y 坐标（这个坐标是定位系统用的坐标，此 USVPose 将以其为原点建立如ENU系）



    // Constructor function
    USVPose() : Node("usv_node_pose") {
        // Init the vars
        sub_Odom = this->create_subscription<nav_msgs::msg::Odometry>("/usv/odom", 1, std::bind(&USVPose::usvOdomCallback, this, std::placeholders::_1));
        sub_IMU = this->create_subscription<sensor_msgs::msg::Imu>("/usv/imu/data", 1, std::bind(&USVPose::usvIMUCallback, this, std::placeholders::_1));
        sub_Localization = this->create_subscription<geometry_msgs::msg::Point>("/target_vessel/odom", 1, std::bind(&USVPose::tvOdomCallback, this, std::placeholders::_1));
    }

    // updateLidarXY 输入激光雷达探测到的目标船坐标，用来更新无人船在 ENU 系下的位置
    // 输入 x_ 和 y_ 为激光雷达探测到目标船在无人船船体系下的坐标值，lidarUpdateXY 将对其进行坐标转换至 ENU 系下
    void updateLidarXY(double x_, double y_, double timeStamp);

    // updateTVEstXY 更新目标船的预估位置，这里输入的目标船位置是基于定位系的，将会用作 ENU 系的原点
    void updateTVEstXY(double tvEstX_, double tvEstY_);

   private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Odom;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_IMU;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_Localization;

    // LIDAR Measure 
    double xLidar = 0;
    double yLidar = 0;
    double vxLidar = 0;
    double vyLidar = 0;
    double lidarTimeStamp = 0;
    double xLidarLast = 0;
    double yLidarLast = 0;
    bool isLidarTimeInit = false;

    // Doppler Measure
    double xDoppler = 0;
    double yDoppler = 0;
    double vxDoppler = 0;
    double vyDoppler = 0;
    double dopplerTimeStamp = 0;
    bool isDopplerTimeInit = false;

    // Odom Measure
    double distLast = 0;
    double xLast = 0;
    double yLast = 0;

    // Time variables
    double odomTimeStamp = 0;
    bool isTimeInit = false;

    // 标志位
    bool isPoseValid = false;       // 此 USVPose 下的位置数据是否有效
    bool isIMUValid = false;        // 此 USVPose 下的IMU数据是否有效

    // USV Odom callback function
    void usvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // USV IMU callback function
    void usvIMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // Target vessel  callback function
    void tvOdomCallback(const geometry_msgs::msg::Point::SharedPtr msg);
};