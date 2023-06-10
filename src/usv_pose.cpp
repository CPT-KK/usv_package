#include "usv_pose.h"

// updateLidarXY 输入激光雷达探测到的目标船坐标，用来更新无人船在 ENU 系下的位置
// 输入 x_ 和 y_ 为激光雷达探测到目标船在无人船船体系下的坐标值，lidarUpdateXY 将对其进行坐标转换至 ENU 系下
void USVPose::updateLidarXY(double x_, double y_, double timeStamp) {
    double dt = 0;
    if (!isLidarTimeInit) {
        frameTF2D(x_, y_, -yaw, xLidarLast, yLidarLast);
        xLidarLast = -xLidarLast;
        yLidarLast = -yLidarLast;
        lidarTimeStamp = timeStamp;
        isLidarTimeInit = true;
    } else {
        dt = timeStamp - lidarTimeStamp;
        frameTF2D(x_, y_, -yaw, xLidar, yLidar);
        xLidar = -xLidar;
        yLidar = -yLidar;

        if (dt != 0) {
            vxLidar = (xLidar - xLidarLast) / dt;
            vyLidar = (yLidar - yLidarLast) / dt;
        }

        xLidarLast = xLidar;
        yLidarLast = yLidar;
        lidarTimeStamp = timeStamp;
    }
    return;
}

// updateTVEstXY 更新目标船的预估位置，这里输入的目标船位置是基于定位系的，将会用作 ENU 系的原点
void USVPose::updateTVEstXY(double tvEstX_, double tvEstY_) {
    tvEstX = tvEstX_;
    tvEstY = tvEstY_;
    isTVEstXYValid = true;
}

// USV Odom callback function
void USVPose::usvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double dt = 0;
    double nowTimeStamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    x = msg->pose.pose.position.x - tvEstX;
    y = msg->pose.pose.position.y - tvEstY;

    if (!isTimeInit && isTVEstXYValid) {
        xLast = x;
        yLast = y;
        odomTimeStamp = nowTimeStamp;
        isTimeInit = true;
    } else if (nowTimeStamp - odomTimeStamp >= 0.05 && isTVEstXYValid) {
        dt = nowTimeStamp - odomTimeStamp;
        if (dt != 0) {
            vx = (x - xLast) / dt;
            vy = (y - yLast) / dt;
        }

        // Save current info
        odomTimeStamp = nowTimeStamp;
        xLast = x;
        yLast = y;

        // Change the flag as needed
        isPoseValid = true;
        isValid = isPoseValid && isIMUValid && isTVEstXYValid;
    }

    return;
}

// USV IMU callback function
void USVPose::usvIMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    // Get acceleration in body x and y, and
    axb = msg->linear_acceleration.x;
    ayb = msg->linear_acceleration.y;
    yawRate = msg->angular_velocity.z;

    // Angle: xyzw to rpy
    tf2::Quaternion imu_quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(imu_quat);
    m.getRPY(roll, pitch, yaw);

    // Now the USV IMU variables are set. Change the flag as available
    isIMUValid = true;
    isValid = isPoseValid && isIMUValid && isTVEstXYValid;

    return;
}

// Target vessel  callback function
void USVPose::tvOdomCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    updateTVEstXY(msg->x, msg->y);
    isTVEstXYValid = true;
    return;
}