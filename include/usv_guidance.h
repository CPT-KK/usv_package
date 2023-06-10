#pragma once

#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

#include "usefulMathFunc.h"
#include "usv_controller.h"
#include "usv_path_planner.h"
#include "usv_pose.h"

using namespace std;
class USVGuidance : USVPathPlanner {
   public:
    // path planning LOS guidance
    double psiSP = 0;           // 期望航向角
    double uSP = 0;             // 期望船体系纵向速度
    double vSP = 0;             // 期望船体系横向速度
    double delta = 6 * 6;       // 前视距离
    double usvW = 0;            // 船宽
    double R = 8;               // 更换跟踪点的判据，小于等于 R 时则切换跟踪点至下一个距离大于 R 的点
    double theta = 0;           // 路径上跟踪点在ENU坐标系下的切线角
    double beta = 0;            // 漂角
    double xErr = 0;            // 在路径坐标系下x方向的误差
    double yErr = 0;            // 在路径坐标系下 y 方向上的误差
    double uErr = 0;            // 船体系纵向速度误差
    double psiErr = 0;          // 航向角误差
    double vErr = 0;            // 船体系侧向速度误差

    double u = 0;  // 船体系纵向速度
    double v = 0;  // 船体系侧向速度

    double uMax = 3.0;

    shared_ptr<USVPose> usvPosePtr;
    shared_ptr<USVControllerPID> usvControllerPIDPtr;

    int pathSize = 0;
    vector<double> xd;        // 当前路径的 x （ENU）
    vector<double> yd;        // 当前路径的 y （ENU）
    int currentIndex = -1;    // 当前正在跟踪路径点的索引
    int endIndex = -1;        // 当前的路径点应该跟踪到这个索引的点的时候，应结束这条路径的跟踪

    USVGuidance(shared_ptr<USVPose> usvPosePtr_, shared_ptr<USVControllerPID> usvControllerPIDPtr_) {
        usvPosePtr = usvPosePtr_;
        usvControllerPIDPtr = usvControllerPIDPtr_;

        // 初始化，预分配内存
        xd.reserve(2000);
        yd.reserve(2000);
    }

    // 置入要跟踪的轨迹，默认从第一个点开始跟踪，跟踪到最后一个点，如果有需要可以修改跟踪的最后一个点
    void setPath(vector<vector<double>> path, int inputEndIndex = -10086);

    // 输入给定船体系纵向速度 uGiven 和切换到下一个跟踪点的判据 dist2NextPoint，guidance() 将发送控制指令给控制器
    void guidance(double uGiven, double dist2NextPoint);

};