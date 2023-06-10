#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <math.h>

#include "usefulMathFunc.h"

using namespace std;

// 生成无人船路径的类
// 定义类时要输入的 usvX_、 usvY_、tvX_、tvY_、tvAngle_ 都是ENU 系下的量
// tvAngle_ 是目标船方向角
// 需要生成路径时，调用成员函数 planDockPath()
class USVPathPlanner {
   
   public:
    double r = 19;  // 路径规划算法的参数：小圆弧的半径（m）
    double l = 20;  // 路径规划算法的参数：小圆弧出口点距离目标船侧向轴线的垂直距离（m）
    double d = 10;   // 路径规划算法的参数：最后直线段距离目标船纵轴的距离（m，不考虑误差时，直线段应与目标船纵轴平行）

    double nPURSUE = 0;    // PURSUE 阶段的路径点数
    double nAVOIDODS = 0;  // avoid_OBS阶段的路径点数
    double nAPPROACH = 0;  // DOCK_APPROACH 阶段的路径点数
    double nORBIT = 0;     // DOCK_ORBIT 阶段的路径点数
    double nTRANSFER = 0;  // DOCK_TRANSFER 阶段的路径点数
    double nADJUST = 0;    // DOCK_ADJUST 阶段的路径点数

    int minDist2TransferIndex = 0;  // DOCK_ORBIT 段中距离 DOCK_TRANSFER 第一个点最近的那个点的索引
    
    double ds = 5;          // 路径中每个点的间隔（大概在ds上下，非精确）

    vector<vector<double> > pathPURSUE;     // PURSUE 阶段的路径，是一个 nPURSUE*2 的数组，内含由 USV 当前位置至目标船大致位置的一条直线
    vector<vector<double> > pathAPPROACH;  // DOCK_APPROACH 阶段的路径，是一个 nAPPROACH*2 的数组，内含由 USV 当前位置至大圆的一条逆时针方向的切线，它的最后一个点是 pathORBIT 的第一个点
    vector<vector<double> > pathORBIT;     // DOCK_ORBIT 阶段的路径，是一个 nORBIT*2 的数组，内含大圆完整的的360度的圆弧点，它的最后一个点【不是】 pathTRANSFER 的第一个点
    vector<vector<double> > pathTRANSFER;  // DOCK_TRANSFER 阶段的路径，是一个 nTRANSFER*2 的数组，内含一段小圆圆弧，连接大圆变轨点至 DOCK_ADJUST 直线段的首个点，它的最后一个点是 pathADJUST 的第一个点
    vector<vector<double> > pathADJUST;    // DOCK_ADJUST 阶段的路径，是一个 nADJUST*2 的数组，内含一段平行于目标船的直线段，此阶段应以极缓慢的速度前进，此阶段的路径可以不用走完，而是要配合上机械臂的大物体探测功能随时停船
    vector<vector<double> > pathAVOIDODS;   //   pathAVOIDODS是一个nAVOIDODS*2的数组，内含避障路径。

    USVPathPlanner() {
        pathPURSUE.reserve(1000);
        pathAVOIDODS.reserve(1000);
        pathAPPROACH.reserve(1000);
        pathORBIT.reserve(1000);
        pathTRANSFER.reserve(1000);
        pathADJUST.reserve(1000);
    }

    // 规划无人船追踪段的路径
    void planPursuePath(double usvX_, double usvY_, double tvEstX_, double tvEstY_);

    // 规划无人船追踪段（避障）的路径
    void PathAvoidOBS(double usvX_, double usvY_, double usvYaw_, double OBSX_, double OBSY_, double OBSCenR_);

    // 规划无人船泊近段（4个小段）的路径
    void planDockPath(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle);

    // 更新无人船泊近段（3个小段，不包含接近段）的路径
    void updateDockPath(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle);

    // 更新无人船泊近段中最后的调整段的路径
    void updateAdjustPath(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle);

    // 规划一条直线路径
    vector<vector<double> > planLinePath(double startX, double startY, double endX, double endY, double pNum);

    // 规划一条圆路径
    vector<vector<double> > planCirclePath(double cirX, double cirY, double cirR, double startAngle, double endAngle, double pNum);

   private:
    double R = 0;       // 计算得到的大圆半径
    double theta = 0;   // 一个角度，用目标船的方向角（ENU 系）减去这个角，就是大小圆变轨点的方向角（ENU 系）；只要 r、d 和 l 给得适当，则 theta 一直是一个正的角度
    double sc1X = 0;     // 小圆弧 1 的圆心 x 坐标（取决于输入的 tvX 和 tvY 在哪个坐标系）
    double sc1Y = 0;     // 小圆弧 1 的圆心 y 坐标（取决于输入的 tvX 和 tvY 在哪个坐标系）
    double sc2X = 0;     // 小圆弧 2 的圆心 x 坐标（取决于输入的 tvX 和 tvY 在哪个坐标系）
    double sc2Y = 0;     // 小圆弧 2 的圆心 y 坐标（取决于输入的 tvX 和 tvY 在哪个坐标系）

    double usvTVAngle = 0;

    double tangentPointAngle = 0;
    double tangentPointX = 0;       // APPROACH -> ORBIT 的切点 x 坐标
    double tangentPointY = 0;       // APPROACH -> ORBIT 的切点 y 坐标

    double transferPoint1X = 0;      // ORBIT -> 小圆弧 1 TRASNFER 的切点 x 坐标
    double transferPoint1Y = 0;      // ORBIT -> 小圆弧 1 TRASNFER 的切点 y 坐标

    double transferPoint2X = 0;      // ORBIT -> 小圆弧 2 TRASNFER 的切点 x 坐标
    double transferPoint2Y = 0;      // ORBIT -> 小圆弧 2 TRASNFER 的切点 y 坐标

    int useSCNo = 0;

    bool isPathUpdateAllowed = false;

    inline void calcDockPathParameters(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle);

    inline void clearPursuePath();

    inline void clearDockPath(bool isClearAPPROACH, bool isClearORBIT, bool isClearTRANSFER, bool isClearADJUST);

};