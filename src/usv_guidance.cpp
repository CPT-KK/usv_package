#include "usv_guidance.h"

void USVGuidance::setPath(vector<vector<double> > path, int inputEndIndex) {
    xd.clear();
    yd.clear();

    // 置入输入的路径
    for (int i = 0; i < static_cast<int>(path.size()); i++) {
        xd.push_back(path[i][0]);
        yd.push_back(path[i][1]);
    }

    // 设置当前跟踪点为此路径的初始点
    currentIndex = 0;

    // 设置输入路径终点的索引
    if (inputEndIndex < 0) {
        endIndex = path.size() - 1;
    } else {
        endIndex = inputEndIndex;
    }
    
    // 保存输入路径的n点数
    pathSize = path.size();
}


// 发送控制指令给控制器
// isSpeedGiven == 1时，实现匀速控制
void USVGuidance::guidance(double uGiven, double dist2NextPoint) {
    if (currentIndex >= endIndex) {
        return;
    }

    // 求船体坐标系下的速度 u, v
    frameTF2D(usvPosePtr->vx, usvPosePtr->vy, usvPosePtr->yaw, u, v);

    // 更新跟踪点
    while (calcNorm2(xd[currentIndex] - usvPosePtr->x, yd[currentIndex] - usvPosePtr->y) <= dist2NextPoint && currentIndex < endIndex) {
        currentIndex = currentIndex + 1;
    }


    // 求跟踪点的切线角
    if (currentIndex == endIndex) {
        theta = theta;
    } else {
        theta = atan2((yd[currentIndex + 1] - yd[currentIndex]), (xd[currentIndex + 1] - xd[currentIndex]));
    }

    // 求无人船的飘角
    if (u == 0 && fabs(v) <= 1e-3) {
        beta = 0;
    } else if (u == 0) {
        beta = PI / 2.0 * sign(v);
    } else {
        beta = atan2(v, u);
    }

    // 计算无人船相对于当前跟踪点切线的垂直方向的误差
    yErr = -(usvPosePtr->x - xd[currentIndex]) * sin(theta) + (usvPosePtr->y - yd[currentIndex]) * cos(theta);

    // 计算期望的朝向角
    psiSP = theta - beta + atan2(-yErr, delta);

    // 计算朝向角误差
    psiErr = (psiSP - usvPosePtr->yaw);

    // 朝向角误差（ENU 下）限幅
    psiErr = wrapPI(psiErr);

    // 期望前进速度给定，使用输入的 u 作为期望
    uSP = uGiven;

    uErr = uSP - u;  // 速度误差（船体系下）
    vErr = 0 - v;   // 侧向速度误差

    printf("=============================================================================\n");
    printf("Guidance 输出: 更新半径 = %05.2f\n", dist2NextPoint);
    printf("此段路径当前跟踪点: No.%d, [%08.2f, %08.2f]. 此段路径终点: No.%d, [%08.2f, %08.2f].\n", currentIndex, xd[currentIndex], yd[currentIndex], endIndex, xd[endIndex], yd[endIndex]);
    printf("USV 船体系速度 u: %05.2f, v: %05.2f，合速度: %05.2f.\n", u, v, calcNorm2(u,v));
    printf("theta: %05.2f, beta: %05.2f, yErr: %05.2f\n", theta, beta, yErr);
    
    usvControllerPIDPtr->moveUSV(uErr, psiErr);

    return;
}