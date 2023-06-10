#include "usv_path_planner.h"


void USVPathPlanner::planPursuePath(double usvX_, double usvY_, double tvEstX_, double tvEstY_) {
    clearPursuePath();
    nPURSUE = ceil(calcNorm2(usvX_ - tvEstX_, usvY_ - tvEstY_) / ds);
    pathPURSUE = planLinePath(usvX_, usvY_, tvEstX_, tvEstY_, nPURSUE);

    return;
}

void USVPathPlanner::PathAvoidOBS(double usvX_, double usvY_, double usvYaw_, double OBSX_, double OBSY_, double OBSCenR_) {
    double pathX = 0;
    double pathY = 0;
    double OBSCenR = OBSCenR_;
    double usvX = usvX_;
    double usvY = usvY_;
    double OBSX = OBSX_;
    double OBSY = OBSY_;
    double usvYaw = usvYaw_;
    double usvX1 = 0, usvY1 = 0, xUSVStar = 0, yUSVStar = 0, xUSVStar1 = 0, yUSVStar1 = 0, xUSVStar2 = 0, yUSVStar2 = 0;
    double disCen = fabs(sqrt(pow(usvX - OBSX, 2.0) + pow(usvY - OBSY, 2.0)));
    // 阶段一 绕过障碍
    frameTF2D(usvX, usvY, usvYaw, usvX1, usvY1);
    xUSVStar = OBSX + OBSCenR * cos(sign(usvY1) * (PI - fabs(atan(usvY1 / usvX1)) - acos(OBSCenR / disCen)));
    yUSVStar = OBSY + OBSCenR * sin(sign(usvY1) * (PI - fabs(atan(usvY1 / usvX1)) - acos(OBSCenR / disCen)));
    frameTF2D(xUSVStar, yUSVStar, -usvYaw, xUSVStar1, yUSVStar1);
    xUSVStar2 = -xUSVStar;
    yUSVStar2 = yUSVStar;
    frameTF2D(xUSVStar2, yUSVStar2, -usvYaw, xUSVStar2, yUSVStar2);
    for (double i = 0; i < (nAVOIDODS - 5) / 2; i++) {
        double balanceFactor = ((nAVOIDODS - 5) / 2 - 1.0 - i) / ((nAVOIDODS - 5) / 2 - 1.0);
        pathX = balanceFactor * usvX + (1.0 - balanceFactor) * xUSVStar1;
        pathY = balanceFactor * usvY + (1.0 - balanceFactor) * yUSVStar1;
        pathAVOIDODS.push_back({pathX, pathY});
    }
    for (double i = 0; i < 5; i++) {
        double balanceFactor = (5- 1.0 - i) / ((5 - 1.0));
        pathX = balanceFactor * xUSVStar1 + (1.0 - balanceFactor) * xUSVStar2;
        pathY = balanceFactor * yUSVStar1 + (1.0 - balanceFactor) * yUSVStar2;
        pathAVOIDODS.push_back({pathX, pathY});
    }
    // 阶段二 前进
    xUSVStar = -usvX1;
    yUSVStar = usvY1;

    frameTF2D(xUSVStar2, yUSVStar2, -usvYaw, xUSVStar2, yUSVStar2);
    frameTF2D(xUSVStar, yUSVStar, -usvYaw, xUSVStar, yUSVStar);

    for (double i = 0; i < (nAVOIDODS - 5) / 2; i++) {
        double balanceFactor = ((nAVOIDODS - 5) / 2 - 1.0 - i) / ((nAVOIDODS - 5) / 2 - 1.0);
        pathX = balanceFactor * xUSVStar2 + (1.0 - balanceFactor) * xUSVStar;
        pathY = balanceFactor * yUSVStar2 + (1.0 - balanceFactor) * yUSVStar;
        pathAVOIDODS.push_back({pathX, pathY});
    }
    return;
}

// 根据输入的数据 usvX_、 usvY_、tvX_、tvY_、tvRectL_、tvRectW_、tvAngle_ 生成路径，存放至类的成员变量中
// 输入的 usvX_、 usvY_、tvX_、tvY_、tvAngle_ 都是ENU 系下的量
// tvAngle_ 是目标船方向角
void USVPathPlanner::planDockPath(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle) {
    clearDockPath(true, true, true, true);
    calcDockPathParameters(usvX, usvY, tvX, tvY, tvRectL, tvRectW, tvAngle);

    // pathAPPROACH 压入 Phase APPROACH: USV 当前位置 -> 大圆切点段的路径点序列（不含大圆切点） 
    pathAPPROACH = planLinePath(usvX, usvY, tangentPointX, tangentPointY, nAPPROACH);

    // pathORBIT 压入 Phase ORBIT: 大圆切点 -> 大圆-小圆变轨点段的路径点序列（不含大圆-小圆变轨点）
    pathORBIT = planCirclePath(tvX, tvY, R, tangentPointAngle, tangentPointAngle + 2 * PI, nORBIT);

    // pathTRANSFER 压入 Phase TRANSFER: 大圆-小圆变轨点 -> 直线段的圆弧路径点序列（不含直线段开始点）    
    // 先判断哪个大圆-小圆变轨点是最近的
    // 计算 ORBIT 每个点离两个大圆-小圆变轨点的距离
    vector<double> dis1Transfer1st;
    vector<double> dis2Transfer1st;
    dis1Transfer1st.reserve(nORBIT + 10);
    dis2Transfer1st.reserve(nORBIT + 10);
    for (int i = 0; i < nORBIT - 1; i++) {
        dis1Transfer1st.push_back(calcNorm2(pathORBIT[i][0] - transferPoint1X, pathORBIT[i][1] - transferPoint1Y));
        dis2Transfer1st.push_back(calcNorm2(pathORBIT[i][0] - transferPoint2X, pathORBIT[i][1] - transferPoint2Y));
    }

    // 在两组距离中，找到距离最小的那个点，提取它的索引
    int min1DistIndex = distance(dis1Transfer1st.begin(), min_element(dis1Transfer1st.begin(), dis1Transfer1st.end()));
    int min2DistIndex = distance(dis2Transfer1st.begin(), min_element(dis2Transfer1st.begin(), dis2Transfer1st.end()));

    // 判断索引的大小，确定 TRANSFER 是从哪个大圆-小圆变轨点开始规划
    if (min1DistIndex <= min2DistIndex) {
        useSCNo = 1;
        minDist2TransferIndex = min1DistIndex;
        pathTRANSFER = planCirclePath(sc1X, sc1Y, r, tvAngle - theta, tvAngle + PI / 2.0, nTRANSFER);
    } else {
        useSCNo = 2;
        minDist2TransferIndex = min2DistIndex;
        pathTRANSFER = planCirclePath(sc2X, sc2Y, r, tvAngle - theta + PI, tvAngle + PI / 2.0 + PI, nTRANSFER);
    }


    // pathADJUST 压入 Phase ADJUST: 直线段开始 -> 直线段结束的几个点
    double startX = 0;
    double startY = 0;
    double endX = 0;
    double endY = 0;
    if (min1DistIndex <= min2DistIndex) {
        startX = tvX + calcNorm2(l, d) * cos(tvAngle + atan(d / l));
        startY = tvY + calcNorm2(l, d) * sin(tvAngle + atan(d / l));
        endX = tvX + calcNorm2(l, d) * cos(tvAngle + PI - atan(d / l));
        endY = tvY + calcNorm2(l, d) * sin(tvAngle + PI - atan(d / l));
    } else {
        startX = tvX + calcNorm2(l, d) * cos(tvAngle + atan(d / l) + PI);
        startY = tvY + calcNorm2(l, d) * sin(tvAngle + atan(d / l) + PI);
        endX = tvX + calcNorm2(l, d) * cos(tvAngle + PI - atan(d / l) + PI);
        endY = tvY + calcNorm2(l, d) * sin(tvAngle + PI - atan(d / l) + PI);
    }
    pathADJUST = planLinePath(startX, startY, endX, endY, nADJUST);

    isPathUpdateAllowed = true;

    return;
}

// 根据输入的数据 usvX_、 usvY_、tvX_、tvY_、tvRectL_、tvRectW_、tvAngle_ 更新 ORBIT、TRANSFER 和 ADJUST 段的路径
void USVPathPlanner::updateDockPath(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle) {
    if (!isPathUpdateAllowed) {
        printf("未执行首次规划路径，不允许更新路径\n");
        return;
    }

    clearDockPath(false, true, true, true);
    calcDockPathParameters(usvX, usvY, tvX, tvY, tvRectL, tvRectW, tvAngle);

    // pathORBIT 压入 Phase ORBIT: 大圆切点 -> 大圆-小圆变轨点段的路径点序列（不含大圆-小圆变轨点）
    pathORBIT = planCirclePath(tvX, tvY, R, usvTVAngle, usvTVAngle + 2 * PI, nORBIT);

    // pathTRANSFER 压入 Phase TRANSFER: 大圆-小圆变轨点 -> 直线段的圆弧路径点序列（不含直线段开始点）    
    // 先判断哪个大圆-小圆变轨点是最近的
    // 计算 ORBIT 每个点离两个大圆-小圆变轨点的距离
    vector<double> dis1Transfer1st;
    vector<double> dis2Transfer1st;
    dis1Transfer1st.reserve(nORBIT + 10);
    dis2Transfer1st.reserve(nORBIT + 10);
    for (int i = 0; i < nORBIT - 1; i++) {
        dis1Transfer1st.push_back(calcNorm2(pathORBIT[i][0] - transferPoint1X, pathORBIT[i][1] - transferPoint1Y));
        dis2Transfer1st.push_back(calcNorm2(pathORBIT[i][0] - transferPoint2X, pathORBIT[i][1] - transferPoint2Y));
    }

    // 在两组距离中，找到距离最小的那个点，提取它的索引
    int min1DistIndex = distance(dis1Transfer1st.begin(), min_element(dis1Transfer1st.begin(), dis1Transfer1st.end()));
    int min2DistIndex = distance(dis2Transfer1st.begin(), min_element(dis2Transfer1st.begin(), dis2Transfer1st.end()));

    // 判断索引的大小，确定 ORBIT 段是第几个点后切换至 TRANSFER 段
    if (min1DistIndex <= min2DistIndex) {
        useSCNo = 1;
        minDist2TransferIndex = min1DistIndex;
        pathTRANSFER = planCirclePath(sc1X, sc1Y, r, tvAngle - theta, tvAngle + PI / 2.0, nTRANSFER);
    } else {
        useSCNo = 2;
        minDist2TransferIndex = min2DistIndex;
        pathTRANSFER = planCirclePath(sc2X, sc2Y, r, tvAngle - theta + PI, tvAngle + PI / 2.0 + PI, nTRANSFER);
    }

    // pathADJUST 压入 Phase ADJUST: 直线段开始 -> 直线段结束的几个点
    double startX = 0;
    double startY = 0;
    double endX = 0;
    double endY = 0;
    if (min1DistIndex <= min2DistIndex) {
        startX = tvX + calcNorm2(l, d) * cos(tvAngle + atan(d / l));
        startY = tvY + calcNorm2(l, d) * sin(tvAngle + atan(d / l));
        endX = tvX + calcNorm2(l, d) * cos(tvAngle + PI - atan(d / l));
        endY = tvY + calcNorm2(l, d) * sin(tvAngle + PI - atan(d / l));
    } else {
        startX = tvX + calcNorm2(l, d) * cos(tvAngle + atan(d / l) + PI);
        startY = tvY + calcNorm2(l, d) * sin(tvAngle + atan(d / l) + PI);
        endX = tvX + calcNorm2(l, d) * cos(tvAngle + PI - atan(d / l) + PI);
        endY = tvY + calcNorm2(l, d) * sin(tvAngle + PI - atan(d / l) + PI);
    }
    pathADJUST = planLinePath(startX, startY, endX, endY, nADJUST);

    return;
}


void USVPathPlanner::updateAdjustPath(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle) {
    if (!isPathUpdateAllowed) {
        printf("未执行首次规划路径，不允许更新路径\n");
        return;
    }

    clearDockPath(false, false, false, true);
    calcDockPathParameters(usvX, usvY, tvX, tvY, tvRectL, tvRectW, tvAngle);

    // TODO
}

vector<vector<double> > USVPathPlanner::planLinePath(double startX, double startY, double endX, double endY, double pNum) {
    vector<vector<double> > path;
    path.reserve(2 * pNum);

    double pathX = 0;
    double pathY = 0;
    double balanceFactor = 0;
    for (double i = 0; i < pNum; i++) {
        balanceFactor = (pNum - 1.0 - i) / (pNum - 1.0);
        pathX = balanceFactor * startX + (1.0 - balanceFactor) * endX;
        pathY = balanceFactor * startY + (1.0 - balanceFactor) * endY;
        path.push_back({pathX, pathY});
    }

    return path;
}

vector<vector<double> > USVPathPlanner::planCirclePath(double cirX, double cirY, double cirR, double startAngle, double endAngle, double pNum) {
    vector<vector<double> > path;
    path.reserve(2 * pNum);

    double pathX = 0;
    double pathY = 0;
    double dAngle = endAngle - startAngle;

    // 如果角度差大于等于 2PI，那么只规划一个完整的圆
    // 如果角度差是一个完整的圆 2PI，那么不要最后一个点
    if (dAngle >= 2 * PI) {
        pNum = pNum - 1;
        dAngle = 2 * PI;
    }

    for (double i = 0; i < pNum; i++) {
        pathX = cirX + cirR * cos(startAngle + i / (pNum - 1) * dAngle);
        pathY = cirY + cirR * sin(startAngle + i / (pNum - 1) * dAngle);
        path.push_back({pathX, pathY});
    }

    return path;
}

inline void USVPathPlanner::calcDockPathParameters(double usvX, double usvY, double tvX, double tvY, double tvRectL, double tvRectW, double tvAngle) {

    if (l <= 0 || l <= 0.5 * tvRectL) {
        printf("路径规划参数 l = %.2f 小于测量目标船长的一半 %.2f，增加 l 至 %.2f\n", l, 0.5 * tvRectL, 0.5 * tvRectL + 6);
        l = 0.5 * tvRectL + 6;
    }

    if (d <= 0 || d <= 0.5 * tvRectW) {
        printf("路径规划参数 d = %.2f 小于测量目标船宽的一半 %.2f，增加 d 至 %.2f\n", d, 0.5 * tvRectW, 0.5 * tvRectW + 1);
        d = 0.5 * tvRectW + 1;
    }

    if (r - d <= 0) {
        printf("路径规划参数 r = %.2f 小于 d = %.2f，增加 r 至 %.2f\n", r, d, 1.2 * d);
        r = 1.2 * d;
    }

    R = r + calcNorm2(r - d, l);
    theta = atan((r - d) / l);

    sc1X = tvX + (R - r) * cos(tvAngle - theta);
    sc1Y = tvY + (R - r) * sin(tvAngle - theta);

    sc2X = tvX + (R - r) * cos(tvAngle - theta + PI);
    sc2Y = tvY + (R - r) * sin(tvAngle - theta + PI);

    transferPoint1X = tvX + R * cos(tvAngle - theta);
    transferPoint1Y = tvY + R * sin(tvAngle - theta);

    transferPoint2X = tvX + R * cos(tvAngle - theta + PI);
    transferPoint2Y = tvY + R * sin(tvAngle - theta + PI);

    usvTVAngle = atan2(usvY - tvY, usvX - tvX);
    
    if (R <= calcNorm2(usvX - tvX, usvY - tvY)) {  
        // 要保证 R <= calcNorm2(usvX - tvX, usvY - tvY)，即 USV 在圆外
        tangentPointAngle = usvTVAngle + acos(R / calcNorm2(usvX - tvX, usvY - tvY));  
    } else {
        // 若 USV 在圆内，则让 USV 掉头往回走，回到大圆上
        tangentPointAngle = 0;
    }  

    tangentPointX = tvX + R * cos(tangentPointAngle);
    tangentPointY = tvY + R * sin(tangentPointAngle);

    nAPPROACH = static_cast<int>(round(calcNorm2(usvX - tangentPointX, usvY - tangentPointY) / ds));
    nORBIT = static_cast<int>(round(2.0 * PI * R / ds));
    nTRANSFER = static_cast<int>(round((theta + PI / 2.0) * r / ds));
    nADJUST = static_cast<int>(round(2 * l / ds));

    return;
}

inline void USVPathPlanner::clearPursuePath() {
    pathPURSUE.clear();
    return;
}

inline void USVPathPlanner::clearDockPath(bool isClearAPPROACH, bool isClearORBIT, bool isClearTRANSFER, bool isClearADJUST) {
    if (isClearAPPROACH) {
        pathAPPROACH.clear();
    }

    if (isClearORBIT) {
        pathORBIT.clear();
    }

    if (isClearTRANSFER) {
        pathTRANSFER.clear();
    }

    if (isClearADJUST) {
        pathADJUST.clear();
    }

    return;
}