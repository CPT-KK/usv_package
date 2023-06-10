#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

const double PI = 3.1415926535;

inline double sign(double x) {
    const double tol = 1e-6;
    if (x > tol) {
        return 1;
    } else if (x < -tol) {
        return -1;
    } else {
        return 0;
    }
}

inline double calcNorm2(double dx, double dy) {
    return sqrt(pow(dx, 2.0) + pow(dy, 2.0));
}

inline double wrapPI(double angle){
    while (fabs(angle) > PI) {
        angle = angle - sign(angle) * 2.0 * PI;
    }
    return angle;
}

inline void frameTF2D(double x, double y, double angle, double& xNew, double & yNew) {
    xNew =  x * cos(angle) + y * sin(angle);
    yNew = -x * sin(angle) + y * cos(angle); 
}

inline void frameTF2D(float x, float y, float angle, float& xNew, float & yNew) {
    xNew =  x * cos(angle) + y * sin(angle);
    yNew = -x * sin(angle) + y * cos(angle); 
}

inline double satuator(double input, double inputMax){
    if (fabs(input) > inputMax) {
        return sign(input) * inputMax;
    } else {
        return input;
    }
}

inline double rad2deg(double radAngle) {
    return radAngle / PI * 180.0;
}

inline double deg2rad(double degAngle) {
    return degAngle / 180.0 * PI;
}

inline float rad2deg(float radAngle) {
    return radAngle / PI * 180.0f;
}

inline float deg2rad(float degAngle) {
    return degAngle / 180.0f * PI;
}

inline void getLineParams(double x0, double y0, double x1, double y1, double& A, double& B, double& C) {
    // Get the line Ax + By + C = 0 from two points (x0, y0) and (x1, y1)
    if (x1 - x0 == 0 && y1 - y0 == 0) {
        x0 = x0 * 3.0;
        y0 = y0 * 3.0;
    }
    A = y1 - y0;
    B = -(x1 - x0);
    C = -x0*(y1 - y0) + y0*(x1 - x0);

    return;
}

inline void getProjection(double x, double y, double A, double B, double C, double& xp, double& yp) {
    // Get the projection (xp, yp) of point (x, y) to the line Ax + By + C = 0
    xp = (A * C + B * (-B * x + A * y))/(- pow(A, 2.0) - pow(B, 2.0));
    yp = (B * C - A * (-B * x + A * y))/(- pow(A, 2.0) - pow(B, 2.0));
    return;
}

inline double getPoint2LineDist(double x, double y, double lx0, double ly0, double lxf, double lyf) {
    // Get the distance of point (x, y) to the line Ax + By + C = 0
    double ldy = lyf - ly0;
    double ldx = lxf - lx0;
    return fabs((ldy*x + ldx*y + lx0*ldy + ly0*ldx)/calcNorm2(ldx, ldy));
}

inline double median(std::vector<double> inVec) {
    if (inVec.size() == 0) {
        return 0.0;
    }
    
    std::sort(inVec.begin(), inVec.end());
    if (inVec.size() % 2 == 0) {
        return 0.5 * (inVec[inVec.size() / 2] + inVec[inVec.size() / 2 + 1]);
    } else {
        return inVec[(inVec.size() + 1) / 2];
    }
}
inline double mean(std::vector<double> inVec) {
    double sum = accumulate(inVec.begin(), inVec.end(), 0.0);
    if (inVec.size() == 0) {
        return 0.0;
    } else {
        return sum / static_cast<double>(inVec.size());
    }
    
}