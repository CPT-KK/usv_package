// #pragma once

// #include <iostream>
// #include <vector>
// #include <fstream>
// #include <memory>
// #include <math.h>

// #include "usv_guidance.h"
// #include "usefulMathFunc.h"

// class USVTest : USVGuidance {
//    public:
//     vector<int> testType;
    
//     vector<double> lineStartX;
//     vector<double> lineStartY;
//     vector<double> lineEndX;
//     vector<double> lineEndY;

//     vector<double> circleR;
//     vector<double> circleCenX;
//     vector<double> circleCenY;
//     vector<double> circleStartTheta;
//     vector<double> circleEndTheta;

//     double dsTestPath = 4;

//     USVTest() {
        
//     }

//     void addLinePath(double lineStartX_, double lineStartY_, double lineEndX_, double lineEndY_) {
//         testType.push_back(1);
//         lineStartX.push_back(lineStartX_);
//         lineStartY.push_back(lineStartY_);
//         lineEndX.push_back(lineEndX_);
//         lineEndY.push_back(lineEndY_);

//         return;
//     }

//     void addCirclePath(double circleCenX_, double circleCenY_, double circleR_, double circleStartTheta_, double circleEndTheta_) {
//         testType.push_back(2);
//         circleR.push_back(circleR_);
//         circleCenX.push_back(circleCenX_);
//         circleCenY.push_back(circleCenY_);
//         circleStartTheta.push_back(circleStartTheta_);
//         circleEndTheta.push_back(circleEndTheta_);

//         return;
//     }

//     void startTest() {
        
//         int testPathPoint = 0;
//         vector<vector<double> > testPath;

//         for (int i = 0; i < static_cast<int>(testType.size()) - 1; i++) {
//             switch (testType[i]) {
//             case 1:
//                 testPathPoint = ceil(calcNorm2(lineStartX[0] - lineEndX[0], lineStartY[0] - lineEndY[0]) / dsTestPath);
//                 testPath = planLinePath(lineStartX[0], lineStartY[0], lineEndX[0], lineEndY[0], testPathPoint);
//                 lineStartX.erase(lineStartX.begin());
//                 lineStartY.erase(lineStartY.begin());
//                 lineEndX.erase(lineEndX.begin());
//                 lineEndY.erase(lineEndY.begin());

//                 break;
            
//             case 2:
//                 testPathPoint = ceil(2 * PI * circleR[0] / dsTestPath);
//                 testPath = planCirclePath(circleCenX[0], circleCenY[0], circleR[0], circleStartTheta[0], circleEndTheta[0], testPathPoint);
//                 circleCenX.erase(circleCenX.begin());
//                 circleCenY.erase(circleCenY.begin());
//                 circleR.erase(circleR.begin());
//                 circleStartTheta.erase(circleStartTheta.begin());
//                 circleEndTheta.erase(circleEndTheta.begin());

//                 break;

//             default:
//                 break;
//             }
//         }
//     }
// };
