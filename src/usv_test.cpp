// // C & C++ std headers
// #include <algorithm>
// #include <iostream>
// #include <memory>
// #include <string>
// #include <vector>

// // Self header
// #include "usv_main.h"

// int test() {
//     // Initialize ROS 2 node
//     rclcpp::init();

//     // Define sleep rates
//     rclcpp::WallRate loopRate(5);

//     // Define publish nodes

//     // Define subscribe nodes
//     auto usvPosePtr = std::make_shared<USVPose>();
//     auto lidarPointCloudPtr = std::make_shared<LIDARPointCloud>(usvPosePtr, 20);
//     auto usvPathPlannerPtr = std::make_shared<USVPathPlanner>();
//     auto usvControllerPtr = std::make_shared<USVControllerPID>(usvPosePtr, 5);
//     auto usvGuidancePtr = std::make_shared<USVGuidance>(usvPosePtr, usvControllerPtr);

//     // Update all subscribe nodes
//     rclcpp::spin_some(usvPosePtr);
//     rclcpp::spin_some(lidarPointCloudPtr);

//     double testEndX = 0.0;    //在ENU坐标系下直线测试路径的终点坐标x
//     double testEndY = 0.0;    //在ENU坐标系下直线测试路径的终点坐标y
//     double testR = 15;        //在ENU坐标系下圆弧测试路径的半径
//     double testTheta = 0.3;   //在ENU坐标系下圆弧测试路径的切入角
//     double testCenX = 0.0;    //在ENU坐标系下圆弧测试路径的圆心坐标x
//     double testCenY = 0.0;    //在ENU坐标系下圆弧测试路径的圆心坐标y
//     double testNum = 0.0;     //测试路径的离散点数
//     vector<vector<double> > testPath;

//     usvPosePtr->updateTVEstXY(500, 0);
//     while (!usvPosePtr->isValid) {
//         rclcpp::spin_some(usvPosePtr);
//         rclcpp::spin_some(lidarPointCloudPtr);
//         loopRate.sleep();
//     }

//     int test = 0;
//     cout << "直线1 or 圆2： ";
//     cin >> test;

//     cout << "最大 u： ";
//     cin >> usvGuidancePtr->uMax;

//     switch (test) {
//         case 1:  // 测试直线
//             rclcpp::spin_some(usvPosePtr);
//             rclcpp::spin_some(lidarPointCloudPtr);
//             testNum = ceil(500 / 4);
//             testPath = usvPathPlannerPtr->planLinePath(usvPosePtr->x, usvPosePtr->y + 10, usvPosePtr->x + 500, usvPosePtr->y + 10, testNum);
//             usvGuidancePtr->setPath(testPath);
//             while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
//                 usvGuidancePtr->guidance();
//                 rclcpp::spin_some(usvPosePtr);
//                 rclcpp::spin_some(lidarPointCloudPtr);
//                 loopRate.sleep();
//                 MOVEUP(5 + 5);
//             }
//             MOVEDOWN(5 + 5);
//             break;

//         default:  // 测试圆弧  
//             cout << "请输入圆弧半径： ";
//             cin >> testR;
//             usvGuidancePtr->R = testR * 0.5;
//             rclcpp::spin_some(usvPosePtr);
//             rclcpp::spin_some(lidarPointCloudPtr);
//             testCenX = usvPosePtr->x;
//             testCenY = usvPosePtr->y + testR;
//             testTheta = 0;
//             testNum = ceil(2 * PI * testR / 4.0);
//             testPath = usvPathPlannerPtr->planCirclePath(testCenX, testCenY, testR, testTheta - 0.5* PI, testTheta +  1.5 * PI, testNum);
//             usvGuidancePtr->setPath(testPath);
//             while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
//                 usvGuidancePtr->guidance();
//                 rclcpp::spin_some(usvPosePtr);
//                 rclcpp::spin_some(lidarPointCloudPtr);
//                 loopRate.sleep();
//                 if (usvGuidancePtr->currentIndex >= usvGuidancePtr->endIndex) {
//                     usvGuidancePtr->currentIndex = 0;
//                 }
//                 MOVEUP(5 + 5);
//             }
//             MOVEDOWN(5 + 5);
//             break;
//     }

//     rclcpp::shutdown();
//     return 0;
// }