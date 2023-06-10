// Self header
#include "usv_main.h"

// 无人船当前状态
int usvState = USV_WAIT;

int main(int argc, char **argv) {
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);

    // 定义 ROS Loop 频率
    rclcpp::WallRate loopRate(5);

    // 初始化无人船状态类
    shared_ptr<USVPose> usvPosePtr = std::make_shared<USVPose>();

    // 初始化无人船激光雷达类
    shared_ptr<LIDARPointCloud> lidarPointCloudPtr = std::make_shared<LIDARPointCloud>(usvPosePtr, 20);

    // 初始化无人船路径规划类
    shared_ptr<USVPathPlanner> usvPathPlannerPtr = std::make_shared<USVPathPlanner>();

    // 初始化无人船控制类
    shared_ptr<USVControllerPID> usvControllerPtr = std::make_shared<USVControllerPID>(usvPosePtr, 5);

    // 初始化无人船制导类
    shared_ptr<USVGuidance> usvGuidancePtr = std::make_shared<USVGuidance>(usvPosePtr, usvControllerPtr);

    // 是否需要重新规划路径的标志位
    bool isPathNeedReplan = false;

    while (rclcpp::ok()) {
        switch (usvState) {
            case USV_WAIT:
                // 无人船先自检，检查自己是否拿到自己的位置
                printf("等待定位系统传来无人船自身位置...\n");
                while (!usvPosePtr->isValid) {
                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);
                    loopRate.sleep();
                }
                // 此阶段，无人船等待定位系统发来的目标船位置，并将其写入 usvPosePtr 中
                printf("等待定位系统传来目标船位置...\n");
                while (!usvPosePtr->isTVEstXYValid) {
                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);
                    loopRate.sleep();
                }
                
                printf("收到定位系统传来的目标船位置，无人船启动\n");
                printf("\n");
                usvState = PURSUE;
                break;

            case PURSUE:
                // 在 PURSUE 段，先检查是否找到目标船，如果没有找到目标船，则开始规划追踪路径，如果找到了，则进入 DOCK_PREPARE
                printf("当前状态：追踪段.\n");

                // 规划追踪段路径
                usvPathPlannerPtr->planPursuePath(usvPosePtr->x, usvPosePtr->y, 0.0, 0.0);
                usvGuidancePtr->setPath(usvPathPlannerPtr->pathPURSUE);
                // 如果没有找到目标船，则继续跟随追踪路径
                while (!lidarPointCloudPtr->isLidarFindTV && usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
                    usvGuidancePtr->guidance(3, 12);
                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);
                    loopRate.sleep();
                    MOVEUP(5 + 5);
                }
                MOVEDOWN(5 + 5);

                   
                // 若找到目标船，则进入 DOCK_PREPARE
                printf("激光雷达探测到目标船.\n");
                printf("估计目标船在: [%.2f, %.2f]（USV 船体系下），方向角: %.4f (rad) 或 %.4f (deg).\n", lidarPointCloudPtr->tvX, lidarPointCloudPtr->tvY,lidarPointCloudPtr->tvAngle, rad2deg(lidarPointCloudPtr->tvAngle));
                printf("\n");
                usvState = DOCK_PREPARE;
                break;

            case PURSUE_DETECT_OBS:
                usvPathPlannerPtr->PathAvoidOBS(usvPosePtr->x, usvPosePtr->y, usvPosePtr->yaw, lidarPointCloudPtr->obsXYR[0][0], lidarPointCloudPtr->obsXYR[0][1], lidarPointCloudPtr->obsXYR[0][2]);
                break;

            case PURSUE_EMERGENCY:
                /* code */
                break;

            case DOCK_PREPARE:
                // 在 DOCK_PREPARE 段，先规划泊近段路径，然后切换状态至 DOCK_APPROACH 段
                printf("当前状态：泊近-准备段.\n");

                // 规划泊近路径
                rclcpp::spin_some(usvPosePtr);
                rclcpp::spin_some(lidarPointCloudPtr);
                usvPathPlannerPtr->planDockPath(usvPosePtr->x, usvPosePtr->y, 0.0, 0.0, lidarPointCloudPtr->tvRectL, lidarPointCloudPtr->tvRectW, lidarPointCloudPtr->tvAngle);
                
                // 如果质量不佳，要准备重新规划泊近路径
                if (lidarPointCloudPtr->tvRectLWRatio < 2.5) {
                    isPathNeedReplan = true;
                    printf("USV 泊近路径已规划，但需重新估计目标船方向角.\n");
                } else {
                    printf("USV 泊近路径已规划.\n");
                }
                printf("\n");
                usvState = DOCK_APPROACH;
                break;

            case DOCK_APPROACH:
                // 在 DOCK_APPROACH 段，跟踪 DOCK_APPROACH 路径，完成后切换状态至 DOCK_ORBIT 段
                printf("当前状态：泊近-接近段.\n");
                printf("下一状态: 泊近-环绕段，路径起点 [%08.2f, %08.2f].\n", usvPathPlannerPtr->pathORBIT[0][0], usvPathPlannerPtr->pathORBIT[0][1]);
                usvGuidancePtr->setPath(usvPathPlannerPtr->pathAPPROACH);
                while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
                    usvGuidancePtr->guidance(1.8, 12);
                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);   
                    loopRate.sleep();
                    MOVEUP(5 + 5);
                }

                MOVEDOWN(5 + 5);
                printf("\n");
                usvState = DOCK_ORBIT;
                break;

            case DOCK_ORBIT:
                if (isPathNeedReplan) {
                    // 如果矩形质量不好，要重新走1/4圈 ORBIT 测量矩形
                    printf("USV 状态：泊近-环绕段（重新测量目标船）.\n");

                    lidarPointCloudPtr->startTVRecord();
                    usvGuidancePtr->setPath(usvPathPlannerPtr->pathORBIT, static_cast<int>(round(0.666667 * static_cast<double>(usvPathPlannerPtr->nORBIT))));
                    while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
                        usvGuidancePtr->guidance(1.25, 15);
                        rclcpp::spin_some(usvPosePtr);
                        rclcpp::spin_some(lidarPointCloudPtr);   
                        loopRate.sleep();
                        MOVEUP(5 + 5);
                    }

                    MOVEDOWN(5 + 5);

                    lidarPointCloudPtr->calcBestRecord();
                    lidarPointCloudPtr->endTVRecord();
                    usvPathPlannerPtr->updateDockPath(usvPosePtr->x, usvPosePtr->y, 0.0, 0.0, lidarPointCloudPtr->tvRecordBestLen, lidarPointCloudPtr->tvRecordBestWid, lidarPointCloudPtr->tvRecordBestAngle);
                    isPathNeedReplan = false;  
                    printf("路径重新规划.\n");
                    printf("重新估计的目标船姿态为: %.4f (rad)，%.4f (deg).\n", lidarPointCloudPtr->tvRecordBestAngle, rad2deg(lidarPointCloudPtr->tvRecordBestAngle));
                    printf("\n");
                }
                
  
                // DOCK_ORBIT->DOCK_TRANSFER
                printf("USV 状态：泊近-环绕段.\n");
                printf("下一段路径: 变轨段，路径起点 [%08.2f, %08.2f].\n", usvPathPlannerPtr->pathTRANSFER[0][0], usvPathPlannerPtr->pathTRANSFER[0][1]);
                usvGuidancePtr->setPath(usvPathPlannerPtr->pathORBIT, usvPathPlannerPtr->minDist2TransferIndex - 1);
                while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
                    usvGuidancePtr->guidance(1.25, 15);

                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);   
                    loopRate.sleep();
                    MOVEUP(5 + 5);
                }

                MOVEDOWN(5 + 5);
                printf("\n");
                usvState = DOCK_TRANSFER;
                break;

            case DOCK_TRANSFER:
                printf("USV 状态：泊近-变轨段.\n");
                printf("下一段路径: 调整段，路径起点 [%08.2f, %08.2f].\n", usvPathPlannerPtr->pathADJUST[0][0], usvPathPlannerPtr->pathADJUST[0][1]);
                usvGuidancePtr->setPath(usvPathPlannerPtr->pathTRANSFER);
                while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
                    usvGuidancePtr->guidance(0.55, 8.5);

                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);   
                    loopRate.sleep();
                    MOVEUP(5 + 5);
                }

                MOVEDOWN(5 + 5);
                printf("\n");
                usvState = DOCK_ADJUST;
                break;

            case DOCK_ADJUST:
                // 在 ADJUST 段，读取激光雷达传来的船的方向角数据，重新规划一次 ADJUST 的路径
                printf("USV 状态：泊近-调整段.\n");
                printf("下一段路径: 泊近-最终段.\n");
                usvGuidancePtr->setPath(usvPathPlannerPtr->pathADJUST);
                while (usvGuidancePtr->currentIndex < usvGuidancePtr->endIndex) {
                    usvGuidancePtr->guidance(0.3, 8);

                    rclcpp::spin_some(usvPosePtr);
                    rclcpp::spin_some(lidarPointCloudPtr);   
                    loopRate.sleep();
                    MOVEUP(5 + 5);
                }

                MOVEDOWN(5 + 5);
                usvState = DOCK_FINAL;
                break;

            case DOCK_FINAL:
                printf("USV 状态：泊近-最终段.\n");
                usvGuidancePtr->guidance(0, 6);

                break;

            case DOCK_EMERGENCY:
                /* code */
                break;

            case USV_STOP:
                printf("USV 状态：泊近-停止段.\n");
                /* code */
                break;

            case ATTACH:
                /* code */
                break;

            default:
                break;
        }

        rclcpp::spin_some(usvPosePtr);
        rclcpp::spin_some(lidarPointCloudPtr);
        loopRate.sleep();
    }
    // Shutdown ROS 2 node
    rclcpp::shutdown();

    return 0;
}