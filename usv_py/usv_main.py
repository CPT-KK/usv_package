#!/usr/bin/env python3

import rospy
import threading
import atexit

from numpy import zeros, rad2deg, median, deg2rad, sin, cos, pi
from numpy.linalg import norm

from usv_lidar import Lidar
from usv_pose import Pose
from usv_path_planner import PathPlanner, planCirclePath
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import rotationZ

STARTUP = 0
STANDBY = 1
PURSUE = 10
PURSUE_DETECT_OBS = 11
PURSUE_EMERGENCY = 19
DOCK_PREPARE = 20
DOCK_APPROACH = 21
DOCK_MEASURE = 22
DOCK_TRANSFER = 23
DOCK_ADJUST = 24
DOCK_FINAL = 25
DOCK_EMERGENCY = 29
ATTACH = 31
STOP = 9

isTestEnable = True
TEST_LINE = 100
TEST_CIRCLE = 101
TEST_BOTH = 102
TEST_MODE = TEST_LINE

# ROS 定频
ROSRATE = 10

@atexit.register 
def clean():
    rospy.loginfo("程序退出...")
    rospy.signal_shutdown("End of USV main node.")

def main(args=None):
    # 添加主节点
    rospy.init_node('usv_main_node')
 
    rosRate = rospy.Rate(ROSRATE)

    # 添加功能节点
    usvPose = Pose()
    usvLidar = Lidar()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control(ROSRATE)
 
    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    
    # 初始化标志位
    isPursuePlan = False
    isDockApproachPlan = False
    isDockMeasurePlan = False
    isDockTransferPlan = False

    isTestLinePlan = False
    isTestCirclePlan = False

    # 无人船状态
    usvState = STARTUP

    # 无人船当前正在使用的路径
    currPath = zeros((2000, 2))

    # 目标船有关
    tvX = 0
    tvY = 0
    tvAngle = 0
    tvRecordNum = -1
    tvRecordAngle = zeros((4000, 1))
      
    # 试一下
    try:
        while not rospy.is_shutdown():
            if usvState == STARTUP:
                if (usvPose.isValid):
                    rospy.loginfo("收到 USV 位置.")
                    usvState = STANDBY
                else:
                    rospy.loginfo("等待传来 USV 位置.")

            elif usvState == STANDBY:
                if (isTestEnable):
                    rospy.loginfo("测试模式启动，不再接收目标船的估计位置.")
                    usvState = TEST_MODE

                if (usvComm.isTVEst):
                    rospy.loginfo("收到目标船的估计位置.")
                    usvState = PURSUE
                else:
                    rospy.loginfo("等待目标船的估计位置.")
                    
            elif usvState == PURSUE:
                # 如果需要规划追踪段路径，则规划并发送给制导
                if (isPursuePlan == False):
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, usvComm.tvEstPosX, usvComm.tvEstPosY)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 追踪段已规划.")
                    rospy.loginfo("当前状态：追踪段.")
                    isPursuePlan = True

                # 如果 USV 当前位置距离当前跟踪点太远，则重新规划（以防万一）
                if (norm([usvPose.x - usvGuidance.path[usvGuidance.currentIdx, 0], usvPose.y - usvGuidance.path[usvGuidance.currentIdx, 1]]) >= 40):
                    rospy.loginfo("USV 疑似脱离路径？立即重新规划追踪段.")
                    isPursuePlan = False
                    continue

                # 如果跑完了所有点都没有找到目标船，则重新规划（以防万一）
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    rospy.loginfo("跑完了追踪段但激光雷达仍没有发现目标船？立即重新规划追踪段.")
                    isPursuePlan = False
                    continue
     
                # 读取激光雷达信息 
                [idxTV, isTVFound, idxOBS, isObsFound] = usvLidar.objRead(usvPose.x, usvPose.y, usvPose.psi, usvPose.beta, usvComm.tvEstPosX, usvComm.tvEstPosY)
                
                # 如果找到目标船，则进入 DOCK
                if (isTVFound):
                    [tvX, tvY] = rotationZ(usvLidar.objInfo[idxTV, 0], usvLidar.objInfo[idxTV, 1], -usvPose.psi)
                    tvX = tvX + usvPose.x
                    tvY = tvY + usvPose.y
                    rospy.loginfo("激光雷达探测到目标船.")
                    rospy.loginfo("估计目标船在: [%.2f, %.2f]（USV 船体系下），方向角: %.4f (rad) 或 %.4f (deg)." % (usvLidar.objInfo[idxTV, 0], usvLidar.objInfo[idxTV, 1], usvLidar.objInfo[idxTV, 4], rad2deg(usvLidar.objInfo[idxTV, 4])))
                    usvState = DOCK_APPROACH
                    continue

                # 如果检测到障碍物，则进入 PURSUE_DETECT_OBS
                if (isObsFound):
                    rospy.loginfo("检测到障碍物，执行避障.")
                    usvState = PURSUE_DETECT_OBS
                    continue

                # 如果没有找到目标船，则继续跟随追踪路径
                [uSP, psiSP] = usvGuidance.guidance(3.0, 25.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            elif usvState == PURSUE_DETECT_OBS:
                # 读取激光雷达信息 
                [idxTV, isTVFound, idxOBS, isObsFound] = usvLidar.objRead(usvPose.x, usvPose.y, usvPose.psi, usvPose.beta, usvComm.tvEstPosX, usvComm.tvEstPosY)
                
                # 判断是否还需要避障
                if (isObsFound):       
                    [uSP, psiSP] = usvGuidance.guidanceOBS(usvLidar.objInfo[idxOBS, 0],usvLidar.objInfo[idxOBS, 1], usvLidar.objInfo[idxOBS, 5], usvPose.psi, usvPose.beta, 1.0)
                    usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r) 
                else:
                    rospy.loginfo("USV 避障完成，恢复追踪目标船.")
                    isPursuePlan = False
                    usvState = PURSUE

            # elif usvState == PURSUE_EMERGENCY:
            
            elif usvState == DOCK_APPROACH:
                if (isDockApproachPlan == False):
                    currPath = usvPathPlanner.planDockApproach(usvPose.x, usvPose.y, tvX, tvY)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 泊近-接近段路径已规划.")
                    rospy.loginfo("USV 状态：泊近-接近段.")
                    isDockApproachPlan = True
                        
                [uSP, psiSP] = usvGuidance.guidance(2.0, 20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 如果接近段结束了，则进入测量段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = DOCK_MEASURE
                    continue

            elif usvState == DOCK_MEASURE:
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.x, usvPose.y, tvX, tvY)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 泊近-测量段路径已规划.")
                    rospy.loginfo("USV 状态：泊近-测量段.")
                    isDockMeasurePlan = True
                    rospy.loginfo("开始测量目标船姿态.")
       
                [uSP, psiSP] = usvGuidance.guidance(2.0, 20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？）
                [idxTV, isTVFound, _, _] = usvLidar.objRead(usvPose.x, usvPose.y, usvPose.psi, usvPose.beta, usvComm.tvEstPosX, usvComm.tvEstPosY)

                # 如果目标船的测量信息满足要求，则读取并保存目标船朝向角（ENU下）
                if (usvLidar.objInfo[idxTV, 2] > 10) & (usvLidar.objInfo[idxTV, 3] > 5) & (usvLidar.objInfo[idxTV, 2] / usvLidar.objInfo[idxTV, 3] > 2.25):
                    tvRecordNum = tvRecordNum + 1
                    tvRecordAngle[tvRecordNum] = usvLidar.objInfo[idxTV, 4] + usvPose.psi

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    [tvX, tvY] = rotationZ(usvLidar.objInfo[idxTV, 0], usvLidar.objInfo[idxTV, 1], -usvPose.psi)
                    tvX = tvX + usvPose.x
                    tvY = tvY + usvPose.y
                    tvAngle = median(tvRecordAngle[0:tvRecordNum])
                    rospy.loginfo("测量的目标船姿态为: %.4f (rad)，%.4f (deg)." % (tvAngle, rad2deg(tvAngle)))
                    rospy.loginfo("结束测量目标船姿态.")
                    usvState = DOCK_TRANSFER

            elif usvState == DOCK_TRANSFER:
                if (isDockTransferPlan == False):
                    currPath = usvPathPlanner.planDockTransfer(currPath[-1, 0], currPath[-1, 1], tvX, tvY, tvAngle) # 用上一段路径的最后一个点作为起始点
                    usvGuidance.setPath(currPath) 
                    isDockTransferPlan = True
                    rospy.loginfo("USV 泊近-变轨段路径已规划.")
                    rospy.loginfo("USV 状态：泊近-变轨段.")
           
                [xSP, ySP, psiSP] = usvGuidance.guidanceVec(12.0, 3.0, usvPose.x, usvPose.y)
                usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = DOCK_ADJUST
             
            elif usvState == DOCK_ADJUST:
                # 在 ADJUST 段，读取大物体方位角，在大物体侧停下来
                rospy.loginfo("USV 状态：泊近-调整段.")
                usvState = DOCK_FINAL

            elif usvState == DOCK_FINAL:
                rospy.loginfo("USV 状态：泊近-最终段.")
                break

            elif usvState == TEST_LINE:
                if (isTestLinePlan == False):
                    endX = 0
                    endY = -100
                    # endX = -400
                    # endY = 300
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 测试-直线路径已规划. 前往 [%d, %d]." % (endX, endY))
                    rospy.loginfo("当前状态：测试-直线.")   
                    isTestLinePlan = True

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):    
                    rospy.loginfo("USV 测试-直线结束.")
                    break

                [uSP, psiSP] = usvGuidance.guidance(4, 20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)
            
            elif usvState == TEST_CIRCLE:
                if (isTestCirclePlan == False):
                    R = 15
                    circleTimes = 3
                    cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
                    cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
                    currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 测试-圆路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes))
                    rospy.loginfo("当前状态：测试-圆.")
                    isTestCirclePlan = True

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):  
                    rospy.loginfo("USV 测试-圆结束.")  
                    break
                
                # R = 30m, dist2Next = 15m, uSP = 3m/s
                # R = 15m, dist2Next = 7m, uSP = 2.6m/s
                [uSP, psiSP] = usvGuidance.guidance(2.6, 7, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            elif usvState == TEST_BOTH:
                
                if (isTestLinePlan == False) & (isTestCirclePlan == False):
                    lineLength = 200
                    endX = usvPose.x + lineLength*cos(usvPose.psi);
                    endY = usvPose.y + lineLength*sin(usvPose.psi);
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 测试-直线+圆的直线段路径已规划. 前往 [%.2f, %.2f]m." % (endX, endY))
                    rospy.loginfo("当前状态：测试-直线+圆 | 直线.")
                    isTestLinePlan = True
                    theSpeed = 4
                    theDist2Next = 20

                if (usvGuidance.currentIdx >= usvGuidance.endIdx) & (isTestLinePlan == True) & (isTestCirclePlan == False):  
                    R = 30
                    circleTimes = 3
                    cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
                    cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
                    currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
                    usvGuidance.setPath(currPath)
                    rospy.loginfo("USV 测试-直线+圆的圆段路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes))    
                    rospy.loginfo("当前状态：测试-直线+圆 | 圆.")
                    isTestCirclePlan = True
                    theSpeed = 3
                    theDist2Next = 16

                if (usvGuidance.currentIdx >= usvGuidance.endIdx & isTestCirclePlan == True & isTestLinePlan == True):  
                    rospy.loginfo("USV 测试-直线+圆结束.")  
                    break
                
                [uSP, psiSP] = usvGuidance.guidance(theSpeed, theDist2Next, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)
                
            # elif usvState == DOCK_EMERGENCY:

            # elif usvState == STOP:

            # elif usvState == ATTACH:

            else:
                # 程序不应该执行到这里
                rospy.loginfo("变量 [usvState] 取到异常值 %d，请检查程序." % (usvState))
                break
            
            rosRate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("程序退出.")

    except Exception as e:
        rospy.loginfo(e)

    return

if __name__ == '__main__':
    main()
