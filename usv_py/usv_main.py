#!/usr/bin/env python3

import rospy
import threading
import atexit
import traceback

from numpy import zeros, rad2deg, median, deg2rad, sin, cos, pi, abs, min, argmin
from numpy.linalg import norm

from usv_lidar import Lidar
from usv_pose import Pose
from usv_path_planner import PathPlanner, planCirclePath
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import rotationZ
from usv_record import genTable, USVData

from rich.console import Console
from rich.table import Column, Table

# 无人船状态定义
    # STARTUP
    # STANDBY
    # STABLE
    # PURSUE
    # PURSUE_DETECT_OBS
    # DOCK_PREPARE
    # DOCK_APPROACH
    # DOCK_MEASURE
    # DOCK_TRANSFER
    # DOCK_ADJUST
    # DOCK_FINAL
    # ATTACH
    # STOP
# 以下是测试状态
    # TEST_LINE
    # TEST_CIRCLE
    # TEST_BOTH
    # TEST_VEC_STABLE

TEST_MODE = "TEST_VEC_STABLE"

# ROS 定频
ROSRATE = 10

@atexit.register 
def clean():
    print("\n程序退出...")
    # rospy.signal_shutdown("End of USV main node.")

def main(args=None):
    # 添加主节点
    rospy.init_node('usv_main_node')

    rosRate = rospy.Rate(ROSRATE)

    # 添加功能节点
    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control(ROSRATE)
    usvData = USVData(ROSRATE)
 
    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    
    # 初始化标志位
    isDockMeasurePlan = False
    isDockTransferPlan = False
    isTestLinePlan = False
    isTestCirclePlan = False
    isTestEnable = False

    # 无人船状态
    usvState = "STARTUP"

    # 无人船当前正在使用的路径
    currPath = zeros((2000, 2))

    # 计数器
    usvStationTimer = 1

    # t0
    t0 = rospy.Time.now().to_sec()

    # 控制台输出初始化
    console = Console()
    latestMsg = "Waiting USV self-check."

    # Set point 
    uSP = float("nan")
    vSP = float("nan")
    psiSP = float("nan")

    # 试一下
    try:
        while not rospy.is_shutdown():
            if usvState == "STARTUP":
                if (usvPose.isGPSValid) & (usvPose.isImuValid) & (usvPose.isDvlValid) & (usvPose.isPodValid):
                    latestMsg = "Waiting sUAV to send heading."
                    usvState = "STANDBY"
 
            elif usvState == "STANDBY":
                if (isTestEnable):
                    usvState = TEST_MODE

                if (usvComm.isSearchFindTV):
                    latestMsg = "Receive heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)
                    usvState = "PURSUE"
                    
            elif usvState == "PURSUE":
                # 如果吊舱找到目标船，则进入 DOCK
                if (usvPose.isPodFindTV):
                    latestMsg = "Pod finds target vessel."
                    usvState = "DOCK_APPROACH"
                    continue 

                # 如果没有找到目标船，则继续跟随追踪路径
                uSP = 3.25
                usvControl.moveUSV(uSP, usvComm.tvAngleEst, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == "PURSUE_DETECT_OBS":
                # 读取激光雷达信息 
                pass
                
                # 判断是否还需要避障
                if (True):       
                    pass
                else:
                    print("USV 避障完成，恢复追踪目标船.")
                    isPursuePlan = False
                    
                    usvState = "PURSUE"
            
            elif usvState == "DOCK_APPROACH":

                # 根据吊舱、激光雷达状态，生成控制指令
                if (usvPose.isLidarFindTV):
                    if (usvPose.tvDist < 15.0):
                        uSP = 0.15
                    elif (usvPose.tvDist > 100.0):
                        uSP = 3.0
                    else:
                        uSP = 0.15 + (3.0 - 0.15) * (usvPose.tvDist - 15.0) / (100.0 - 15.0)

                    psiSP = usvPose.psi + usvPose.tvAngleLidar

                elif (usvPose.isPodFindTV):
                    uSP = 3.25
                    psiSP = usvPose.psi + usvPose.tvAnglePod

                else:
                    latestMsg = "Pod & LIDAR lose target vessel, back to pursue."
                    usvState = "PURSUE"
                    continue
                
                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 如果接近段结束了，则 DOCK_FINAL（10月7、8、9日），或 DOCK_MEASURE（真正比赛）
                if (usvPose.isLidarFindTV) & (usvPose.tvDist < 12.0):
                    latestMsg = "Approach finished. Start to stablize the USV."
                    usvState = "DOCK_FINAL"
                    # usvState = "DOCK_MEASURE"
                    continue

            elif usvState == "DOCK_MEASURE":             
                # 使用激光雷达读取的位置信息，规划测量路径
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockMeasurePlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                [uSP, psiSP] = usvGuidance.guidance(2.0, 20.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 读取目标船的测量信息，若满足要求，则读取并保存目标船朝向角（ENU下）
                tvAngle = 0

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_TRANSFER"

            elif usvState == "DOCK_TRANSFER":
                # 使用激光雷达读取的位置信息，规划变轨路径
                if (isDockTransferPlan == False):
                    currPath = usvPathPlanner.planDockTransfer(usvPose.xLidar, usvPose.yLidar, 0, 0, tvAngle)
                    usvGuidance.setPath(currPath) 
                    isDockTransferPlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                [xSP, ySP, psiSP] = usvGuidance.guidanceVec(12.0, 3.0, usvPose.xLidar, usvPose.yLidar)

                # 控制无人船（矢量）
                usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    # print("\n变轨完成")
                    usvState = "DOCK_ADJUST"
             
            elif usvState == "DOCK_ADJUST":
                # 在 ADJUST 段，读取大物体方位角，在大物体侧停下来
                usvState = "DOCK_FINAL"

            elif usvState == "DOCK_FINAL":
                # 等待船接近静止再发送起飞指令
                if (usvStationTimer >= 5 * ROSRATE):
                    latestMsg = "Stablization finished. Start to send take-off flag."
                    usvComm.sendTakeOffFlag()
                    usvComm.sendTVPosFromLidar(-usvPose.xLidar, -usvPose.yLidar)
                    continue
                elif (abs(usvPose.uDVL) <= 0.2):
                    usvStationTimer = usvStationTimer + 1 
                else:
                    usvStationTimer = 0

                # 保持静止
                psiSP = usvPose.psi + usvPose.tvAngleLidar
                usvControl.moveUSV(0, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            # 测试部分
            elif usvState == "TEST_LINE":
                if (isTestLinePlan == False):
                    endX = -50
                    endY = 50
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    print("USV 测试-直线路径已规划. 前往 [%d, %d]." % (endX, endY))
                    isTestLinePlan = True       

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):    
                    print("\nUSV 测试-直线结束.")
                    break

                [uSP, psiSP] = usvGuidance.guidance(3.5, 20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)
            
            elif usvState == "TEST_CIRCLE":
                if (isTestCirclePlan == False):
                    R = 15
                    circleTimes = 3
                    cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
                    cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
                    currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
                    usvGuidance.setPath(currPath)
                    # print("USV 测试-圆路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes))
                    isTestCirclePlan = True

                # print("\rUSV 状态：测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = " ")
                
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):  
                    # print("\nUSV 测试-圆结束.")  
                    break
                
                # R = 30m, dist2Next = 15m, uSP = 3m/s
                # R = 15m, dist2Next = 7m, uSP = 2.6m/s
                [uSP, psiSP] = usvGuidance.guidance(2.6, 7, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == "TEST_BOTH": 
                if (isTestLinePlan == False) & (isTestCirclePlan == False):
                    lineLength = 200
                    endX = usvPose.x + lineLength*cos(usvPose.psi);
                    endY = usvPose.y + lineLength*sin(usvPose.psi);
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    # print("USV 测试-直线+圆的直线段路径已规划. 前往 [%.2f, %.2f]m." % (endX, endY))
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
                    # print("\nUSV 测试-直线+圆的圆段路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes))    
                    isTestCirclePlan = True
                    theSpeed = 3
                    theDist2Next = 16

                # print("\rUSV 状态：测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = " ")

                if (usvGuidance.currentIdx >= usvGuidance.endIdx) & (isTestCirclePlan == True) & (isTestLinePlan == True):  
                    # print("\nUSV 测试-直线+圆结束.")  
                    break
                
                [uSP, psiSP] = usvGuidance.guidance(theSpeed, theDist2Next, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

            # 矢量推力 测试部分
            elif usvState == "TEST_VEC_STABLE":
                # print("\rUSV 状态：矢量推力-自稳测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = " ")
                usvControl.moveUSV(3, deg2rad(-20), usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == "STABLE":
                # print("\rUSV 状态：自稳 | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = " ")
                psiSP = usvPose.psi + usvPose.tvAngleLidar
                usvControl.moveUSV(0, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            else:
                # 程序不应该执行到这里
                print("\n变量 [usvState] 取到异常值 %d，请检查程序." % (usvState))
                break
            
            # 打印当前状态
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvPose, usvComm, dt) 
            console.print(theTable)

            # 写入当前状态到文件
            usvData.saveData(dt, usvPose, usvComm, uSP, vSP, psiSP)

            # 发送无人船的东西 
            usvComm.sendUSVState(usvState)

            rosRate.sleep()

        # 程序不应该执行到这里
        console.print("\n[red]程序跳出主循环.")
        console.print(locals())

    except KeyboardInterrupt:
        console.print("\n[red]检测到 Ctrl + C，退出 ...")
        
    except Exception as e:
        console.print("\n[red]程序异常，请检查.")
        console.print_exception(show_locals=True)
         
    return

if __name__ == '__main__':
    main()
