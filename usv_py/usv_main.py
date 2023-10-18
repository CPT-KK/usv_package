#!/usr/bin/env python3

import rospy
import threading
import atexit
import traceback
import signal

from numpy import zeros, rad2deg, median, deg2rad, sin, cos, pi, abs, min, argmin, mean, tan, arctan
from numpy.linalg import norm

from usv_can import USVCAN
from usv_pose import Pose
from usv_path_planner import PathPlanner, planCirclePath
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import removeOutliers, wrapToPi
from usv_record import genTable, USVData

from rich.console import Console
from rich.table import Column, Table

from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# 无人船状态定义
    # STARTUP
    # STANDBY
    # STABLE
    # PURSUE
    # PURSUE_OBS
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

TEST_MODE = "TEST_LINE"

# ROS 定频
ROS_RATE = 10

@atexit.register 
def clean():
    print(">>>>>>> USV program has exited.")

def interuptFunc(signum, frame):
    console = Console()
    console.print("\n[red]>>>>>>> Ctrl + C pressed! Exiting...")
    exit()

def main(args=None):
    # 控制台输出初始化
    console = Console()
    latestMsg = "Waiting USV self-check to complete..."
    console.print("[green]>>>>>>> Console initialized.")

    # 注册 Ctrl + C
    signal.signal(signal.SIGINT, interuptFunc)
    signal.signal(signal.SIGTERM, interuptFunc)
    console.print("[green]>>>>>>> Interrupt function initialized.")

    # 添加主节点
    rospy.init_node('usv_main_node', anonymous=True)
    rosRate = rospy.Rate(ROS_RATE)
    console.print("[green]>>>>>>> ROS node initialized.")

    # 添加功能类
    usvCAN = USVCAN()
    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance(ROS_RATE)
    usvControl = Control(ROS_RATE)
    usvData = USVData(ROS_RATE)
    console.print("[green]>>>>>>> Function classes initialized.")

    # 添加服务节点
    console.print("[green]>>>>>>> Waiting MAVROS services...")
    rospy.wait_for_service("/mavros/cmd/arming")
    mavrosArmClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    armCmd = CommandBoolRequest(value=True)
    disarmCmd = CommandBoolRequest(value=False)

    rospy.wait_for_service("/mavros/set_mode")
    mavrosSetModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)
    holdSetMode = SetModeRequest(custom_mode='AUTO.LOITER')
    console.print("[green]>>>>>>> Connect to MAVROS services.")

    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    console.print("[green]>>>>>>> ROS spin started.")
    
    # 初始化标志位
    isDockApproachPlan = False
    isDockMeasurePlan = False
    isDockTransferPlan = False
    isTestLinePlan = False
    isTestCirclePlan = False
    isTestEnable = False

    # 无人船状态
    usvState = "STANDBY"

    # 无人船当前正在使用的路径
    currPath = zeros((2000, 2))

    # 计数器 
    t0 = rospy.Time.now().to_sec()
    t1 = rospy.Time.now().to_sec()

    tvAngle = zeros((1, 5000))
    tvAngleIdx = 0

    # Set point 量
    uSP = float("nan")
    vSP = float("nan")
    psiSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")

    # 试一下  
    while not rospy.is_shutdown():
        try:
            if usvState == "STARTUP":
                # 单独为激光雷达设置启动检查
                pubTopicList = sum(rospy.get_published_topics(), [])
                usvPose.isLidarValid = ('/filter/target' in pubTopicList)
                    
                if (usvPose.isImuValid) & (usvPose.isDvlValid) & (usvPose.isPodValid) & (usvPose.isLidarValid):
                    latestMsg = "Waiting sUAV to send heading..."
                    usvState = "STANDBY"

            elif usvState == "STANDBY":
                if (isTestEnable):
                    usvState = TEST_MODE

                if (usvComm.isSearchFindTV):
                    latestMsg = "Receive heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)

                    usvState = "PURSUE"

                    if (not usvPose.state.armed) & (mavrosArmClient.call(armCmd).success):
                        pass

            elif usvState == "PURSUE":
                # 如果吊舱找到目标船，则进入 DOCK
                if (usvPose.isPodFindTV):
                    latestMsg = "Pod finds target vessel!"
                    usvState = "DOCK_APPROACH"
                    continue 

                # 如果障碍物，进入 PURSUE_OBS
                if (usvPose.isLidarFindObs):
                    latestMsg = "Obstacle detected!"
                    usvState = "PURSUE_OBS"
                    continue 
                
                # 如果没有找到目标船，则继续跟随追踪路径
                uSP = 3.25
                psiSP = usvComm.tvAngleEst
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == "PURSUE_OBS":
                # 读取激光雷达信息 
                pass
                
                # 判断是否还需要避障
                if (usvPose.isLidarFindObs):    
                    # 计算避障所需航向角
                    if (usvPose.obsAngleLidar >= 0):
                        psiSP = usvPose.obsAngleLidar + usvPose.psi - deg2rad(12.0)
                    else:
                        psiSP = usvPose.obsAngleLidar + usvPose.psi + deg2rad(12.0)
                    
                    uSP = 3.25
                    usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)
                else:
                    latestMsg = "Back to follow heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)     
                    usvState = "PURSUE"
                    continue
            
            elif usvState == "DOCK_APPROACH":

                # 根据吊舱、激光雷达状态，生成控制指令
                if (usvPose.isLidarFindTV):
                    latestMsg = "Lidar finds target vessel!"
                    if (usvPose.tvDist < 15.0):
                        uSP = 1.0
                    elif (usvPose.tvDist > 100.0):
                        uSP = 3.0
                    else:
                        uSP = 1.0 + (3.0 - 1.0) * (usvPose.tvDist - 15.0) / (100.0 - 15.0)

                    psiSP = usvPose.tvAngleLidar

                elif (usvPose.isPodFindTV):
                    uSP = 3.25
                    psiSP = usvPose.tvAnglePod

                else:
                    latestMsg = "Pod & Lidar lose target vessel, fall back to pursue."
                    usvState = "PURSUE"
                    continue
                
                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 如果接近段结束了，则 DOCK_MEASURE
                if (usvPose.isLidarFindTV) & (usvPose.tvDist < 60.0):
                    latestMsg = "Approach finished. Start to measure the USV..."
                    usvState = "DOCK_ALIGN"
                    continue

            elif usvState == "DOCK_ALIGN":
                if (isDockApproachPlan == False):
                    currPath = usvPathPlanner.planDockApproach(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockApproachPlan = True
                    latestMsg = "Start align with the measure circle ..."
                
                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(1.7, 12.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_MEASURE"
            
            elif usvState == "DOCK_MEASURE":             
                # 使用激光雷达读取的位置信息，规划测量路径
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockMeasurePlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(1.5, 8.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 读取目标船的测量信息，若满足要求，则读取并保存目标船朝向角（ENU下）
                thisTVAngle = usvPose.tvHeading
                if abs(thisTVAngle) > deg2rad(85):
                    thisTVAngle = deg2rad(90)
                tvAngle[0, tvAngleIdx] = thisTVAngle
                tvAngleIdx = tvAngleIdx + 1
                latestMsg = "Estimating target vessel heading: %.2f deg." % rad2deg(thisTVAngle)

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    # 去除离群点
                    tvAngle = removeOutliers(tvAngle[0, 0:tvAngleIdx-1])
                    # 计算平均值
                    tvAngleMean = mean(tvAngle)
                    
                    latestMsg = "Estimating finished with average heading %.2f deg. Transfer to the target vessel..." % rad2deg(tvAngleMean)
                    usvState = "DOCK_TRANSFER"

            elif usvState == "DOCK_TRANSFER":
                # 使用激光雷达读取的位置信息，规划变轨路径
                if (isDockTransferPlan == False):
                    currPath = usvPathPlanner.planDockTransfer(usvPose.xLidar, usvPose.yLidar, 0, 0, tvAngleMean)
                    usvGuidance.setPath(currPath) 
                    isDockTransferPlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = 1.5 - 0.8 * (usvGuidance.currentIdx / usvGuidance.endIdx)
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(uSP, 5.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx) | (usvPose.tvDist < 5.0):
                    latestMsg = "Transfer finished. Final adjusting..."
                    usvState = "DOCK_ADJUST"
                
            elif usvState == "DOCK_ADJUST":
                # 在 ADJUST 段，读取大物体方位角，在大物体侧停下来
                usvState = "DOCK_FINAL"
                t1 = rospy.Time.now().to_sec()

            elif usvState == "DOCK_FINAL":
                # 等待船接近静止再发送起飞指令
                if (usvPose.state.armed):
                    mavrosArmClient.call(disarmCmd)
                    
                if (rospy.Time.now().to_sec() - t1 >= 5.0):
                    latestMsg = "Stablization finished. Sending the take-off flag..."
                    usvComm.sendTakeOffFlag()
                    usvComm.sendTVPosFromLidar(-usvPose.xLidar, -usvPose.yLidar)
                elif (abs(usvPose.uDVL) <= 0.2):
                    pass
                else:
                    t1 = rospy.Time.now().to_sec()

                # 保持静止
                uSP = 0.0
                psiSP = tvAngleMean
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            # 测试部分
            elif usvState == "TEST_LINE":
                if (isTestLinePlan == False):
                    endX = -50
                    endY = 50
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    latestMsg = "USV 测试-直线路径已规划. 前往 [%d, %d]." % (endX, endY)
                    isTestLinePlan = True       

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):    
                    latestMsg = "USV 测试-直线结束."
                    break

                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(3.5, 20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)
            
            elif usvState == "TEST_CIRCLE":
                if (isTestCirclePlan == False):
                    R = 15
                    circleTimes = 3
                    cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
                    cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
                    currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
                    usvGuidance.setPath(currPath)
                    latestMsg = "USV 测试-圆路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes)
                    isTestCirclePlan = True

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):  
                    latestMsg = "USV 测试-圆结束."
                    break
                
                # R = 30m, dist2Next = 15m, uSP = 3m/s
                # R = 15m, dist2Next = 7m, uSP = 2.6m/s
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(2.6, 7, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == "TEST_BOTH": 
                if (isTestLinePlan == False) & (isTestCirclePlan == False):
                    lineLength = 200
                    endX = usvPose.x + lineLength*cos(usvPose.psi);
                    endY = usvPose.y + lineLength*sin(usvPose.psi);
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    latestMsg = "USV 测试-直线+圆的直线段路径已规划. 前往 [%.2f, %.2f]m." % (endX, endY)
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
                    latestMsg = "USV 测试-直线+圆的圆段路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes) 
                    isTestCirclePlan = True
                    theSpeed = 3
                    theDist2Next = 16

                if (usvGuidance.currentIdx >= usvGuidance.endIdx) & (isTestCirclePlan == True) & (isTestLinePlan == True):  
                    latestMsg ="USV 测试-直线+圆结束."
                    break
                
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(theSpeed, theDist2Next, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == "STABLE":
                uSP = 0.0
                psiSP = usvPose.psi + usvPose.tvAngleLidar
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            else:
                # 程序不应该执行到这里
                console.print("\n[red] >>>>>>> USV state: %s invalid. Check code." % (usvState))
                break
        
            # 打印当前状态
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP) 
            console.print(theTable)

            # 写入当前状态到文件
            usvData.saveData(usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP)

            # 发送无人船的东西 
            usvComm.sendUSVState(usvState)

            rosRate.sleep()
            
        except Exception as e:
            console.print("\n")
            console.print_exception(show_locals=True)
            console.print("[red]>>>>>>> Unexpected exception caught. Check code.")
            break
    
    # 程序不应该执行到这里
    console.print("[red]>>>>>>> USV program jumped out from the main loop.")
    return

if __name__ == '__main__':
    main()
