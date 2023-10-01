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

# 无人船状态定义
STARTUP = 0
STANDBY = 1
STABLE = 2
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

# 测试状态定义
isTestEnable = False
TEST_LINE = 100
TEST_CIRCLE = 101
TEST_BOTH = 102

TEST_VEC_STABLE = 103
TEST_VECT_CIRCLE = 104
TEST_VECT_BOTH = 105

TEST_MODE = TEST_VEC_STABLE

# ROS 定频
ROSRATE = 10

# 控制台输出字符串dingyi
endStr = " ... "

@atexit.register 
def clean():
    rospy.loginfo("程序退出...")
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
 
    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    
    # 初始化标志位
    isWaitSearch = False
    isPursuePlan = False
    isAckPodFindTV = False
    isAckLidarFindTV = False
    isAckSendtUAVTakeOffFlag = False
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

    # 
    usvStationTimes = 0
      
    # 试一下
    try:
        while not rospy.is_shutdown():
            if usvState == STARTUP:
                if (usvPose.isValid):
                    print("收到 USV 位置 !")
                    usvState = STANDBY
                else:
                    print("\r等待 USV 位置", end = endStr)

            elif usvState == STANDBY:
                if (isTestEnable):
                    print("测试模式启动，不再接收目标船的估计位置.")
                    usvState = TEST_MODE

                if (usvComm.isSearchFindTV):
                    print("收到航向 %.2f deg." % rad2deg(usvComm.tvAngleEst))
                    usvState = PURSUE
                else:
                    print("\r等待前往目标船的航向...", end = endStr)
                    
            elif usvState == PURSUE:
                print("\rUSV 状态：追踪段 | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")

                # 如果吊舱找到目标船，则进入 DOCK
                if (usvPose.isPodFindTV):
                    print("\n吊舱探测到目标船，估计方位角: %.2f (deg)." % rad2deg(usvPose.tvAnglePod))
                    usvState = DOCK_APPROACH
                    continue 

                # 如果没有找到目标船，则继续跟随追踪路径
                uSP = 3.25
                usvControl.moveUSV(uSP, usvComm.tvAngleEst, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == PURSUE_DETECT_OBS:
                # 读取激光雷达信息 
                pass
                
                # 判断是否还需要避障
                if (True):       
                    pass
                else:
                    print("USV 避障完成，恢复追踪目标船.")
                    isPursuePlan = False
                    
                    usvState = PURSUE
            
            elif usvState == DOCK_APPROACH:
                podOutput = rad2deg(usvPose.tvAnglePod) * usvPose.isPodFindTV + (1 - usvPose.isPodFindTV)
                lidarOutPut = usvPose.tvDist * usvPose.isLidarFindTV + (1 - usvPose.isLidarFindTV)
                print("\rUSV 状态：泊近-接近段 (%.2f deg, %.2f m) | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (podOutput, lidarOutPut, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")

                # 根据吊舱、激光雷达状态，生成控制指令
                if (usvPose.isLidarFindTV):
                    if (usvPose.tvDist < 20.0):
                        uSP = 0.1
                    elif (usvPose.tvDist > 100.0):
                        uSP = 3.0
                    else:
                        uSP = 0.1 + (3.0 - 0.1) * (usvPose.tvDist - 20.0) / (100.0 - 20.0)

                    psiSP = usvPose.psi + usvPose.tvAngleLidar
                elif (usvPose.isPodFindTV):
                    uSP = 3
                    psiSP = usvPose.psi + usvPose.tvAnglePod
                else:
                    print("\n吊舱丢失目标船，恢复追踪段.")
                    usvState = PURSUE
                    isDockApproachPlan = False
                    isPursuePlan = False
                    isAckPodFindTV = False
                    isAckLidarFindTV = False
                    continue
                
                # 控制无人船
                usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 如果接近段结束了，则 DOCK_FINAL（10月7、8、9日），或 DOCK_MEASURE（真正比赛）
                if (usvPose.isLidarFindTV) & (usvPose.tvDist < 15.0):
                    print("\n接近完成")
                    usvState = DOCK_FINAL
                    # usvState = DOCK_MEASURE
                    continue

            elif usvState == DOCK_MEASURE:
                print("\rUSV 状态：泊近-测量段 | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")
                
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
                    print("\n测量完成，测量的目标船姿态为: %.4f (rad)，%.4f (deg)." % (tvAngle, rad2deg(tvAngle)))
                    usvState = DOCK_TRANSFER

            elif usvState == DOCK_TRANSFER:
                print("\rUSV 状态：泊近-变轨段 | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")
                
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
                    print("\n变轨完成")
                    usvState = DOCK_ADJUST
             
            elif usvState == DOCK_ADJUST:
                # 在 ADJUST 段，读取大物体方位角，在大物体侧停下来
                print("USV 状态：泊近-调整段.")
                usvState = DOCK_FINAL

            elif usvState == DOCK_FINAL:
                print("\rUSV 状态：泊近-最终段 | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")
                # 等待船接近静止再发送起飞指令
                if (usvStationTimes > 5 * ROSRATE):
                    print("\n稳定完成，发送 tUAV 起飞指令.")
                    usvComm.sendTakeOffFlag()
                    usvComm.sendTVPosFromLidar()
                    usvComm.sendTakeOffFlag()
                    usvComm.sendTVPosFromLidar()
                    usvComm.sendTakeOffFlag()
                    usvComm.sendTVPosFromLidar()
                    usvState = STABLE
                    continue
                elif (abs(usvPose.uDVL) <= 0.2):
                    usvStationTimes = usvStationTimes + 1 
                else:
                    usvStationTimes = 0

                # 保持静止
                psiSP = usvPose.psi + usvPose.tvAngleLidar
                usvControl.moveUSV(0, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            # 测试部分
            elif usvState == TEST_LINE:
                if (isTestLinePlan == False):
                    endX = -50
                    endY = 50
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    print("USV 测试-直线路径已规划. 前往 [%d, %d]." % (endX, endY))
                    isTestLinePlan = True       

                print("\rUSV 状态：测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):    
                    print("\nUSV 测试-直线结束.")
                    break

                [uSP, psiSP] = usvGuidance.guidance(3.5, 20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)
            
            elif usvState == TEST_CIRCLE:
                if (isTestCirclePlan == False):
                    R = 15
                    circleTimes = 3
                    cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
                    cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
                    currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
                    usvGuidance.setPath(currPath)
                    print("USV 测试-圆路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes))
                    isTestCirclePlan = True

                print("\rUSV 状态：测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")
                
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):  
                    print("\nUSV 测试-圆结束.")  
                    break
                
                # R = 30m, dist2Next = 15m, uSP = 3m/s
                # R = 15m, dist2Next = 7m, uSP = 2.6m/s
                [uSP, psiSP] = usvGuidance.guidance(2.6, 7, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == TEST_BOTH: 
                if (isTestLinePlan == False) & (isTestCirclePlan == False):
                    lineLength = 200
                    endX = usvPose.x + lineLength*cos(usvPose.psi);
                    endY = usvPose.y + lineLength*sin(usvPose.psi);
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
                    usvGuidance.setPath(currPath)
                    print("USV 测试-直线+圆的直线段路径已规划. 前往 [%.2f, %.2f]m." % (endX, endY))
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
                    print("\nUSV 测试-直线+圆的圆段路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes))    
                    isTestCirclePlan = True
                    theSpeed = 3
                    theDist2Next = 16

                print("\rUSV 状态：测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")

                if (usvGuidance.currentIdx >= usvGuidance.endIdx) & (isTestCirclePlan == True) & (isTestLinePlan == True):  
                    print("\nUSV 测试-直线+圆结束.")  
                    break
                
                [uSP, psiSP] = usvGuidance.guidance(theSpeed, theDist2Next, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
                usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

            # 矢量推力 测试部分
            elif usvState == TEST_VEC_STABLE:
                print("\rUSV 状态：矢量推力-自稳测试 | [x, y] = [%.2f, %.2f] m | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")
                usvControl.moveUSV(3, deg2rad(-20), usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            elif usvState == STABLE:
                print("\rUSV 状态：自稳 | [u, v] = [%.2f, %.2f] m/s | psi = %.2f deg | r = %.2f deg/s" % (usvPose.uDVL, usvPose.vDVL, rad2deg(usvPose.psi), rad2deg(usvPose.r)), end = "")
                psiSP = usvPose.psi + usvPose.tvAngleLidar
                usvControl.moveUSV(0, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

            else:
                # 程序不应该执行到这里
                print("变量 [usvState] 取到异常值 %d，请检查程序." % (usvState))
                break
            
            rosRate.sleep()

        # 程序不应该执行到这里
        print("程序跳出主循环. usvState = %d, rospy_is_shutdown() = %d." % (usvState, rospy.is_shutdown()))

    except KeyboardInterrupt:
        print("检测到 Ctrl + C，退出 ...")

    except Exception as e:
        print("程序异常，请检查.")
        traceback.print_exc()
         
    return

if __name__ == '__main__':
    main()
