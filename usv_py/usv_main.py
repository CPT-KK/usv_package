import rclpy
import threading

from numpy import zeros, rad2deg, median
from numpy.linalg import norm

from usv_lidar import Lidar
from usv_pose import Pose
from usv_path_planner import PathPlanner
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

def main(args=None):
    rclpy.init(args=args)
    
    # 添加主节点
    mainNode = rclpy.create_node('usv_main_node')

    # 定频
    rate = 5
    rosRate = mainNode.create_rate(rate)
    
    # 添加功能节点
    usvPose = Pose()
    usvLidar = Lidar()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control(rate)
    
    # 设置节点并行 spin
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mainNode)
    executor.add_node(usvPose)
    executor.add_node(usvLidar)
    executor.add_node(usvComm)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # 初始化标志位
    isPursuePlan = False
    isDockApproachPlan = False
    isDockMeasurePlan = False
    isDockTransferPlan = False

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
        while rclpy.ok():
            if usvState == STARTUP:
                if (usvPose.isValid):
                    mainNode.get_logger().info("收到 USV 位置.")
                    usvState = STANDBY
                else:
                    mainNode.get_logger().info("等待传来 USV 位置.")

            elif usvState == STANDBY:
                if (usvComm.isTVEst):
                    mainNode.get_logger().info("收到目标船的估计位置.")
                    usvState = PURSUE
                else:
                    mainNode.get_logger().info("等待目标船的估计位置.")
                    
            elif usvState == PURSUE:
                # 如果需要规划追踪段路径，则规划并发送给制导
                if (isPursuePlan == False):
                    currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, usvComm.tvEstPosX, usvComm.tvEstPosY)
                    usvGuidance.setPath(currPath)
                    mainNode.get_logger().info("USV 追踪段已规划.")
                    mainNode.get_logger().info("当前状态：追踪段.")
                    isPursuePlan = True

                # 如果 USV 当前位置距离当前跟踪点太远，则重新规划（以防万一）
                if (norm([usvPose.x - usvGuidance.path[usvGuidance.currentIdx, 0], usvPose.y - usvGuidance.path[usvGuidance.currentIdx, 1]]) >= 40):
                    mainNode.get_logger().info("USV 疑似脱离路径？立即重新规划追踪段.")
                    isPursuePlan = False
                    continue

                # 如果跑完了所有点都没有找到目标船，则重新规划（以防万一）
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    mainNode.get_logger().info("跑完了追踪段但激光雷达仍没有发现目标船？立即重新规划追踪段.")
                    isPursuePlan = False
                    continue
     
                # 读取激光雷达信息
                [idxTV, isTVFound, _, isObsFound] = usvLidar.objRead(usvPose.x, usvPose.y, usvPose.psi, usvComm.tvEstPosX, usvComm.tvEstPosY)
                
                # 如果找到目标船，则进入 DOCK_PREPARE
                if (isTVFound):
                    [tvX, tvY] = rotationZ(usvLidar.objInfo[idxTV, 0], usvLidar.objInfo[idxTV, 1], -usvPose.psi)
                    tvX = tvX + usvPose.x
                    tvY = tvY + usvPose.y
                    mainNode.get_logger().info("激光雷达探测到目标船.")
                    mainNode.get_logger().info("估计目标船在: [%.2f, %.2f]（USV 船体系下），方向角: %.4f (rad) 或 %.4f (deg)." % (usvLidar.objInfo[idxTV, 0], usvLidar.objInfo[idxTV, 1], usvLidar.objInfo[idxTV, 4], rad2deg(usvLidar.objInfo[idxTV, 4])))
                    usvState = DOCK_APPROACH
                    continue

                # 如果检测到障碍物，则进入 PURSUE_DETECT_OBS
                if (isObsFound):
                    mainNode.get_logger().info("检测到障碍物，执行避障.")
                    usvState = PURSUE_DETECT_OBS
                    continue

                # 如果没有找到目标船，则继续跟随追踪路径
                
                [uSP, psiSP] = usvGuidance.guidance(3.0, 25.0, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.psi)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            # elif usvState == PURSUE_DETECT_OBS:

            # elif usvState == PURSUE_EMERGENCY:
            
            elif usvState == DOCK_APPROACH:
                if (isDockApproachPlan == False):
                    currPath = usvPathPlanner.planDockApproach(usvPose.x, usvPose.y, tvX, tvY)
                    usvGuidance.setPath(currPath)
                    mainNode.get_logger().info("USV 泊近-接近段路径已规划.")
                    mainNode.get_logger().info("USV 状态：泊近-接近段.")
                    isDockApproachPlan = True
                        
                [uSP, psiSP] = usvGuidance.guidance(2.0, 20.0, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.psi)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 如果接近段结束了，则进入测量段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = DOCK_MEASURE
                    continue

            elif usvState == DOCK_MEASURE:
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.x, usvPose.y, tvX, tvY)
                    usvGuidance.setPath(currPath)
                    mainNode.get_logger().info("USV 泊近-测量段路径已规划.")
                    mainNode.get_logger().info("USV 状态：泊近-测量段.")
                    isDockMeasurePlan = True
                    mainNode.get_logger().info("开始测量目标船姿态.")
       
                [uSP, psiSP] = usvGuidance.guidance(2.0, 20.0, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.psi)
                usvControl.moveUSV(uSP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？）
                [idxTV, isTVFound, _, _] = usvLidar.objRead(usvPose.x, usvPose.y, usvPose.psi, usvComm.tvEstPosX, usvComm.tvEstPosY)

                # 读取目标船信息
                tvRecordNum = tvRecordNum + 1
                tvRecordAngle[tvRecordNum] = usvLidar.objInfo[idxTV, 4] + usvPose.psi

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    [tvX, tvY] = rotationZ(usvLidar.objInfo[idxTV, 0], usvLidar.objInfo[idxTV, 1], -usvPose.psi)
                    tvX = tvX + usvPose.x
                    tvY = tvY + usvPose.y
                    tvAngle = median(tvRecordAngle[0:tvRecordNum])
                    mainNode.get_logger().info("测量的目标船姿态为: %.4f (rad)，%.4f (deg)." % (tvAngle, rad2deg(tvAngle)))
                    mainNode.get_logger().info("结束测量目标船姿态.")
                    usvState = DOCK_TRANSFER

            elif usvState == DOCK_TRANSFER:
                if (isDockTransferPlan == False):
                    currPath = usvPathPlanner.planDockTransfer(usvPose.x, usvPose.y, tvX, tvY, tvAngle)
                    usvGuidance.setPath(currPath)
                    isDockTransferPlan = True
                    mainNode.get_logger().info("USV 泊近-变轨段路径已规划.")
                    mainNode.get_logger().info("USV 状态：泊近-变轨段.")
           
                [xSP, ySP, psiSP] = usvGuidance.guidanceVec(12.0, 2.0)
                usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)
             
            elif usvState == DOCK_ADJUST:
                # 在 ADJUST 段，读取大物体方位角，在大物体侧停下来
                mainNode.get_logger().info("USV 状态：泊近-调整段.")

                # 调整段结束，进入最终段
                usvState = DOCK_FINAL

            elif usvState == DOCK_FINAL:
                mainNode.get_logger().info("USV 状态：泊近-最终段.")
                return

            # elif usvState == DOCK_EMERGENCY:

            # elif usvState == STOP:

            # elif usvState == ATTACH:

            else:
                print("%.2fs, [%.2f, %.2f]m, [%.2f, %.2f]m/s, [%.2f, %.2f]m/s^{-2}, %.2frad, %.2frad/s" % (usvPose.t, usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r))
                print("%.2fs, %d, " % (usvLidar.t, usvLidar.objNum))
            
            rosRate.sleep()

    except KeyboardInterrupt:
        pass   

    except Exception as e:
        raise(e)
        # pass


if __name__ == '__main__':
    main()
