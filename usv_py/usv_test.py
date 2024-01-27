#!/usr/bin/env python3

import rospy
import threading
import atexit
import signal
from rich.console import Console
from numpy import zeros, rad2deg, deg2rad, pi, abs, mean, sin, cos, tan, arctan, arctan2, std, sqrt, isnan

from usv_pose import Pose
from usv_path_planner import PathPlanner
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import removeOutliers, wrapToPi, linearClip, rotationZ
from usv_record import genTable, USVData

from rosgraph_msgs.msg import Clock
from usv_constants import *


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
    # usvCAN = USVCAN()
    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control()
    usvData = USVData()
    console.print("[green]>>>>>>> Function classes initialized.")

    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    console.print("[green]>>>>>>> ROS spin started.")

    #
    rospy.wait_for_message("/clock", Clock)

    usvPose.isSearchFindTV = True
    usvPose.isSearchPointRealTV = True
    usvPose.tvEstPosX = 0
    usvPose.tvEstPosY = -3
    usvPose.tvAngleEst = -0.25*pi

    # usvPose.isPodFindTV = True
    # usvPose.tvAnglePod = -0.25*pi

    usvPose.uDVL = 0
    usvPose.vDVL = 0
    usvPose.yaw = deg2rad(0)

    while (not usvPose.isLidarFindTV):
        rosRate.sleep()

    usvPose.isSearchFindTV = False
    usvPose.isPodFindTV = False

    # 保存目标船朝向角的数组
    tvInfo = zeros((3, 5000))
    tvInfoIdx = 0
    tvHighestXYZs = zeros((3, 5000))
    tvHighestInfoIdx = 0

    # 设置计时器
    timer1 = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - timer1 < 10):
        tvInfo[0, tvInfoIdx] = usvPose.tvHeading
        tvInfo[1, tvInfoIdx] = usvPose.tvLength
        tvInfo[2, tvInfoIdx] = usvPose.tvWidth
        tvInfoIdx = tvInfoIdx + 1
        latestMsg = "Estimating target vessel heading: %.2f deg. L: %.2f m. W: %.2f m." % (rad2deg(usvPose.tvHeading), usvPose.tvLength, usvPose.tvWidth)
        print(latestMsg)

        rosRate.sleep()

    # 去除后面未使用的索引
    tvInfo = tvInfo[:, 0:tvInfoIdx-1]
    tvHeadings = tvInfo[0, :]
    tvLengths = tvInfo[1, :]
    tvWidths = tvInfo[2, :]

    # 如果标准差大于 ANGLE_DOCK_MEASURE_JUMP，认为大概率是-90°与90°跳变的情况，因此对 tvHeadings 中负角度加一个 pi
    stdHeading = std(tvHeadings)
    if (stdHeading > ANGLE_DOCK_MEASURE_JUMP):
        tvHeadings[tvHeadings < 0] = tvHeadings[tvHeadings < 0] + pi

    # 去除离群点
    tvHeadings = removeOutliers(tvHeadings, 0.087266, 20)
    tvLengths = removeOutliers(tvLengths, 0.5, 20)
    tvWidths = removeOutliers(tvWidths, 0.5, 20)

    # 计算平均值，并将结果角度映射到-90°~90°
    tvHeadingMean = arctan(tan(mean(tvHeadings)))
    tvLengthMean = mean(tvLengths)
    tvWidthMean = mean(tvWidths)

    latestMsg = "Estimating finished with average heading %.2f deg. L: %.2f m. W: %.2f m. Begin final approach..." % (rad2deg(tvHeadingMean), tvLengthMean, tvWidthMean)
    print(latestMsg)

    finalPsi = tvHeadingMean

    # 将当前时间写入 t1 计时器
    timer1 = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - timer1 < SECS_WAIT_HEIGHT_SEARCH):
        latestMsg = "USV has been stablized. Measuring the highest point... [%.2f / %.2f]s." % (rospy.Time.now().to_sec() - timer1, SECS_WAIT_HEIGHT_SEARCH)
        print(latestMsg)

        tvHighestXYZs[0, tvHighestInfoIdx] = usvPose.tvHighestX
        tvHighestXYZs[1, tvHighestInfoIdx] = usvPose.tvHighestY
        tvHighestXYZs[2, tvHighestInfoIdx] = usvPose.tvHighestZ
        tvHighestInfoIdx = tvHighestInfoIdx + 1
        rosRate.sleep()

    # 等待测量完成
    tvHighestXYZs = tvHighestXYZs[:, 0:tvHighestInfoIdx-1]
    tvHighestXs = removeOutliers(tvHighestXYZs[0, :], 0.1, 20)
    tvHighestYs = removeOutliers(tvHighestXYZs[1, :], 0.1, 20)
    tvHighestZs = removeOutliers(tvHighestXYZs[2, :], 0.1, 20)
    tvHighestXMean = mean(tvHighestXs)
    tvHighestYMean = mean(tvHighestYs)
    tvHighestZMean = mean(tvHighestZs)
    print(f"tvHighestXMean: {tvHighestXMean}. tvHighestYMean: {tvHighestYMean}. tvHighestZMean: {tvHighestZMean}.")
    if (tvHighestZMean >= HEALTHY_Z_TOL):   
        # 最高点测量健康，向最高点映射到船中轴线上的点泊近
        # 注意：这里的 rotationZ 是要对点（向量）进行旋转，即求取点在旋转后的坐标（同一坐标系下），
        # 而不是同一个点在不同坐标系下的表示，故取负号
        [tvHighestXMean2, _] = rotationZ(tvHighestXMean, tvHighestYMean, finalPsi)
        if (tvHighestXMean2 >= 0):
            finalX = (-0.5 * tvLengthMean + tvHighestXMean2) / 2
            finalY = 0.0
        else:
            finalX = (0.5 * tvLengthMean + tvHighestXMean2) / 2
            finalY = 0.0
        [finalX, finalY] = rotationZ(finalX, finalY, -finalPsi)
    else: 
        # 最高点测量不健康，向目标船中心泊近
        finalX = 0.0
        finalY = 0.0
    print(f"Final X: {finalX}. Final Y: {finalY}.")

if __name__ == '__main__':
    main()