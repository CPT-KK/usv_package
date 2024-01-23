#!/usr/bin/env python3
import rospy, threading, struct
import message_filters

from numpy import arctan2, rad2deg, arctan, sqrt, deg2rad, zeros, argmax, argmin, array, abs, tan, arctan
from numpy.linalg import norm
from usv_math import rotationZ, wrapToPi

from std_msgs.msg import Float64MultiArray, Int8
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Pose():
    # USV 状态量
    x = float("nan")
    y = float("nan")
    vx = float("nan")
    vy = float("nan")
    u = float("nan")
    v = float("nan")
    axb = float("nan")
    ayb = float("nan")
    azb = float("nan")

    roll = float("nan")
    pitch = float("nan")
    yaw = float("nan")
    beta = float("nan")
    r = float("nan")
    yawOffset = deg2rad(0)

    isGPSValid = False
    isImuValid = False

    # Dvl-A125 状态量
    isDvlValid = False
    xDVL = float("nan")
    yDVL = float("nan")
    uDVL = float("nan")
    vDVL = float("nan")
    betaDVL = float("nan")

    # Lidar 量
    isLidarValid = False
    isLidarFindTV = False
    isLidarFindTVPrevious = False
    objectNum = float("nan")
    tLidar = float("nan")
    tvX = float("nan")             # 目标船在 ENU 系（原点为无人船）下的 x 坐标
    tvY = float("nan")             # 目标船在 ENU 系（原点为无人船）下的 y 坐标
    tvAngleLidar = float("nan")    # 目标船在 ENU 系（原点为无人船）下的方向角
    tvDist = float("nan")          # 目标船距离无人船的距离
    tvHeading = float("nan")       # 目标船在 ENU 系下的朝向
    tvLength = float("nan")        # 目标船长度
    tvWidth = float("nan")         # 目标船宽度
    tvHighestX = float("nan")      # 目标船最高点 X 坐标（原点为目标船）
    tvHighestY = float("nan")      # 目标船最高点 Y 坐标（原点为目标船）
    tvHighestZ = float("nan")      # 目标船最高点 Z 坐标（原点为目标船）
    xLidar = float("nan")          # 无人船在 ENU 系（原点为目标船）下的 x 坐标
    yLidar = float("nan")          # 无人船在 ENU 系（原点为目标船）下的 y 坐标

    isLidarFindObs = False
    obsX = float("nan")
    obsY = float("nan")
    obsDist = float("nan")
    obsAngleLidar = float("nan")
    obsDistTol = 70.0
    obsAngleTol = deg2rad(15.0)

    # Pod 量  
    isPodValid = False
    isPodFindTV = False
    tvAnglePod = float("nan")  # ENU 系下的吊舱角
    podState = float("nan")
    isPodResetting = False

    # sUAV 量
    isSearchFindTV = False
    isSearchPointRealTV = False # 搜索无人机会给多个点，用此标志位判断给的点是否为真的目标船
    tvEstPosX = float("nan")    # 相对于以无人船为原心的 ENU 的 X
    tvEstPosY = float("nan")    # 相对于以无人船为原心的 ENU 的 Y
    tvAngleEst = deg2rad(-150)

    # isSearchFindUSV = False
    # usvEstPosX = float("nan")
    # usvEstPosY = float("nan")

    # 容忍误差
    anglePodLidarTol = deg2rad(15.0)
    distLidarIntTol = 5.0
    distSearchLidarStrictTol = 150.0
    distSearchLidarSoftTol = 180.0
    
    def __init__(self):
        # For PX4 MAVROS local position and velocity (Velocity is in USV body frame)
        self.px4OdomSub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.poseCallback, queue_size=1) 

        # For USV IMU
        self.px4IMUSub = rospy.Subscriber('/usv/imu/data', Imu, self.imuCallback, queue_size=1) 

        # For DVL
        self.dvlPosSub = rospy.Subscriber('/usv/dvl/position', PoseStamped, self.dvlPosCallback)
        self.dvlVelSub = rospy.Subscriber('/usv/dvl/velocity', Vector3Stamped, self.dvlCallback)

        # For Pod
        self.podResetPub = rospy.Publisher('/usv/pod/servo_cmd', Int8, queue_size=1)
        self.podSub = rospy.Subscriber('/usv/pod/pod_servo_ctrl/data', Float64MultiArray, self.podCallback, queue_size=1)

        # For Lidar
        self.lidarSub = rospy.Subscriber('/filter/target', PoseArray, self.lidarCallback, queue_size=1)

        # For target vessel position from sUAV
        self.tvEstPosSub = rospy.Subscriber('/target_nav_position', PoseStamped, self.tvOdomCallback)

        # For USV position from sUAV
        # self.usvEstPosSub = rospy.Subscriber('/usv_nav_position', PoseStamped, self.usvOdomCallback)
        
    def __del__(self):
        pass

    def poseCallback(self, odomMsg):
        self.x = odomMsg.pose.pose.position.x
        self.y = odomMsg.pose.pose.position.y
        self.u = odomMsg.twist.twist.linear.x
        self.v = odomMsg.twist.twist.linear.y

        [self.vx, self.vy]  = rotationZ(self.u, self.v, -self.yaw)

        if (norm([self.vx, self.vy]) < 0.1):
            self.beta = 0
        else:
            self.beta = arctan2(self.v, self.u)
        
        self.isGPSValid = True
        
    def imuCallback(self, imuMsg):
        self.axb = imuMsg.linear_acceleration.x
        self.ayb = imuMsg.linear_acceleration.y
        self.azb = imuMsg.linear_acceleration.z
        self.r = imuMsg.angular_velocity.z
        [self.roll, self.pitch, psiRaw] = euler_from_quaternion([imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w])

        self.yaw = psiRaw + self.yawOffset
        self.isImuValid = True
    
    def dvlPosCallback(self, dvlMsg):
        self.xDVL = dvlMsg.pose.position.x
        self.yDVL = dvlMsg.pose.position.y
    
    def dvlCallback(self, dvlMsg):
        self.uDVL = dvlMsg.vector.x
        self.vDVL = dvlMsg.vector.y
        if (norm([self.uDVL, self.vDVL]) < 0.1):
            self.betaDVL = 0
        else:
            self.betaDVL = arctan2(self.vDVL, self.uDVL)

        self.isDvlValid = True
   
    def podCallback(self, msg):
        if (msg.data[0] == 1):
            self.isPodFindTV = True
            self.tvAnglePod = self.yaw + msg.data[2]
            self.podState = msg.data[3]
        else:
            self.isPodFindTV = False
            # self.tvAnglePod = float("nan")

        self.isPodValid = True

    def podReset(self):
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))
        self.podResetPub.publish(Int8(data=1))

    def lidarCallback(self, msg):          
        if (len(msg.poses) % 2 == 0):
            self.objectNum = int(len(msg.poses) / 2)
        else:
            return

        # 确保：self.objectNum > 0 的 for 循环一定会执行
        if (self.objectNum <= 0):
            return

        # 记录激光雷达扫描到物体的 x y（船体为圆心的ENU系下）、船-物体方位角和距离
        objectX = zeros([self.objectNum, 1])
        objectY = zeros([self.objectNum, 1])
        objectAngle = zeros([self.objectNum, 1])
        objectDist = zeros([self.objectNum, 1])
        objectLength = zeros([self.objectNum, 1])
        objectWidth = zeros([self.objectNum, 1])
        objectHeading = zeros([self.objectNum, 1])
        objectHighestX = zeros([self.objectNum, 1])
        objectHighestY = zeros([self.objectNum, 1])
        objectHighestZ = zeros([self.objectNum, 1])
        
        for i in range(self.objectNum):
            objectX[i, 0] = msg.poses[2*i].position.x
            objectY[i, 0] = msg.poses[2*i].position.y
            objectAngle[i, 0] = arctan2(objectY[i, 0], objectX[i, 0])
            objectDist[i, 0] = sqrt(objectX[i, 0]**2 + objectY[i, 0]**2)
            [_, _, objectHeading[i, 0]] = euler_from_quaternion([msg.poses[2*i].orientation.x, msg.poses[2*i].orientation.y, msg.poses[2*i].orientation.z, msg.poses[2*i].orientation.w])

            objectHighestX[i, 0] = msg.poses[2*i+1].position.x
            objectHighestY[i, 0] = msg.poses[2*i+1].position.y
            objectHighestZ[i, 0] = msg.poses[2*i+1].position.z
            objectLength[i, 0] = msg.poses[2*i+1].orientation.x
            objectWidth[i, 0] = msg.poses[2*i+1].orientation.y

        # 判断激光雷达扫描到的物体是否为目标船
        if (self.isLidarFindTV):
            # 如果此时吊舱*未*扫描到目标船，则激光雷达必须先前锁定过目标船才能识别目标船
            # 否则，直接认为未识别到目标船，不会进入此 if 段
            # 首先，通过上一时刻目标船位置和 USV 速度计算目标船这一时刻的预测位置
            # 然后，读取激光雷达这一时刻扫描到物体的真实位置
            # 随后，比对真实位置和预测位置的误差
            # 若在位置容许范围内，则识别到目标船；否则，认为未识别到目标船

            # 计算时间差
            dt = msg.header.stamp - self.tLidar

            # 如果时间差太大，则认为无法通过激光雷达上一时刻的值来预测（船不再走直线）
            if (dt.to_sec() > 5.0):
                rospy.logerr("Lidar loses target vessel for 5 seconds! Flag [isLidarFindTV] is set to FALSE.")
                self.isLidarFindTV = False
                return

            # 根据无人船上一时刻的位置坐标 [xLidar, yLidar] 预测这一时刻的坐标 [xLidarEst, yLidarEst]
            [vx, vy] = rotationZ(self.uDVL, self.vDVL, -self.yaw)
            xLidarEst = self.xLidar + dt.to_sec() * vx
            yLidarEst = self.yLidar + dt.to_sec() * vy

            # 根据这一时刻的所有物体位置，计算无人船相对于以物体为原点的 ENU 的坐标
            # [xLidarPossible, yLidarPossible] = rotationZ(objectX, objectY, 0.0)
            xLidarPossible = -objectX
            yLidarPossible = -objectY

            # 判断：如果预测位置与真实位置接近，则认为此物体是目标船
            dDist = sqrt((xLidarEst - xLidarPossible) ** 2 + (yLidarEst - yLidarPossible) ** 2)
            dDistIndex = argmin(dDist)
            if (dDist[dDistIndex, 0] < self.distLidarIntTol):  
                # 记录目标船信息
                self.tvX = objectX[dDistIndex, 0]
                self.tvY = objectY[dDistIndex, 0]
                self.tvAngleLidar = objectAngle[dDistIndex, 0]
                self.tvDist = objectDist[dDistIndex, 0]
                self.tvLength = objectLength[dDistIndex, 0]
                self.tvWidth = objectWidth[dDistIndex, 0]
                self.tvHeading = objectHeading[dDistIndex, 0]
                self.tvHighestX = objectHighestX[dDistIndex, 0] - self.tvX
                self.tvHighestY = objectHighestY[dDistIndex, 0] - self.tvY
                self.tvHighestZ = objectHighestZ[dDistIndex, 0]  
                
                # 记录无人船坐标
                self.xLidar = xLidarPossible[dDistIndex, 0]
                self.yLidar = yLidarPossible[dDistIndex, 0]     
                
                # 成功才记录此刻的时间戳
                self.tLidar = msg.header.stamp

                # 将找到目标船标志位置为真
                self.isLidarFindTV = True  
                self.isLidarFindTVPrevious = True 

            else:
                rospy.logwarn(f"Pod and lidar lose detection for {dt.to_sec():.2f}")

        elif (self.isPodFindTV):
            # 吊舱识别目标船方法
            # 如果此时吊舱扫描到目标船，则比对吊舱偏航角和船-物体方位角
            # 若在角度容许范围内，则认为此物体是目标船，否则，进入激光雷达位置预测判断
            dAnglePodLidar = abs(objectAngle - self.tvAnglePod)
            dxSearchLidar = abs(objectX - self.tvEstPosX)
            dySearchLidar = abs(objectY - self.tvEstPosY)
            distSearchLidar = sqrt(dxSearchLidar ** 2 + dySearchLidar ** 2)
            tvIndex = argmin(dAnglePodLidar)
            if (dAnglePodLidar[tvIndex, 0] <= self.anglePodLidarTol):
                # 记录目标船信息
                self.tvX = objectX[tvIndex, 0]
                self.tvY = objectY[tvIndex, 0]
                self.tvAngleLidar = objectAngle[tvIndex, 0]
                self.tvDist = objectDist[tvIndex, 0]
                self.tvLength = objectLength[tvIndex, 0]
                self.tvWidth = objectWidth[tvIndex, 0]
                self.tvHeading = objectHeading[tvIndex, 0]
                self.tvHighestX = objectHighestX[tvIndex, 0] - self.tvX
                self.tvHighestY = objectHighestY[tvIndex, 0] - self.tvY
                self.tvHighestZ = objectHighestZ[tvIndex, 0]

                # 根据目标船坐标计算无人船坐标
                self.xLidar = -self.tvX
                self.yLidar = -self.tvY

                # 将找到目标船标志位置为真
                self.isLidarFindTV = True

                # 成功才记录此刻的时间戳
                self.tLidar = msg.header.stamp
        
        elif (self.isSearchFindTV) & (self.isSearchPointRealTV):
            # 坐标比对识别目标船方法
            # 如果搜索无人机给了目标船坐标，并且这个坐标是真的目标船（不是虚拟点）
            # 则通过 搜索无人机传入的目标船位置 与 无人船测量的目标船位置 作差
            # 如果这个差值小于容忍误差，则认为此物体是目标船
            dxSearchLidar = abs(objectX - self.tvEstPosX)
            dySearchLidar = abs(objectY - self.tvEstPosY)
            distSearchLidar = sqrt(dxSearchLidar ** 2 + dySearchLidar ** 2)
            tvIndex = argmin(distSearchLidar)     
            if (distSearchLidar[tvIndex, 0] <= self.distSearchLidarStrictTol):
                # 记录目标船信息
                self.tvX = objectX[tvIndex, 0]
                self.tvY = objectY[tvIndex, 0]
                self.tvAngleLidar = objectAngle[tvIndex, 0]
                self.tvDist = objectDist[tvIndex, 0]
                self.tvLength = objectLength[tvIndex, 0]
                self.tvWidth = objectWidth[tvIndex, 0]
                self.tvHeading = objectHeading[tvIndex, 0]
                self.tvHighestX = objectHighestX[tvIndex, 0] - self.tvX
                self.tvHighestY = objectHighestY[tvIndex, 0] - self.tvY
                self.tvHighestZ = objectHighestZ[tvIndex, 0]

                # 根据目标船坐标计算无人船坐标
                self.xLidar = -self.tvX
                self.yLidar = -self.tvY

                # 将找到目标船标志位置为真
                self.isLidarFindTV = True

                # 成功才记录此刻的时间戳
                self.tLidar = msg.header.stamp

        elif (self.isLidarFindTVPrevious) & (not self.isLidarFindTV):
            tvIndex = argmin(objectDist)
            
            self.tvX = objectX[tvIndex, 0]
            self.tvY = objectY[tvIndex, 0]
            self.tvAngleLidar = objectAngle[tvIndex, 0]
            self.tvDist = objectDist[tvIndex, 0]
            self.tvLength = objectLength[tvIndex, 0]
            self.tvWidth = objectWidth[tvIndex, 0]
            self.tvHeading = objectHeading[tvIndex, 0]
            self.tvHighestX = objectHighestX[tvIndex, 0] - self.tvX
            self.tvHighestY = objectHighestY[tvIndex, 0] - self.tvY
            self.tvHighestZ = objectHighestZ[tvIndex, 0]

            # 根据目标船坐标计算无人船坐标
            self.xLidar = -self.tvX
            self.yLidar = -self.tvY

            # 将找到目标船标志位置为真
            self.isLidarFindTV = True

            # 成功才记录此刻的时间戳
            self.tLidar = msg.header.stamp

        else:
            # 此时激光雷达扫描到物体，但是吊舱和坐标比对均不认为物体是目标船
            # 因此，需要判断激光雷达扫描到的物体是否为障碍物
            distMinIdx = argmin(objectDist)

            # 使用距离 obsDistTol 和角度 obsAngleTol 判断最近的那个 object 是否为障碍物                                   
            if (objectDist[distMinIdx, 0] < self.obsDistTol) & (abs(objectAngle[distMinIdx, 0] - self.yaw) < self.obsAngleTol):
                self.obsX = objectX[distMinIdx, 0]
                self.obsY = objectY[distMinIdx, 0]
                self.obsDist = objectDist[distMinIdx, 0]
                self.obsAngleLidar = objectAngle[distMinIdx, 0]
                self.isLidarFindObs = True
            elif (objectDist[distMinIdx, 0] > self.obsDistTol) | (abs(objectAngle[distMinIdx, 0] - self.yaw) > 2.0 * self.obsAngleTol):
                self.isLidarFindObs = False

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.pose.position.x
        self.tvEstPosY = msg.pose.position.y
        self.tvAngleEst = deg2rad(msg.pose.orientation.w)
        self.isSearchFindTV = True
        if (msg.header.frame_id == "target"):
            self.isSearchPointRealTV = True
        else:
            self.isSearchPointRealTV = False


if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_pose_test_node')
    rosRate = rospy.Rate(10)

    usvPose = Pose()

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while not rospy.is_shutdown():
        try:
            print("-----------------------------------------------")
            print(f"Vel [{usvPose.uDVL:.2f}, {usvPose.vDVL:.2f}]m/s | Acc [{usvPose.axb:.2f}, {usvPose.ayb:.2f}]m/s^2 | RPY [{rad2deg(usvPose.roll):.2f}, {rad2deg(usvPose.pitch):.2f}, {rad2deg(usvPose.yaw):.2f}]deg | Yaw rate {rad2deg(usvPose.r):.2f}deg/s")

            if (usvPose.isPodFindTV):
                print(f"吊舱扫描到目标船 {rad2deg(usvPose.tvAnglePod):.2f} deg")
            else:
                print("吊舱未扫描到目标船")
            
            if (usvPose.isLidarFindTV):
                thisTVAngle = arctan(tan(wrapToPi(usvPose.tvHeading + usvPose.yaw)))
                print(f"激光雷达扫描到目标船 [{usvPose.tvX:.2f}, {usvPose.tvY:.2f}]m，距离 {usvPose.tvDist:.2f}m，方位 {rad2deg(usvPose.tvAngleLidar):.2f} deg，朝向 {rad2deg(thisTVAngle):.2f} deg ")
                print(f"无人船的位置 [{usvPose.xLidar:.2f}, {usvPose.yLidar:.2f}]m")

            elif (usvPose.objectNum > 0):
                print(f"激光雷达扫描到 {usvPose.objectNum} 个物体，但不认为它们是目标船")
            else:
                print("激光雷达未扫描到目标船")

            rosRate.sleep()
        except KeyboardInterrupt:
            break
