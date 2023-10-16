#!/usr/bin/env python3
import rospy, threading
import message_filters

from numpy import arctan2, rad2deg, arctan, sqrt, deg2rad, zeros, argmax, argmin, array, abs, tan, arctan
from numpy.linalg import norm
from usv_math import rotationZ, wrapToPi

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.msg import State

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
    psi = float("nan")
    beta = float("nan")
    r = float("nan")
    psiOffset = deg2rad(0)

    state = State()
    
    isGPSValid = False
    isImuValid = False
    isDvlValid = False
    isPodValid = False
    isLidarValid = False

    # Dvl-A125 状态量
    xDVL = 0
    yDVL = 0
    uDVL = 0
    vDVL = 0
    betaDVL = 0

    # Lidar 状态量
    isLidarFindTV = False
    objectNum = 0
    tLidar = 0
    tvX = 0
    tvY = 0
    tvAngleLidar = 0
    tvDist = 0
    tvHeading = 0
    xLidar = 0
    yLidar = 0

    isLidarFindObs = False
    obsX = 0
    obsY = 0
    obsDist = 0
    obsAngleLidar = 0
    obsDistTol = 50.0
    obsAngleTol = deg2rad(15.0)

    # Pod 变量
    isPodFindTV = False
    tvAnglePod = 0
    podTimer = 0
    podTolSec = 0.01

    # 容忍误差
    angleTol = deg2rad(15.0)
    distTol = 5.0
    
    def __init__(self):
        # For PX4 MAVROS local position and velocity (Velocity is in USV body frame)
        self.px4OdomSub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.poseCallback, queue_size=1) 

        # For PX4 MAVROS IMU
        self.px4IMUSub = rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback, queue_size=1) 

        # For DVL
        self.dvlPosSub = rospy.Subscriber('/usv/dvl/position', PoseStamped, self.dvlPosCallback)
        self.dvlVelSub = rospy.Subscriber('/usv/dvl/velocity', Vector3Stamped, self.dvlCallback)

        # For Pod
        self.podSub = rospy.Subscriber('/usv/pod_servo_ctrl/data', Float64MultiArray, self.podCallback, queue_size=1)

        # For Lidar
        self.lidarSub = rospy.Subscriber('/filter/target', PoseArray, self.lidarCallback, queue_size=1)

        # For MAVROS state
        self.stateSub = rospy.Subscriber("mavros/state", State, self.stateCallback)

        
    def __del__(self):
        self.bus.shutdown()
    
    def stateCallback(self, stateMsg):
        self.state = stateMsg

    def poseCallback(self, odomMsg):
        self.x = odomMsg.pose.pose.position.x
        self.y = odomMsg.pose.pose.position.y
        self.u = odomMsg.twist.twist.linear.x
        self.v = odomMsg.twist.twist.linear.y

        [self.vx, self.vy]  = rotationZ(self.u, self.v, -self.psi)

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

        self.psi = psiRaw + self.psiOffset
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
            self.tvAnglePod = msg.data[2]
        else:
            self.isPodFindTV = False
            self.tvAnglePod = float("nan")

        self.isPodValid = True

    def lidarCallback(self, msg):   
        self.objectNum = len(msg.poses)

        # 确保：self.objectNum > 0 的 for 循环一定会执行
        if (self.objectNum <= 0):
            return

        # 记录激光雷达扫描到物体的 x y（船体系下）、船-物体方位角和距离
        objectX = zeros([self.objectNum, 1])
        objectY = zeros([self.objectNum, 1])
        objectAngle = zeros([self.objectNum, 1])
        objectDist = zeros([self.objectNum, 1])
        objectHeading = zeros([self.objectNum, 1])
        for i in range(self.objectNum):
            objectX[i, 0] = msg.poses[i].position.x
            objectY[i, 0] = msg.poses[i].position.y
            objectAngle[i, 0] = arctan2(objectY[i, 0], objectX[i, 0])
            objectDist[i, 0] = sqrt(objectX[i, 0]**2 + objectY[i, 0]**2)
            [_, _, objectHeading[i, 0]] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])

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
            [vx, vy] = rotationZ(self.uDVL, self.vDVL, -self.psi)
            xLidarEst = self.xLidar + dt.to_sec() * vx
            yLidarEst = self.yLidar + dt.to_sec() * vy

            # 根据这一时刻的所有物体位置，计算无人船相对于以物体为原点的 ENU 的坐标
            [xLidarPossible, yLidarPossible] = rotationZ(objectX, objectY, -self.psi)
            xLidarPossible = -xLidarPossible
            yLidarPossible = -yLidarPossible

            # 判断：如果预测位置与真实位置接近，则认为此物体是目标船
            dDist = sqrt((xLidarEst - xLidarPossible) ** 2 + (yLidarEst - yLidarPossible) ** 2)
            dDistIndex = argmin(dDist)
            if (dDist[dDistIndex, 0] < self.distTol):  
                # 记录目标船信息
                self.tvX = objectX[dDistIndex, 0]
                self.tvY = objectY[dDistIndex, 0]
                self.tvAngleLidar = objectAngle[dDistIndex, 0]
                self.tvDist = objectDist[dDistIndex, 0]
                self.tvHeading = arctan(tan(wrapToPi(objectHeading[dDistIndex, 0] + self.psi)))
                if abs(self.tvHeading) > deg2rad(85):
                    self.tvHeading = deg2rad(90)
                  
                # 记录无人船坐标
                self.xLidar = xLidarPossible[dDistIndex, 0]
                self.yLidar = yLidarPossible[dDistIndex, 0]     
                
                # 成功才记录此刻的时间戳
                self.tLidar = msg.header.stamp   

                # 将找到目标船标志位置为真
                self.isLidarFindTV = True    

            else:
                rospy.logwarn("Pod and lidar lose detection for %.2fs", dt.to_sec())

        elif (self.isPodFindTV):
            # 如果此时吊舱扫描到目标船，则比对吊舱偏航角和船-物体方位角
            # 若在角度容许范围内，则认为此物体是目标船，否则，进入激光雷达位置预测判断
            self.tLidar = msg.header.stamp
            dAnglePodLidar = abs(objectAngle - self.tvAnglePod)
            tvIndex = argmin(dAnglePodLidar)
            if (dAnglePodLidar[tvIndex, 0] <= self.angleTol):
                # 记录目标船信息
                self.tvX = objectX[tvIndex, 0]
                self.tvY = objectY[tvIndex, 0]
                self.tvAngleLidar = objectAngle[tvIndex, 0]
                self.tvDist = objectDist[tvIndex, 0]   
                self.tvHeading = arctan(tan(wrapToPi(objectHeading[tvIndex, 0] + self.psi))) 
                if abs(self.tvHeading) > deg2rad(85):
                    self.tvHeading = deg2rad(90)

                # 根据目标船坐标计算无人船坐标
                [self.xLidar, self.yLidar] = rotationZ(self.tvX, self.tvY, -self.psi)
                self.xLidar = -self.xLidar
                self.yLidar = -self.yLidar       

                # 将找到目标船标志位置为真
                self.isLidarFindTV = True    
        
        else:
            # 此时 isPodFindTV 和 isLidarFindTV 均 false
            # 判断激光雷达扫描到的物体是否为障碍物
            # 没有扫描到目标船才进行障碍物判断
            distMinIdx = argmin(objectDist)

            # 使用距离 obsDistTol 和角度 obsAngleTol 判断最近的那个 object 是否为障碍物
            if (objectDist[distMinIdx, 0] < self.obsDistTol) & (abs(objectAngle[distMinIdx, 0]) < self.obsAngleTol):
                self.obsX = objectX[distMinIdx, 0]
                self.obsY = objectY[distMinIdx, 0]
                self.obsDist = objectDist[distMinIdx, 0]
                self.obsAngleLidar = objectAngle[distMinIdx, 0]
                self.isLidarFindObs = True
            else:
                self.isLidarFindObs = False


if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_pos_test_node')
    rosRate = rospy.Rate(2)

    usvPose = Pose()

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            print("USV: [%.2f, %.2f]m | [%.2f, %.2f]m/s | [%.2f, %.2f]m/s^2 | %.2fdeg" % (usvPose.x, usvPose.y, usvPose.u, usvPose.v, usvPose.axb, usvPose.ayb, rad2deg(usvPose.psi)))

            if (usvPose.isPodFindTV):
                print("吊舱扫描到目标船 %.2f deg" % rad2deg(usvPose.tvAnglePod))
            else:
                print("吊舱未扫描到目标船")
            
            if (usvPose.isLidarFindTV):
                print("激光雷达扫描到目标船 [%.2f, %.2f]m，方位 %.2f deg " % (usvPose.tvX, usvPose.tvY, rad2deg(arctan2(usvPose.tvY, usvPose.tvX))))
                print("无人船的位置 [%.2f, %.2f]m" % (usvPose.xLidar, usvPose.yLidar))
            elif (usvPose.objectNum > 0):
                print("激光雷达扫描到 %d 个物体，但不认为它们是目标船" % usvPose.objectNum)
            else:
                print("激光雷达未扫描到目标船")

            rosRate.sleep()
        except KeyboardInterrupt:
            break
