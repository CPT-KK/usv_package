#!/usr/bin/env python3
import rospy, threading
import message_filters
from numpy import arctan2, rad2deg, arctan, sqrt, deg2rad, zeros, argmax, argmin
from numpy.linalg import norm
from usv_math import rotationZ

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Pose():
    t = 0.0
    x = 0.0
    y = 0.0
    vx = 0.0
    vy = 0.0
    u = 0.0
    v = 0.0
    axb = 0.0
    ayb = 0.0

    psi = 0.0
    beta = 0.0
    r = 0.0
    
    isValid = False

    uDVL = 0
    vDVL = 0
    betaDVL = 0

    isLidarFindTV = False
    objectNum = 0
    tLidarLast = 0
    tLidar = 0
    tvX = 0
    tvY = 0
    tvAngleLidar = 0
    tvDist = 0
    xLidar = 0
    yLidar = 0

    isPodFindTV = False
    tvAnglePod = 0

    angleTol = deg2rad(5.0)
    distTol = 5.0

    def __init__(self):
        # For PX4 MAVROS local position and velocity (Velocity is in USV body frame)
        self.px4OdomSub = message_filters.Subscriber('/mavros/local_position/odom', Odometry) 

        # For PX4 MAVROS IMU
        self.px4IMUSub = message_filters.Subscriber('/mavros/imu/data', Imu) 

        self.ts = message_filters.ApproximateTimeSynchronizer([self.px4OdomSub, self.px4IMUSub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.poseCallback)

        # For doppler
        self.dvlVelSub = rospy.Subscriber('/usv/dvl/velocity', Vector3Stamped, self.dvlCallback)

        # For Pod
        self.podSub = rospy.Subscriber('/usv/pod_servo_ctrl/data', Float64MultiArray, self.podCallback, queue_size=1)

        # For LIDAR
        self.lidarSub = rospy.Subscriber('/filter/target', PoseArray, self.lidarCallback, queue_size=1)

    def poseCallback(self, odom, imu):
        self.t = 0.5 * (odom.header.stamp.secs + 1e-9 * odom.header.stamp.nsecs + imu.header.stamp.secs + 1e-9 * imu.header.stamp.nsecs)
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.u = odom.twist.twist.linear.x
        self.v = odom.twist.twist.linear.y
        self.axb = imu.linear_acceleration.x
        self.ayb = imu.linear_acceleration.y
        self.r = imu.angular_velocity.z     

        [_, _, self.psi] = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        
        [self.vx, self.vy]  = rotationZ(self.u, self.v, -self.psi)

        if (norm([self.vx, self.vy]) < 0.1):
            self.beta = 0
        else:
            self.beta = arctan2(self.v, self.u)
        self.isValid = True
        
    def dvlCallback(self, dvlMsg):
        self.uDVL = dvlMsg.vector.x
        self.vDVL = dvlMsg.vector.y
        if (norm([self.uDVL, self.vDVL]) < 0.1):
            self.betaDVL = 0
        else:
            self.betaDVL = arctan2(self.vDVL, self.uDVL)

    def podCallback(self, msg):
        if (msg.data[0] == 1):
            self.isPodFindTV = True
            self.tvAnglePod = msg.data[2]
        else:
            self.isPodFindTV = False
            self.tvAnglePod = None

    def lidarCallback(self, msg):   
        self.objectNum = len(msg.poses)

        # 通过两个 if 确保：
        # self.objectNum > 0 的 for 循环一定会执行
        # self.isPodFindTV 和 self.isLidarFindTV 有一个为 true
        if (self.objectNum <= 0):
            self.isLidarFindTV == False
            return

        if (self.isPodFindTV == False) & (self.isLidarFindTV == False):
            return

        # 记录激光雷达扫描到物体的 x y（船体系下）、船-物体方位角和距离
        objectX = zeros([self.objectNum, 1])
        objectY = zeros([self.objectNum, 1])
        objectAngle = zeros([self.objectNum, 1])
        objectDist = zeros([self.objectNum, 1])
        for i in range(self.objectNum):
            objectX[i, 0] = msg.poses[i].position.x
            objectY[i, 0] = msg.poses[i].position.y
            objectAngle[i, 0] = arctan2(objectY, objectX)
            objectDist[i, 0] = sqrt(objectX**2 + objectY**2)

        # 判断激光雷达扫描到的物体是否为目标船
        if (self.isPodFindTV):
            # 如果此时吊舱扫描到目标船，则比对吊舱偏航角和船-物体方位角
            # 若在角度容许范围内，则认为此物体是目标船，否则，进入激光雷达位置预测判断
            self.tLidar = msg.header.stamp
            dAnglePodLidar = abs(objectAngle - self.tvAnglePod)
            tvIndex = argmin(dAnglePodLidar)
            if (dAnglePodLidar[tvIndex, 0] <= self.angleTol):
                self.isLidarFindTV = True
                self.tvX = objectX[tvIndex, 0]
                self.tvY = objectY[tvIndex, 0]
                [self.xLidar, self.yLidar] = rotationZ(self.tvX, self.tvY, -self.psi)
                self.xLidar = -self.xLidar
                self.yLidar = -self.yLidar
                self.tvAngleLidar = objectAngle[tvIndex, 0]
                self.tvDist = objectDist[tvIndex, 0]       

        elif (self.isLidarFindTV):
            # 如果此时吊舱*未*扫描到目标船，则激光雷达必须先前锁定过目标船才能识别目标船
            # 否则，直接认为未识别到目标船，不会进入此 elif 段
            # 首先，通过上一时刻目标船位置和 USV 速度计算目标船这一时刻的预测位置
            # 然后，读取激光雷达这一时刻扫描到物体的真实位置
            # 随后，比对真实位置和预测位置的误差
            # 若在位置容许范围内，则识别到目标船；否则，认为未识别到目标船
            
            # 计算时间差
            dt = msg.header.stamp - self.tLidar
            # 如果时间差太大，则认为无法预测（船不再走直线）
            if (dt.to_sec() > 3.0):
                self.isLidarFindTV = False
                return

            # 记录此刻的时间戳
            self.tLidar = msg.header.stamp

            # 根据上一时刻的数据计算无人船的预测坐标 [usvENUEstX, usvENUEstY]
            [vx, vy] = rotationZ(self.uDVL, self.vDVL, -self.psi)
            usvEstX = self.xLidar + dt * vx
            usvEstY = self.yLidar + dt * vy

            # 根据这一时刻的所有物体位置，计算无人船相对于以物体为原点的 ENU 的坐标
            [usvPossibleX, usvPossibleY] = rotationZ(objectX, objectY, -self.psi)
            usvPossibleX = -usvPossibleX
            usvPossibleY = -usvPossibleY

            # 判断：如果预测位置与真实位置接近，则认为此物体是目标船
            dDist = norm([usvEstX - usvPossibleX, usvEstY - usvPossibleY])
            dDistIndex = argmin(dDist)
            if (dDist[dDistIndex] < self.distTol):
                self.isLidarFindTV = True
                self.tvX = objectX[dDistIndex, 0]
                self.tvY = objectY[dDistIndex, 0]
                [self.xLidar, self.yLidar] = rotationZ(self.tvX, self.tvY, -self.psi)
                self.xLidar = -self.xLidar
                self.yLidar = -self.yLidar
                self.tvAngleLidar = objectAngle[dDistIndex, 0]
                self.tvDist = objectDist[dDistIndex, 0]
                
        # for i in range(self.objectNum):
        #     objectX = msg.poses[i].position.x
        #     objectY = msg.poses[i].position.y
        #     objectAngle = arctan2(objectY, objectX)
        #     objectDist = sqrt(objectX**2 + objectY**2)

        #     # 判断激光雷达扫描到的物体是否为目标船   
        #     if (self.isPodFindTV):
        #         # 如果此时吊舱扫描到目标船，则比对吊舱偏航角和激光雷达扫描到物体的方位角
        #         # 若在角度容许范围内，则认为此物体是目标船，否则，认为未识别目标船
        #         if (abs(objectAngle - self.tvAnglePod) <= deg2rad(5)):
        #             self.isLidarFindTV = True
        #             self.tvX = objectX
        #             self.tvY = objectY
        #             [self.xLidar, self.yLidar] = rotationZ(objectX, objectY, -self.psi)
        #             self.xLidar = -self.xLidar
        #             self.yLidar = -self.yLidar
        #             self.tvAngleLidar = objectAngle
        #             self.tvDist = objectDist
        #             self.tLidarLast = self.tLidar
        #             self.tLidar = msg.header.stamp
        #             break
        #     elif (self.isLidarFindTV):
        #         # 如果此时吊舱*未*扫描到目标船，则激光雷达必须先前锁定过目标船才能识别目标船
        #         # 否则，直接认为未识别到目标船，不会进入此 elif 段
        #         # 首先，通过上一时刻目标船位置和 USV 速度计算目标船这一时刻的预测位置
        #         # 然后，读取激光雷达这一时刻扫描到物体的真实位置
        #         # 随后，比对真实位置和预测位置的误差
        #         # 若在位置容许范围内，则识别到目标船；否则，认为未识别到目标船

        #         # 获取时间差
        #         self.tLidarLast = self.tLidar
        #         self.tLidar = msg.header.stamp
        #         dt = self.tLidar.secs - self.tLidarLast.secs + 1e-9 * (self.tLidar.nsecs - self.tLidarLast.nsecs)

        #         # 如果时间差太大，则认为无法预测
        #         if (dt > 3.0):
        #             self.isLidarFindTV = False
        #             return

        #         # 根据上一时刻数据计算目标船的预测位置
        #         [vx, vy] = rotationZ(self.uDVL, self.vDVL, -self.psi)
        #         usvENUEstX = self.xLidar + dt * vx
        #         usvENUEstY = self.yLidar + dt * vy

        #         # 获取物体的真实位置
        #         [usvENUX, usvENUY] = rotationZ(objectX, objectY, -self.psi)
        #         usvENUX = -usvENUX
        #         usvENUY = -usvENUY

        #         # 判断：如果预测位置与真实位置接近，则认为此物体是目标船
        #         if (norm([usvENUEstX - usvENUX, usvENUEstY - usvENUY]) < 5.0):
        #             self.isLidarFindTV = True
        #             self.tvX = objectX
        #             self.tvY = objectY
        #             [self.xLidar, self.yLidar] = rotationZ(objectX, objectY, -self.psi)
        #             self.xLidar = -self.xLidar
        #             self.yLidar = -self.yLidar
        #             self.tvAngleLidar = objectAngle
        #             self.tvDist = objectDist
        #             break
        #     else:
        #         self.isLidarFindTV = False
                    

if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_pos_test_node')
    rosRate = rospy.Rate(2)

    usvPose = Pose()

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            rospy.loginfo("USV: [%.2f, %.2f]m | [%.2f, %.2f]m/s | [%.2f, %.2f]m/s^2 | %.2fdeg" % (usvPose.x, usvPose.y, usvPose.u, usvPose.v, usvPose.axb, usvPose.ayb, rad2deg(usvPose.psi)))

            if (usvPose.isPodFindTV):
                rospy.loginfo("吊舱扫描到目标船 %.2f deg" % rad2deg(usvPose.tvAnglePod))
            else:
                rospy.loginfo("吊舱未扫描到目标船")
            
            if (usvPose.isLidarFindTV):
                rospy.loginfo("激光雷达扫描到目标船 [%.2f, %.2f]m" % (usvPose.tvX, usvPose.tvY))
            elif (usvPose.objectNum > 0):
                rospy.loginfo("激光雷达扫描到 %d 个物体，但不认为它们是目标船" % usvPose.objectNum)
            else:
                rospy.loginfo("激光雷达未扫描到目标船")

            rosRate.sleep()
        except KeyboardInterrupt:
            break
