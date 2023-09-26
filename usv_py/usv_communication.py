#!/usr/bin/env python3

import rospy, threading
from geometry_msgs.msg import Pose2D, PoseStamped, PoseArray, Vector3Stamped
from std_msgs.msg import Float64MultiArray, Int8
from sensor_msgs.msg import Imu
from numpy import arctan, deg2rad, zeros, abs, sqrt, rad2deg, arctan2
from numpy.linalg import norm
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from usv_math import rotationZ

class Communication():
    uDVL = 0
    vDVL = 0
    r = 0

    isSearchFindTV = False
    tvEstPosX = 0
    tvEstPosY = 0
    course2TV = 0

    isArmFindBigObj = False
    bigObjAngle = 0

    isLidarFindTV = False
    objectNum = 0
    tLidarLast = 0
    tLidar = 0
    tvX = 0
    tvY = 0
    usvX = 0
    usvY = 0
    tvAngleLidar = 0
    tvDist = 0

    isPodFindTV = False
    tvAnglePod = 0

    def __init__(self):
        self.tvEstPosSub = rospy.Subscriber('/target_nav_position', Pose2D, self.tvOdomCallback)

        self.bigObjPosSub = rospy.Subscriber('/usv/big_obj/pose', PoseStamped, self.bigObjCallback)

        self.lidarSub = rospy.Subscriber('/filter/target', PoseArray, self.lidarCallback)
        self.podSub = rospy.Subscriber('/usv/pod_servo_ctrl/data', Float64MultiArray, self.podCallback)

        self.dvlVelSub = rospy.Subscriber('/mavros/vision_speed/speed_vector', Vector3Stamped, self.dvlCallback)

        self.px4IMUSub = rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)

        self.uavTakeOffFlagPub = rospy.Publisher('/tuav1_takeoff_flag', Int8, queue_size=2)
        self.tvPosFromLidarPub = rospy.Publisher('/target_lidar_position', Pose2D, queue_size=2)

    def sendTakeOffFlag(self):
        self.uavTakeOffFlagPub.publish(Int8(data=1))
    
    def sendTVPosFromLidar(self):
        if (self.isLidarFindTV):
            self.tvPosFromLidarPub.publish(Pose2D(x=-self.usvX, y=-self.usvY, theta=arctan2(-self.usvY, -self.usvX)))

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.x
        self.tvEstPosY = msg.y
        self.course2TV = deg2rad(msg.theta)
        self.isSearchFindTV = True

    def bigObjCallback(self, msg):
        self.bigObjAngle = arctan(msg.pose.position.y, msg.pose.position.x)
        self.isArmFindBigObj = True

    def lidarCallback(self, msg):   
        self.objectNum = len(msg.poses)
        self.tLidarLast = self.tLidar
        self.tLidar = msg.header.stamp

        if (self.objectNum <= 0):
            self.isLidarFindTV == False
            return

        if (self.isPodFindTV == False) & (self.isLidarFindTV == False):
            return

        # 到这里，一定可以确保：
        # self.objectNum > 0，for 循环一定会执行
        # self.isPodFindTV 和 self.isLidarFindTV 有一个为 true
        for i in range(self.objectNum):
            objectX = msg.poses[i].position.x
            objectY = msg.poses[i].position.y
            objectAngle = arctan(objectY / objectX)
            objectDist = sqrt(objectX**2 + objectY**2)

            # 判断激光雷达扫描到的物体是否为目标船   
            if (self.isPodFindTV):
                # 如果此时吊舱扫描到目标船，则比对吊舱偏航角和激光雷达扫描到物体的方位角
                # 若在角度容许范围内，则认为此物体是目标船，否则，认为未识别目标船
                if (abs(objectAngle - self.tvAnglePod) <= deg2rad(10)):
                    self.isLidarFindTV = True
                    self.tvX = objectX
                    self.tvY = objectY
                    [self.usvX, self.usvY] = rotationZ(objectX, objectY, -self.psi)
                    self.usvX = -self.usvX
                    self.usvY = -self.usvY
                    self.tvAngleLidar = objectAngle
                    self.tvDist = objectDist
                    break
            elif (self.isLidarFindTV):
                # 如果此时吊舱*未*扫描到目标船，则激光雷达必须先前锁定过目标船才能识别目标船
                # 否则，直接认为未识别到目标船，不会进入此 elif 段
                # 首先，通过上一时刻目标船位置和 USV 速度计算目标船这一时刻的预测位置
                # 然后，读取激光雷达这一时刻扫描到物体的真实位置
                # 随后，比对真实位置和预测位置的误差
                # 若在位置容许范围内，则识别到目标船；否则，认为未识别到目标船

                # 获取时间差
                dt = self.tLidar.secs - self.tLidarLast.secs + 1e-9 * (self.tLidar.nsecs - self.tLidarLast.nsecs)

                # 根据上一时刻数据计算目标船的预测位置
                [vx, vy] = rotationZ(self.uDVL, self.vDVL, -self.psi)
                usvENUEstX = self.usvX + dt * vx
                usvENUEstY = self.usvY + dt * vy

                # 获取物体的真实位置
                [usvENUX, usvENUY] = rotationZ(objectX, objectY, -self.psi)
                usvENUX = -usvENUX
                usvENUY = -usvENUY

                # 判断：如果预测位置与真实位置接近，则认为此物体是目标船是·
                if (norm([usvENUEstX - usvENUX, usvENUEstY - usvENUY]) < 5.0):
                    self.isLidarFindTV = True
                    self.tvX = objectX
                    self.tvY = objectY
                    [self.usvX, self.usvY] = rotationZ(objectX, objectY, -self.psi)
                    self.usvX = -self.usvX
                    self.usvY = -self.usvY
                    self.tvAngleLidar = objectAngle
                    self.tvDist = objectDist
                    break
            else:
                self.isLidarFindTV = False
                    

    def dvlCallback(self, dvlMsg):
        self.uDVL = dvlMsg.vector.x
        self.vDVL = dvlMsg.vector.y

    def podCallback(self, msg):
        if (msg.data[0] == 1):
            self.isPodFindTV = True
            self.tvAnglePod = msg.data[2]
        else:
            self.isPodFindTV = False
            self.tvAnglePod = None

    def imuCallback(self, imu):
        [_, _, self.psi] = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        self.r = imu.angular_velocity.z 

    # def podlidarCallback(self. msgPod, msgLidar):
        # pass


if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_comm_test_node')
    rate = 10
    rosRate = rospy.Rate(rate)
    usvComm = Communication()

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            if (usvComm.isLidarFindTV):
                rospy.loginfo("激光雷达扫描到目标船 [%.2f, %.2f]m" % (usvComm.tvX, usvComm.tvY))
            elif (usvComm.objectNum > 0):
                rospy.loginfo("激光雷达扫描到 %d 个物体，但不认为它们是目标船" % usvComm.objectNum)
            else:
                rospy.loginfo("激光雷达未扫描到目标船")

            if (usvComm.isPodFindTV):
                rospy.loginfo("吊舱扫描到目标船 %.2f deg" % rad2deg(usvComm.tvAnglePod))
            else:
                rospy.loginfo("吊舱未扫描到目标船")

            rosRate.sleep()
        except KeyboardInterrupt:
            break

    
        