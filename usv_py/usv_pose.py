#!/usr/bin/env python3
import rospy, threading
import message_filters
from numpy import arctan2, rad2deg, arctan, sqrt, deg2rad
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
    beta =0.0
    r = 0.0
    
    isValid = False

    u_dvl = 0
    v_dvl = 0

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
        self.u_dvl = dvlMsg.vector.x
        self.v_dvl = dvlMsg.vector.y

    def podCallback(self, msg):
        if (msg.data[0] == 1):
            self.isPodFindTV = True
            self.tvAnglePod = msg.data[2]
        else:
            self.isPodFindTV = False
            self.tvAnglePod = None

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

        
