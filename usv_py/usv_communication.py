#!/usr/bin/env python3

import rospy, threading
from geometry_msgs.msg import Pose2D, PoseStamped, PoseArray, Vector3Stamped
from std_msgs.msg import Float64MultiArray, Int8, String
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
    tvAngleEst = deg2rad(110)

    isArmFindBigObj = False
    bigObjAngle = 0


    def __init__(self):
        # 订阅搜索无人机提供目标船的话题
        self.tvEstPosSub = rospy.Subscriber('/target_nav_position', Pose2D, self.tvOdomCallback)

        # 订阅大物体定位的话题
        self.bigObjPosSub = rospy.Subscriber('/usv/big_obj/pose', PoseStamped, self.bigObjCallback)

        # 创建发送 tUAV1 起飞标志位的话题
        self.uavTakeOffFlagPub = rospy.Publisher('/tuav1_takeoff_flag', Int8, queue_size=2)
        self.tvPosFromLidarPub = rospy.Publisher('/target_lidar_position', Pose2D, queue_size=2)

        # 创建发送无人船状态的话题
        self.usvStatePub = rospy.Publisher('/usv/state', String, queue_size=2)

    def sendTakeOffFlag(self):
        self.uavTakeOffFlagPub.publish(Int8(data=1))
    
    def sendTVPosFromLidar(self, x, y):
        self.tvPosFromLidarPub.publish(Pose2D(x=x, y=y, theta=arctan2(y, x)))

    def sendUSVState(self, theState):
        self.usvStatePub.publish(String(data=theState))

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.x
        self.tvEstPosY = msg.y
        self.tvAngleEst = deg2rad(msg.theta)
        self.isSearchFindTV = True

    def bigObjCallback(self, msg):
        self.bigObjAngle = arctan(msg.pose.position.y, msg.pose.position.x)
        self.isArmFindBigObj = True

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
            # Add test code
            if (usvComm.isSearchFindTV):
                rospy.loginfo("sUAV 发现目标船在朝向 %.2f deg 处" % rad2deg(usvComm.tvAngleEst))
            else:
                rospy.loginfo("sUAV 未发现目标船")

            rosRate.sleep()
        except KeyboardInterrupt:
            break

    
        
