#!/usr/bin/env python3

import rospy, threading
from geometry_msgs.msg import Pose2D, PoseStamped, PoseArray

from numpy import arctan, deg2rad, zeros, abs, sqrt

class Communication():
    isTVEst = True
    tvEstPosX = 0
    tvEstPosY = 0
    course2TV = deg2rad(-176)
    isBigObj = False
    bigObjAngle = 0

    objectNum = 0
    objectPoses = PoseArray()
    objectAngle = zeros((20, 1))
    objectDist = zeros((20, 1))

    isLidarFindTV = False
    tvX = 0
    tvY = 0
    tvAngle = 0
    tvDist = 0

    def __init__(self):
        self.tvEstPosSub = rospy.Subscriber('/target_nav_position', Pose2D, self.tvOdomCallback)
        self.bigObjPosSub = rospy.Subscriber('/usv/big_obj/pose', PoseStamped, self.bigObjCallback)
        self.lidarSub = rospy.Subscriber('/filter/target', PoseArray, self.lidarCallback)

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.x
        self.tvEstPosY = msg.y
        self.course2TV = deg2rad(msg.theta)
        self.isTVEst = True

    def bigObjCallback(self, msg):
        self.bigObjAngle = arctan(msg.pose.position.y, msg.pose.position.x)
        self.isBigObj = True

    def lidarCallback(self, msg):
        self.objectNum = len(msg.poses)
        self.objectPoses = msg
        self.objectAngle = zeros((self.objectNum, 1))
        self.objectDist = zeros((self.objectNum, 1))

        for i in range(self.objectNum):
            self.objectAngle[i, 0] = arctan(self.objectPoses.poses[i].position.y/self.objectPoses.poses[i].position.x)
            self.objectDist[i, 0] = sqrt(self.objectPoses.poses[i].position.x**2 + self.objectPoses.poses[i].position.y**2)

            # 判断是否为目标船
            if (abs(self.objectAngle[i, 0]) < deg2rad(30)) & (self.objectDist[i, 0] < 100.0) & (self.objectPoses.poses[i].position.x > 0):
                self.isLidarFindTV = True
                self.tvX = self.objectPoses.poses[i].position.x
                self.tvY = self.objectPoses.poses[i].position.y
                self.tvAngle = self.objectAngle[i, 0]
                self.tvDist = self.objectDist[i, 0]
                break

if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_control_test_node')
    rate = 10
    rosRate = rospy.Rate(rate)
    usvComm = Communication()

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            if (usvComm.isLidarFindTV):
                rospy.loginfo("扫描到目标船 [%.2f, %.2f]m" % (usvComm.tvX, usvComm.tvY))
            else:
                rospy.loginfo("未扫描到目标船")
            rosRate.sleep()
        except KeyboardInterrupt:
            break
