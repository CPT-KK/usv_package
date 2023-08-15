#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

from numpy import arctan

class Communication():
    isTVEst = False
    tvEstPosX = 0
    tvEstPosY = 0

    isBigObj = False
    bigObjAngle = 0

    def __init__(self):
        self.tvEstPosSub = rospy.Subscriber('/usv/target/pose', PoseStamped, self.tvOdomCallback)
        self.bigObjPosSub = rospy.Subscriber('/usv/big_obj/pose', PoseStamped, self.bigObjCallback)

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.pose.position.x
        self.tvEstPosY = msg.pose.position.y
        self.isTVEst = True

    def bigObjCallback(self, msg):
        self.bigObjAngle = arctan(msg.pose.position.y, msg.pose.position.x)
        self.isBigObj = True
