#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped

from numpy import arctan, deg2rad

class Communication():
    isTVEst = False
    tvEstPosX = 0
    tvEstPosY = 0

    isBigObj = False
    bigObjAngle = 0

    def __init__(self):
        self.tvEstPosSub = rospy.Subscriber('/target_nav_position', Pose2D, self.tvOdomCallback)
        self.bigObjPosSub = rospy.Subscriber('/usv/big_obj/pose', PoseStamped, self.bigObjCallback)

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.x
        self.tvEstPosY = msg.y
        self.course2TV = deg2rad(msg.theta)
        self.isTVEst = True

    def bigObjCallback(self, msg):
        self.bigObjAngle = arctan(msg.pose.position.y, msg.pose.position.x)
        self.isBigObj = True
