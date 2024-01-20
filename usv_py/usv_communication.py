#!/usr/bin/env python3

import rospy, threading
from geometry_msgs.msg import Pose2D, PoseStamped, PoseArray, Vector3Stamped, Vector3
from std_msgs.msg import Float64MultiArray, Int8, String, Float64
from sensor_msgs.msg import Imu
from numpy import arctan, deg2rad, zeros, abs, sqrt, rad2deg, arctan2
from numpy.linalg import norm
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from usv_math import rotationZ

import socket
import json
import re

class Communication():
    isArmFindBigObj = False
    largeObjX = 0
    largeObjY = 0
    largeObjAngle = 0

    suavState = "EMPTY"
    tuav1State = "EMPTY"
    armState = "EMPTY"

    def __init__(self):
        # 订阅大物体定位的话题
        self.bigObjPosSub = rospy.Subscriber('/usv/largebox_pos', PoseStamped, self.bigObjCallback)

        # 创建发送 tUAV1 起飞标志位的话题
        self.uavTakeOffFlagPub = rospy.Publisher('/tuav1_takeoff_flag', Int8, queue_size=2)
        self.tvPosFromLidarPub = rospy.Publisher('/target_lidar_position', Pose2D, queue_size=2)

        # 创建发送无人船状态的话题
        self.usvStatePub = rospy.Publisher('/usv/state', String, queue_size=2)

        # 创建发送数据链距离的话题
        # self.usvDatalinkPub = rospy.Publisher('/usv/data_link/distance', Vector3Stamped, queue_size=2)

        # 分系统状态
        self.suavStateSub = rospy.Subscriber('/suav/state', String, self.suavStateCallback, queue_size=10)
        self.tuav1StateSub = rospy.Subscriber('/tuav8/state', String, self.tuav1StateCallback, queue_size=10)
        self.armStateSub = rospy.Subscriber('/arm/state', String, self.armStateCallback, queue_size=10)
        self.releaseAttachStructPub = rospy.Publisher('/servo_control_cmd', Int8, queue_size=10)

    def __del__(self):
        pass
    
    def sendTakeOffFlag(self):
        self.uavTakeOffFlagPub.publish(Int8(data=1))
    
    def sendTVPosFromLidar(self, x, y, psi):
        self.tvPosFromLidarPub.publish(Pose2D(x=x, y=y, theta=psi))

    def sendUSVState(self, theState):
        self.usvStatePub.publish(String(data=theState))

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.x
        self.tvEstPosY = msg.y
        self.tvAngleEst = deg2rad(msg.theta)
        self.isSearchFindTV = True

    def bigObjCallback(self, msg):
        self.largeObjX = -msg.pose.position.y
        self.largeObjY = msg.pose.position.x + 1.55
        self.largeObjAngle = arctan(self.largeObjY / self.largeObjX)
        self.isArmFindBigObj = True

    def suavStateCallback(self, msg):
        self.suavState = msg.data

    def tuav1StateCallback(self, msg):
        self.tuav1State = msg.data

    def armStateCallback(self, msg):
        self.armStateCallback = msg.data

    def releaseAttachStruct(self, input):
        self.releaseAttachStructPub.publish(Int8(data=input))

    # def datalinkPub(self):
    #     with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    #         request_json = json.dumps({"get":"radioinfo"}).encode('utf-8')
    #         s.sendto(request_json,('192.168.147.50', 9999))
    #         s.settimeout(1.0)
            
    #         try:
    #             responce, _ = s.recvfrom(1024)
    #             responce_str = responce.decode('utf-8')

    #             match = re.search(r'{.*}',responce_str)

    #             if match:
    #                 valid_json = match.group(0)
    #                 try:
    #                     responce_data = json.loads(valid_json)

    #                 except json.JSONDecodeError as e:
    #                     print(f"JSON decoding error: {e}")

    #             for sender in responce_data["senders"]:
    #                 dist = sender["dist"]
    #                 ipAddr = sender["ipAddr"]
    #                 print("dist:", dist, "ipAddr:", ipAddr)

    #             out = Vector3Stamped()
    #             out.header.stamp = rospy.Time.now()
    #             out.vector.x = dist
    #             self.usvDatalinkPub.publish(out)

    #         except socket.timeout:
    #             print("Timed out waiting for a Datalink packet.")
        
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

            usvComm.datalinkPub()
            rosRate.sleep()
        except KeyboardInterrupt:
            break

    
        
