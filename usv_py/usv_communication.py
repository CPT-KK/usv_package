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
    uDVL = 0
    vDVL = 0
    r = 0

    isSearchFindTV = True
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

        # 创建发送数据链距离的话题
        self.usvDatalinkPub = rospy.Publisher('/usv/data_link/distance', Vector3Stamped, queue_size=2)

    def __del__(self):
        pass
    
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

    def datalinkPub(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            request_json = json.dumps({"get":"radioinfo"}).encode('utf-8')
            s.sendto(request_json,('192.168.147.50', 9999))
            s.settimeout(1.0)
            
            try:
                responce, _ = s.recvfrom(1024)
                responce_str = responce.decode('utf-8')

                match = re.search(r'{.*}',responce_str)

                if match:
                    valid_json = match.group(0)
                    try:
                        responce_data = json.loads(valid_json)

                    except json.JSONDecodeError as e:
                        print(f"JSON decoding error: {e}")

                for sender in responce_data["senders"]:
                    dist = sender["dist"]
                    ipAddr = sender["ipAddr"]
                    print("dist:", dist, "ipAddr:", ipAddr)

                out = Vector3Stamped()
                out.header.stamp = rospy.Time.now()
                out.vector.x = dist
                self.usvDatalinkPub.publish(out)

            except socket.timeout:
                print("Timed out waiting for a Datalink packet.")
        
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

    
        
