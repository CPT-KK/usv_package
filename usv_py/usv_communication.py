from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

from numpy import arctan

class Communication(Node):
    isTVEst = True
    tvEstPosX = 695
    tvEstPosY = 100

    isBigObj = False
    bigObjAngle = 0

    def __init__(self):
        super().__init__('usv_comm_node')
        self.tvEstPosSub = self.create_subscription(PoseStamped, '/usv/target/pose', self.tvOdomCallback, 10)
        self.bigObjPosSub = self.create_subscription(PoseStamped, '/usv/big_obj/pose', self.bigObjCallback, 10)
        # self.transUAVPub

    def tvOdomCallback(self, msg):
        self.tvEstPosX = msg.pose.position.x
        self.tvEstPosY = msg.pose.position.y
        self.isTVEst = True

    def bigObjCallback(self, msg):
        self.bigObjAngle = arctan(msg.pose.position.y, msg.pose.position.x)
        self.isBigObj = True
