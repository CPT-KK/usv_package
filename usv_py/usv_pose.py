import message_filters
from numpy import arctan2
from numpy.linalg import norm
from usv_math import rotationZ
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from quaternions import Quaternion

class Pose(Node):
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

    t_lidar = 0
    x_lidar = 0
    y_lidar = 0
    vx_lidar = 0
    vy_lidar = 0
    x_lidar_last = 0
    y_lidat_last = 0

    def __init__(self):
        super().__init__('usv_pose_node')
        self.odomSub = message_filters.Subscriber(self, Odometry, '/usv/odom')
        self.imuSub = message_filters.Subscriber(self, Imu, '/usv/imu/data')

        ts = message_filters.ApproximateTimeSynchronizer([self.odomSub, self.imuSub], 10, 0.1)
        ts.registerCallback(self.poseCallback)

    def poseCallback(self, odom, imu):
        self.t = 0.5 * (odom.header.stamp.sec + 1e-9 * odom.header.stamp.nanosec + imu.header.stamp.sec + 1e-9 * imu.header.stamp.nanosec)
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.vx = odom.twist.twist.linear.x
        self.vy = odom.twist.twist.linear.y
        self.axb = imu.linear_acceleration.x
        self.ayb = imu.linear_acceleration.y
        self.r = imu.angular_velocity.z     

        q = Quaternion(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z)
        rpyAngle = Quaternion.get_euler(q)
        self.psi = rpyAngle[2]
        
        [self.u, self.v] = rotationZ(self.vx, self.vy, self.psi)
        if (norm([self.vx, self.vy]) < 0.1):
            self.beta = 0
        else:
            self.beta = arctan2(self.v, self.u)
        self.isValid = True



