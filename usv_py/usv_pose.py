#!/usr/bin/env python3
import rospy, threading
import message_filters
from numpy import arctan2
from numpy.linalg import norm
from usv_math import rotationZ

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# from quaternions import Quaternion
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

    t_lidar = 0
    x_lidar = 0
    y_lidar = 0
    vx_lidar = 0
    vy_lidar = 0
    x_lidar_last = 0
    y_lidat_last = 0

    def __init__(self):
        # For PX4 MAVROS local position and velocity (Velocity is in USV body frame)
        self.px4OdomSub = message_filters.Subscriber('/mavros/local_position/odom', Odometry) 

        # For PX4 MAVROS IMU
        self.px4IMUSub = message_filters.Subscriber('/mavros/imu/data', Imu) 

        self.ts = message_filters.ApproximateTimeSynchronizer([self.px4OdomSub, self.px4IMUSub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.poseCallback)

    def poseCallback(self, odom, imu):
        
        self.t = 0.5 * (odom.header.stamp.secs + 1e-9 * odom.header.stamp.nsecs + imu.header.stamp.secs + 1e-9 * imu.header.stamp.nsecs)
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.u = odom.twist.twist.linear.x
        self.v = odom.twist.twist.linear.y
        self.axb = imu.linear_acceleration.x
        self.ayb = imu.linear_acceleration.y
        self.r = imu.angular_velocity.z     

        [_, _, self.psi] = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.w, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        
        [self.vx, self.vy]  = rotationZ(self.u, self.v, -self.psi)

        if (norm([self.vx, self.vy]) < 0.1):
            self.beta = 0
        else:
            self.beta = arctan2(self.v, self.u)
        self.isValid = True
        

if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_pos_test_node')
    rosRate = rospy.Rate(2)

    usvPose = Pose()

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            rospy.loginfo("USV: [%.2f, %.2f]m | [%.2f, %.2f]m/s | [%.2f, %.2f]m/s^2 | %.2fdeg" % (usvPose.x, usvPose.y, usvPose.vx, usvPose.vy, usvPose.axb, usvPose.ayb, usvPose.psi * 57.3))
            rosRate.sleep()
        except KeyboardInterrupt:
            break

        
