#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import PointStamped

from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2, rad2deg, deg2rad, clip, abs, sign, pi, sqrt, array, max
from numpy.linalg import norm

from usv_math import rotationZ, wrapToPi

from PID import PID

class Control():
     
    # 控制器限幅参数
    xErrMax = 10.0
    yErrMax = 10.0

    uSPMax = 4.0
    vSPMax = 1.25
    rSPMax = deg2rad(10)
    
    # USV motor from 50RPM to 1300RPM
    # USV angle from -85deg to 85deg
    rpmThreshold = 10
    rpmMin = 40
    rpmMax = 1000
    angleMax = deg2rad(90)

    # 无人船参数
    usvMass = 775.0
    usvInerZ = 2072.475
    usvWidth = 3.3
    usvThrust2Center = 1.55

    def __init__(self, control_frequency):
        self.lThrustPublisher_ = rospy.Publisher("/workshop_setup/pods/left", Int16, queue_size=10)
        self.rThrustPublisher_ = rospy.Publisher("/workshop_setup/pods/right", Int16, queue_size=10)
        self.lAnglePublisher_ = rospy.Publisher("/workshop_setup/pod_steer/left_steer", Float32, queue_size=10)
        self.rAnglePublisher_ = rospy.Publisher("/workshop_setup/pod_steer/right_steer", Float32, queue_size=10)

        # PID 初始化
        self.uPID = PID(0.8, 0.06, 0.012, control_frequency)
        self.vPID = PID(0.5, 0.02, 0.008, control_frequency)
        self.psiPID = PID(1.15, 0.008, 0.00, control_frequency)
        self.rPID = PID(2.1, 0.03, 0.005, control_frequency)

        self.xPID = PID(0.3, 0.000, 0.000, control_frequency)
        self.yPID = PID(0.15, 0.000, 0.000, control_frequency)
        self.vxPID = PID(0.6, 0.0012, 0.05, control_frequency)
        self.vyPID = PID(0.3, 0.0008, 0.05, control_frequency)

    def __del__(self):
        pass

    def moveUSV(self, uSP, psiSP, u, axb, psi, r):
        # 计算朝向角误差
        psiErr = psiSP - psi

        # 朝向角误差限幅
        psiErr = wrapToPi(psiErr)

        # 根据 psiErr 的值，计算可行的 uSP (避免速度太大，转弯转不过来)
        uSP = uSP * (0.1 + 0.9 * (1 - abs(psiErr) / pi))  

        # 轴向速度限幅
        uSP = clip(uSP, -self.uSPMax, self.uSPMax)

        # 计算轴向速度误差
        uErr = uSP - u

        # 计算推力大小
        axbSP = self.uPID.compute(uErr, axb)

        # 计算期望朝向角速度
        rSP = self.psiPID.compute(psiErr, r)

        # 期望朝向角速度限幅
        rSP = clip(rSP, -self.rSPMax, self.rSPMax)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.rPID.compute(rErr)

        # 混控计算
        [rpmL, rpmR, angleL, angleR] = self.mixer(axbSP, 0, etaSP)

        # 发布推力
        self.thrustPub(rpmL, rpmR, angleL, angleR)

        return uSP

    def moveUSVVec(self, xSP, ySP, psiSP, x, y, u, v, axb, ayb, psi, r):
        # 计算 x y 误差
        xErr = xSP - x
        yErr = ySP - y

        # x y 误差转船体系
        [xErr, yErr] = rotationZ(xErr, yErr, psi)
        
        # 计算 vx vy setpoints
        uSP = self.xPID.compute(xErr, u)
        vSP = self.yPID.compute(yErr, v)

        # u v setpoints 限幅
        uSP = clip(uSP, -1.0, 1.0)
        vSP = clip(vSP, -0.8, 0.8)
       
        # 计算 vx vy 误差
        uErr = uSP - u
        vErr = vSP - v

        # 计算加速度 setpoints 大小
        axbSP = self.vxPID.compute(uErr, axb)
        aybSP = self.vyPID.compute(vErr, ayb)

        # 计算朝向角误差
        psiErr = psiSP - psi

        # 朝向角误差限幅
        psiErr = wrapToPi(psiErr)

        # 计算期望朝向角速度
        rSP = self.psiPID.compute(psiErr, r)

        # 期望朝向角速度限幅
        rSP = wrapToPi(rSP)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.rPID.compute(rErr)

        # 送入混控
        [rpmL, rpmR, angleL, angleR] = self.mixer(axbSP, aybSP, etaSP)

        # 发布推力
        self.thrustPub(rpmL, rpmR, angleL, angleR)

        return [uSP, vSP]

    def moveUSVLateral(self, vSP, psiSP, v, ayb, psi, r):
        # 计算朝向角误差
        psiErr = psiSP - psi

        # 朝向角误差限幅
        psiErr = wrapToPi(psiErr)

        # 根据 psiErr 的值，计算可行的 vSP (避免速度太大，转弯转不过来)
        vSP = vSP * (0.1 + 0.9 * (1 - abs(psiErr) / pi))  

        # 侧向速度限幅
        vSP = clip(vSP, -self.vSPMax, self.vSPMax)

        # 计算侧向速度误差
        vErr = vSP - v

        # 计算推力大小
        aybSP = self.vPID.compute(vErr, ayb)

        # 计算期望朝向角速度
        rSP = self.psiPID.compute(psiErr, r)

        # 期望朝向角速度限幅
        rSP = clip(rSP, -self.rSPMax, self.rSPMax)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.rPID.compute(rErr)

        # 混控计算
        [rpmL, rpmR, angleL, angleR] = self.mixerLateral(aybSP, etaSP)

        # 发布推力
        self.thrustPub(rpmL, rpmR, angleL, angleR)

        return vSP
    
    def mixer(self, axSP, aySP, etaSP):
        # 计算推力偏角
        angleL = arctan(aySP / axSP)
        angleR = arctan(aySP / axSP)

        # 推力偏角限幅
        angleL = clip(angleL, -self.angleMax, self.angleMax)
        angleR = clip(angleR, -self.angleMax, self.angleMax)

        # 计算推力大小
        rpmTranslate = self.usvMass * sign(axSP) * sqrt(axSP ** 2 + aySP ** 2)
        rpmRotate = self.usvInerZ * etaSP / (self.usvThrust2Center * cos(angleL))
        rpmL = rpmTranslate / 2.0 - rpmRotate / 2.0
        rpmR = rpmTranslate / 2.0 + rpmRotate / 2.0

        # 推力大小限幅
        if (abs(rpmL) < self.rpmThreshold) | (abs(rpmR) < self.rpmThreshold):
            rpmL = 0
            rpmR = 0
        elif (abs(rpmL) <= self.rpmMin) | (abs(rpmR) <= self.rpmMin):  
            rpmL = rpmL * max(array([self.rpmMin / abs(rpmL), self.rpmMin / abs(rpmR)]))
            rpmR = rpmR * max(array([self.rpmMin / abs(rpmL), self.rpmMin / abs(rpmR)]))

        rpmL = clip(rpmL, -self.rpmMax, self.rpmMax)
        rpmR = clip(rpmR, -self.rpmMax, self.rpmMax)

        return [rpmL, rpmR, angleL, angleR]

    def mixerLateral(self, aySP, etaSP):
        # 计算推力偏角
        angleL = self.angleMax
        angleR = 0

        # 计算推力大小
        rpmL = self.usvMass * aySP
        rpmR = self.usvInerZ * etaSP / self.usvThrust2Center

        # 推力大小限幅
        if (abs(rpmL) < self.rpmThreshold) | (abs(rpmR) < self.rpmThreshold):
            rpmL = 0
            rpmR = 0
        elif (abs(rpmL) <= self.rpmMin) | (abs(rpmR) <= self.rpmMin):  
            rpmL = rpmL * max(array([self.rpmMin / abs(rpmL), self.rpmMin / abs(rpmR)]))
            rpmR = rpmR * max(array([self.rpmMin / abs(rpmL), self.rpmMin / abs(rpmR)]))

        rpmL = clip(rpmL, -self.rpmMax, self.rpmMax)
        rpmR = clip(rpmR, -self.rpmMax, self.rpmMax)

        return [rpmL, rpmR, angleL, angleR]
    
    def thrustPub(self, rpmL, rpmR, angleL, angleR):
        lT = Int16(data=int(rpmL))
        rT = Int16(data=int(rpmR))
        lA = Float32(data=rad2deg(-angleL))
        rA = Float32(data=rad2deg(-angleR))

        self.lThrustPublisher_.publish(lT)
        self.rThrustPublisher_.publish(rT)
        self.lAnglePublisher_.publish(lA)
        self.rAnglePublisher_.publish(rA)

        return
    
if __name__ == '__main__':
    # 以下代码为测试代码
    rospy.init_node('usv_control_test_node')
    rate = 10
    rosRate = rospy.Rate(rate)
    usvControl = Control(rate)

    lastTime = 10
    loopTimes = round(lastTime / (1 / rate))

    rpmValue = 150
    angleValue = deg2rad(90)
    
    # rospy.loginfo("Moving left torq...")
    # for i in range(loopTimes):
    #     usvControl.thrustPub(rpmValue, 0, 0, 0)
    #     rosRate.sleep()

    # rospy.loginfo("Moving right torq...")
    # for i in range(loopTimes):
    #     usvControl.thrustPub(0, rpmValue, 0, 0)
    #     rosRate.sleep()

    rospy.loginfo("Moving left angle...")
    for i in range(loopTimes):
        usvControl.thrustPub(0, 0, deg2rad(89.9), deg2rad(0))
        rosRate.sleep()

    rospy.loginfo("Moving right angle...")
    for i in range(loopTimes):
        usvControl.thrustPub(0, 0, deg2rad(89.9), deg2rad(-89.9))
        rosRate.sleep()

    rospy.loginfo("Stopping...")
    for i in range(loopTimes):
        usvControl.thrustPub(0, 0, 0, 0)
        rosRate.sleep()
