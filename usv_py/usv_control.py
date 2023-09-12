#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16

from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2, rad2deg, deg2rad, clip, abs, sign
from numpy.linalg import norm

from usv_math import rotationZ, wrapToPi

from PID import PID

class Control():
     
    # 控制器限幅参数
    xErrMax = 10.0
    yErrMax = 10.0

    uSPMax = 4.0
    rSPMax = deg2rad(45)
    uSPMax = 3
    vSPMax = 1

    # USV motor from 50RPM to 1300RPM
    # USV angle from -85deg to 85deg
    rpmThreshold = 20
    rpmMin = 50
    rpmMax = 1250
    angleMax = deg2rad(85) # 85 deg

    # 无人船参数
    usvMass = 775.0
    usvInerZ = 2072.475
    usvWidth = 2.4
    usvThrust2Center = 1.1

    XU = 0.0
    XUU = 0.0
    YU = 0.0
    YUU = 0.0
    NU = 0.0
    NUU = 0.0

    def __init__(self, control_frequency):
        self.lThrustPublisher_ = rospy.Publisher("/workshop_setup/pods/left", Int16, queue_size=10)
        self.rThrustPublisher_ = rospy.Publisher("/workshop_setup/pods/right", Int16, queue_size=10)
        self.lAnglePublisher_ = rospy.Publisher("/workshop_setup/pod_steer/left_steer", Float32, queue_size=10)
        self.rAnglePublisher_ = rospy.Publisher("/workshop_setup/pod_steer/right_steer", Float32, queue_size=10)

        # PID 初始化
        self.uPID = PID(0.75, 0.05, 0.01, control_frequency)
        self.psiPID = PID(1, 0.00, 0.0, control_frequency)
        self.rPID = PID(2, 0.01, 0.1, control_frequency)

        self.xPID = PID(0.12, 0.000, 0.000, control_frequency)
        self.yPID = PID(0.15, 0.000, 0.000, control_frequency)
        self.vxPID = PID(0.4, 0.00, 0.05, control_frequency)
        self.vyPID = PID(0.5, 0.00, 0.05, control_frequency)

    def __del__(self):
        # 析构时，关闭无人船推力输出
        
        for i in range(20):
            self.thrustPub(0.0, 0.0, 0.0, 0.0)


    def moveUSV(self, uSP, psiSP, u, axb, psi, r):
        # 计算船体系速度

        # 限幅
        uSP = clip(uSP, - self.uSPMax, self.uSPMax)

        # 计算轴向速度误差
        uErr = uSP - u

        # 计算推力大小
        axbSP = self.uPID.compute(uErr, axb)
        aybSP = 0.0

        # 计算朝向角误差
        psiErr = psiSP - psi

        # 朝向角误差限幅
        psiErr = wrapToPi(psiErr)

        # 计算期望朝向角速度
        rSP = self.psiPID.compute(psiErr, r)

        # 期望朝向角速度限幅
        rSP = clip(rSP, - self.rSPMax, self.rSPMax)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.rPID.compute(rErr)

        # 混控计算
        [rpmL, rpmR, angleL, angleR] = self.mixer(axbSP, aybSP, etaSP)

        # 发布推力
        self.thrustPub(rpmL, rpmR, angleL, angleR)

        # print("=============================================================================")
        # print("Control 输出: ")
        # print("USV 位置：[%.2f, %.2f]m | USV 速度：[%.2f, %.2f]m/s" % (x, y, vx, vy))
        # print("u 速度误差：%.2fm/s, 角度误差：%.2f, 角速度误差: %.2f," % (uErr, psiErr, rErr))
        # print("左右转速 Setpoint：[%.2f, %.2f] rpm" % (rpmL, rpmR))

        return
    
    def moveUSVVec2(self, uSP, vSP, psiSP, u, v, axb, ayb, psi, r):
        # u v setpoints 限幅
        uSP = clip(uSP, -self.uSPMax, self.uSPMax)
        vSP = clip(vSP, -self.vSPMax, self.vSPMax)

        # 计算 vx vy 误差
        uErr = uSP - u
        vErr = vSP - v

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

        return

    def moveUSVVec(self, xSP, ySP, psiSP, x, y, vx, vy, axb, ayb, psi, r):
        # 计算 x y 误差
        xErr = xSP - x
        yErr = ySP - y
        
        # 计算 vx vy setpoints
        vxSP = self.xPID.compute(xErr, vx)
        vySP = self.yPID.compute(yErr, vy)

        # vx vy setpoints 限幅
        vxSP = clip(vxSP, - self.uSPMax, self.uSPMax)
        vySP = clip(vySP, - self.vSPMax, self.vSPMax)
       
        # 计算 vx vy 误差
        vxErr = vxSP - vx
        vyErr = vySP - vy

        # 计算船体系速度误差，作为加速度 setpoints
        [uErr, vErr] = rotationZ(vxErr, vyErr, psi)

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

        return

    def mixer(self, axSP, aySP, etaSP):
        # 计算推力偏角
        angleL = arctan(aySP / axSP)
        angleR = arctan(aySP / axSP)

        # 推力偏角限幅
        angleL = clip(angleL, -self.angleMax, self.angleMax)
        angleR = clip(angleR, -self.angleMax, self.angleMax)

        # 计算推力大小
        rpmL = self.usvMass * sign(axSP) * norm([axSP, aySP]) / 2.0 - self.usvInerZ * etaSP / (2.0 * self.usvThrust2Center * cos(angleL))
        rpmR = self.usvMass * sign(axSP) * norm([axSP, aySP]) / 2.0 + self.usvInerZ * etaSP / (2.0 * self.usvThrust2Center * cos(angleR))

        # 推力大小限幅
        rpmL = clip(rpmL, -self.rpmMax, self.rpmMax)
        rpmR = clip(rpmR, -self.rpmMax, self.rpmMax)

        if (abs(rpmL) <= self.rpmMin) & (abs(rpmL) >= self.rpmThreshold):
            rpmL = sign(rpmL) * self.rpmMin
        if (abs(rpmL) <= self.rpmMin) & (abs(rpmL) < self.rpmThreshold):
            rpmL = 0

        if (abs(rpmR) <= self.rpmMin) & (abs(rpmR) >= self.rpmThreshold):
            rpmR = sign(rpmR) * self.rpmMin
        if (abs(rpmR) <= self.rpmMin) & (abs(rpmR) < self.rpmThreshold):
            rpmR = 0  

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
    usvControl = Control(5)

    lastTime = 8
    loopTimes = round(lastTime / (1 / rate))

    rpmValue = 50
    angleValue = deg2rad(85)
    
    rospy.loginfo("Moving forward...")
    for i in range(loopTimes):
        usvControl.thrustPub(rpmValue, rpmValue, 0, 0)
        rosRate.sleep()

    # rospy.loginfo("Moving backward...")
    # for i in range(loopTimes):
    #     usvControl.thrustPub(-rpmValue, -rpmValue, 0, 0)
    #     rosRate.sleep()

    # rospy.loginfo("Moving left forward...")
    # for i in range(loopTimes):
    #     usvControl.thrustPub(rpmValue, rpmValue, angleValue, angleValue)
    #     rosRate.sleep()

    # rospy.loginfo("Moving right forward...")
    # for i in range(loopTimes):
    #     usvControl.thrustPub(rpmValue, rpmValue, -angleValue, -angleValue)
    #     rosRate.sleep()

    rospy.loginfo("Stopping...")
    for i in range(loopTimes):
        usvControl.thrustPub(0, 0, 0, 0)
        rosRate.sleep()
