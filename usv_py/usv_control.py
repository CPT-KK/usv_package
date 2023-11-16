#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16, Float32MultiArray

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
    rSPMax = deg2rad(8)
    
    # USV motor from 50RPM to 1300RPM
    # USV angle from -85deg to 85deg
    rpmThreshold = 2
    rpmMin = 35
    rpmMax = 1000
    angleMax = deg2rad(15)

    # 无人船参数
    usvMass = 775.0
    usvInerZ = 2072.475
    usvWidth = 3.3
    usvThrust2Center = 1.55

    # Actuator setpoint 量
    rpmLeftSP = 0
    rpmRightSP = 0
    angleLeftSP = 0
    angleRightSP = 0

    # Actuator 当前状态估计变量
    rpmLeftEst = 0
    rpmRightEst = 0
    angleLeftEst = 0
    angleRightEst = 0

    # 电池
    battSOC = [float("nan")] * 4
    battCellVoltMin = [float("nan")] * 4 

    def __init__(self, control_frequency):
        # For USV actuator setpoints
        self.lThrustPublisher_ = rospy.Publisher("/usv/torqeedo/left/setpoint", Int16, queue_size=10)
        self.rThrustPublisher_ = rospy.Publisher("/usv/torqeedo/right/setpoint", Int16, queue_size=10)
        self.lAnglePublisher_ = rospy.Publisher("/usv/pod/left/setpoint", Float32, queue_size=10)
        self.rAnglePublisher_ = rospy.Publisher("/usv/pod/right/setpoint", Float32, queue_size=10)

        # For USV actuator estimates
        self.lThrustSubscriber_ = rospy.Subscriber("/usv/torqeedo/left/estimate", Int16, self.lThrustCallback, queue_size=1)
        self.rThrustSubscriber_ = rospy.Subscriber("/usv/torqeedo/right/estimate", Int16, self.rThrustCallback, queue_size=1)
        self.lAngleSubscriber_ = rospy.Subscriber("/usv/pod/left/estimate", Float32, self.lAngleCallback, queue_size=1)
        self.rAngleSubscriber_ = rospy.Subscriber("/usv/pod/right/estimate", Float32, self.rAngleCallback, queue_size=1)

        # For USV battery info
        self.battSOCSubscriber_ = rospy.Subscriber("/usv/battery/soc", Float32MultiArray, self.battSOCCallback, queue_size=1)
        self.battCellVoltMinSubscriber_ = rospy.Subscriber("/usv/battery/cell_volt_min", Float32MultiArray, self.battCellVoltMinCallback, queue_size=1)

        # PID 初始化
        self.uPID = PID(0.8, 0.06, 0.012, control_frequency)
        self.vPID = PID(2, 0.0, 0.00, control_frequency)
        self.psiPID = PID(0.25, 0.0002, 0.02, control_frequency)
        self.rPID = PID(11.5, 0.5, 0.1, control_frequency)

        self.xPID = PID(0.3, 0.000, 0.000, control_frequency)
        self.yPID = PID(0.2, 0.000, 0.000, control_frequency)
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
        uSP = uSP * (0.05 + 0.95 * (1 - abs(psiErr) / pi))  

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
        self.mixer(axbSP, 0, etaSP)

        # 发布推力
        self.thrustPub()

        return [uSP, rSP]

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
        rSP = clip(rSP, -self.rSPMax, self.rSPMax)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.rPID.compute(rErr)

        # 送入混控
        self.mixer(axbSP, aybSP, etaSP)

        # 发布推力
        self.thrustPub()

        return [uSP, vSP, rSP]

    def moveUSVLateral(self, vSP, psiSP, v, ayb, psi, r):
        # 计算朝向角误差
        psiErr = psiSP - psi

        # 朝向角误差限幅
        psiErr = wrapToPi(psiErr)

        # 根据 psiErr 的值，计算可行的 vSP (避免速度太大，转弯转不过来)
        vSP = vSP * (0.5 + 0.5 * (1 - abs(psiErr) / pi))  

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
        self.mixerLateral(aybSP, etaSP)

        # 发布推力
        self.thrustPub()

        return [vSP, rSP]
    
    def mixer(self, axSP, aySP, etaSP):
        # 计算推力偏角
        self.angleLeftSP = arctan(aySP / axSP)
        self.angleRightSP = arctan(aySP / axSP)

        # 推力偏角限幅
        self.angleLeftSP = clip(self.angleLeftSP, -self.angleMax, self.angleMax)
        self.angleRightSP = clip(self.angleRightSP, -self.angleMax, self.angleMax)

        # 计算推力大小，目前这样除以 cos(self.angleLeftEst) 还不会出现除以 0 的情况
        rpmTranslate = self.usvMass * sign(axSP) * sqrt(axSP ** 2 + aySP ** 2) / 2.0
        rpmRotate = self.usvInerZ * etaSP / self.usvThrust2Center / 2.0
        self.rpmLeftSP = rpmTranslate * cos(self.angleLeftSP - self.angleLeftEst) - rpmRotate / cos(self.angleLeftEst)
        self.rpmRightSP = rpmTranslate * cos(self.angleRightSP - self.angleRightEst) + rpmRotate / cos(self.angleRightEst)

        # 推力大小限幅
        self.rpmLeftSP = clip(self.rpmLeftSP, -self.rpmMax, self.rpmMax)
        self.rpmRightSP = clip(self.rpmRightSP, -self.rpmMax, self.rpmMax)

        return

    def mixerLateral(self, aySP, etaSP):
        # 计算推力偏角
        self.angleLeftSP = deg2rad(90)
        self.angleRightSP = 0

        # 计算推力大小
        rpmTranslate = self.usvMass * aySP
        rpmRotate = self.usvInerZ * etaSP / self.usvThrust2Center / 2.0
        self.rpmLeftSP = rpmTranslate * cos(self.angleLeftSP - self.angleLeftEst)
        self.rpmRightSP = rpmRotate

        # 推力大小限幅
        self.rpmLeftSP = clip(self.rpmLeftSP, -self.rpmMax, self.rpmMax)
        self.rpmRightSP = clip(self.rpmRightSP, -self.rpmMax, self.rpmMax)

        return
    
    def thrustSet(self, rpmLeft, rpmRight, angleLeft, angleRight):
        self.rpmLeftSP = rpmLeft
        self.rpmRightSP = rpmRight
        self.angleLeftSP = angleLeft
        self.angleRightSP = angleRight

        return

    def thrustPub(self):
        lT = Int16(data=int(self.rpmLeftSP))
        rT = Int16(data=int(self.rpmRightSP))
        lA = Float32(data=rad2deg(-self.angleLeftSP))
        rA = Float32(data=rad2deg(-self.angleRightSP))

        self.lThrustPublisher_.publish(lT)
        self.rThrustPublisher_.publish(rT)
        self.lAnglePublisher_.publish(lA)
        self.rAnglePublisher_.publish(rA)

        return
    
    def lThrustCallback(self, msg):
        self.rpmLeftEst = msg.data * 4.0
        return
    
    def rThrustCallback(self, msg):
        self.rpmRightEst = msg.data * 4.0
        return
    
    def lAngleCallback(self, msg):
        self.angleLeftEst = deg2rad(-msg.data / 500.0)
        return
    
    def rAngleCallback(self, msg):
        self.angleRightEst = deg2rad(-msg.data / 500.0)
        return

    def battSOCCallback(self, msg):
        self.battSOC[int(msg.data[0]) - 1] = msg.data[1]
        return

    def battCellVoltMinCallback(self, msg):
        self.battCellVoltMin[int(msg.data[0]) - 1] = msg.data[1]
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
    angleValue = deg2rad(15)
    
    # rospy.loginfo("Moving left torq...")
    # for i in range(loopTimes):
    #     usvControl.thrustSet(rpmValue, 0, 0, 0)
    #     usvControl.thrustPub()
    #     rosRate.sleep()

    # rospy.loginfo("Moving right torq...")
    # for i in range(loopTimes):
    #     usvControl.thrustSet(0, rpmValue, 0, 0)
    #     usvControl.thrustPub()
    #     rosRate.sleep()

    rospy.loginfo("Moving left angle...")
    for i in range(loopTimes):
        usvControl.thrustSet(0, 0, deg2rad(15), deg2rad(0))
        usvControl.thrustPub()
        rosRate.sleep()

    rospy.loginfo("Moving right angle...")
    for i in range(loopTimes):
        usvControl.thrustSet(0, 0, deg2rad(15), deg2rad(-15))
        usvControl.thrustPub()
        rosRate.sleep()

    rospy.loginfo("Stopping...")
    for i in range(loopTimes):
        usvControl.thrustSet(0, 0, 0, 0)
        usvControl.thrustPub()
        rosRate.sleep()
