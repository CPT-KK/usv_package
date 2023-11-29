#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16, Float32MultiArray

from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2, rad2deg, deg2rad, clip, abs, sign, pi, sqrt, array, max
from numpy.linalg import norm

from usv_math import rotationZ, wrapToPi

from PID import PID

class Control():
     
    # 控制器限幅参数
    __axbSPMax = 3.0        # 控制器中 axbSP 的最大值
    __aybSPMax = 1.0        # 矢量控制器中 aybSP 的最大值

    __uSPMax = 4.0          # 差动控制器中 uSP 的最大值
    __uSPVecMax = 1.0       # 矢量控制器中 uSP 的最大值
    __vSPMax = 1.0          # 矢量控制器中 vSP 的最大值
    __rSPMax = deg2rad(8)   # 控制器中 rSP 的最大值
    
    # 矢量控制器所处状态
    vecCtrlState = 0        # 0: 正在修正轴向和航向的误差；1: 正在修正侧向的误差

    # USV RPM and pod angle limits
    __rpmMin = 35                   # 最小转速
    __rpmMax = 1000                 # 最大转速
    __rpmRotateMax = 800            # 用于转动的最大转速
    __angleMax = deg2rad(90)        # 舵角最大值
    __angleMaxState0 = deg2rad(20)  # Control state 0 下的舵角最大值

    # 无人船参数
    __MASS = 775.0          # 质量
    __INERZ = 2072.475      # 惯性矩
    __TORQLEN = 1.55        # 力臂长度

    # Actuator setpoint 量
    rpmLeftSP = 0       # 左侧发动机转速 setpoint   
    rpmRightSP = 0      # 右侧发动机转速 setpoint
    angleLeftSP = 0     # 左侧舵角 setpoint
    angleRightSP = 0    # 右侧舵角 setpoint

    # Actuator 当前状态估计变量
    rpmLeftEst = float("nan")      # 左侧发动机转速 estimate
    rpmRightEst = float("nan")     # 右侧发动机转速 estimate
    angleLeftEst = float("nan")    # 左侧舵角 estimate
    angleRightEst = float("nan")   # 右侧舵角 estimate

    # 电池信息
    battSOC = [float("nan")] * 4            # 电池剩余容量百分比
    battCellVoltMin = [float("nan")] * 4    # 电池最低单体电压

    def __init__(self):
        # For USV actuator setpoints
        self.__lThrustPublisher = rospy.Publisher("/usv/torqeedo/left/setpoint", Int16, queue_size=10)
        self.__rThrustPublisher = rospy.Publisher("/usv/torqeedo/right/setpoint", Int16, queue_size=10)
        self.__lAnglePublisher = rospy.Publisher("/usv/pod/left/setpoint", Float32, queue_size=10)
        self.__rAnglePublisher = rospy.Publisher("/usv/pod/right/setpoint", Float32, queue_size=10)

        # For USV actuator estimates
        self.__lThrustSubscriber = rospy.Subscriber("/usv/torqeedo/left/estimate", Int16, self.lThrustCallback, queue_size=1)
        self.__rThrustSubscriber = rospy.Subscriber("/usv/torqeedo/right/estimate", Int16, self.rThrustCallback, queue_size=1)
        self.__lAngleSubscriber = rospy.Subscriber("/usv/pod/left/estimate", Float32, self.lAngleCallback, queue_size=1)
        self.__rAngleSubscriber = rospy.Subscriber("/usv/pod/right/estimate", Float32, self.rAngleCallback, queue_size=1)

        # For USV battery info
        self.__battSOCSubscriber = rospy.Subscriber("/usv/battery/soc", Float32MultiArray, self.battSOCCallback, queue_size=1)
        self.__battCellVoltMinSubscriber = rospy.Subscriber("/usv/battery/min_cell_volt", Float32MultiArray, self.battCellVoltMinCallback, queue_size=1)

        # PID 初始化
        self.__uPID = PID(0.8, 0.06, -0.012)
        self.__psiPID = PID(0.23, 0.0002, -0.02)
        self.__rPID = PID(14, 0.5, -0.1)

        self.__xPID = PID(0.5, 0.00, 0.0)
        self.__yPID = PID(0.4, 0.00, 0.0)
        self.__vxPID = PID(1.2, 0.00, -0.0)
        self.__vyPID = PID(0.7, 0.00, -0.0)

    def __del__(self):
        pass

    def moveUSV(self, uSP, psiSP, u, axb, psi, r):
        # 计算朝向角误差
        psiErr = psiSP - psi

        # 朝向角误差限幅
        psiErr = wrapToPi(psiErr)

        # 根据 psiErr 的值，计算可行的 uSP (避免速度太大，转弯转不过来)
        if (psiErr > pi / 4):
            uSP = 0
        else:
            uSP = uSP * (0.1 + 0.9 * (1 - abs(psiErr) / (pi / 4)))  

        # 轴向速度限幅
        uSP = clip(uSP, -self.__uSPMax, self.__uSPMax)

        # 计算轴向速度误差
        uErr = uSP - u

        # 计算推力大小
        axbSP = self.__uPID.compute(uErr, axb)

        # 加速度 setpoints 限幅
        axbSP = clip(axbSP, -self.__axbSPMax, self.__axbSPMax)

        # 计算期望朝向角速度
        rSP = self.__psiPID.compute(psiErr, r)

        # 期望朝向角速度限幅
        rSP = clip(rSP, -self.__rSPMax, self.__rSPMax)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.__rPID.compute(rErr)

        # 混控计算
        self.mixer(axbSP, 0, etaSP)

        # 发布推力
        self.thrustPub()

        return [uSP, rSP, axbSP, etaSP]

    def moveUSVVec(self, xSP, ySP, psiSP, x, y, u, v, axb, ayb, psi, r):
        # 计算 x y 误差
        xErr = xSP - x
        yErr = ySP - y

        # x y 误差转船体系
        [xErr, yErr] = rotationZ(xErr, yErr, psi)

        # 选择矢量控制器状态
        if ((abs(yErr) > 2.0) | (abs(v) > 1.0)) & (self.vecCtrlState == 0):
            self.vecCtrlState = 1
        elif (abs(yErr) <= 1.0) & (abs(ayb) <= 0.5) & (self.vecCtrlState == 1):
            self.vecCtrlState = 0

        uSP = 0.0
        vSP = 0.0
        rSP = 0.0
        axbSP = 0.0
        aybSP = 0.0
        etaSP = 0.0
        if (self.vecCtrlState == 1):
            # 计算并修正侧向误差
            vSP = self.__yPID.compute(yErr, v)
            vSP = clip(vSP, -self.__vSPMax, self.__vSPMax)
            vErr = vSP - v
            aybSP = self.__vyPID.compute(vErr, ayb)
            aybSP = clip(aybSP, -self.__aybSPMax, self.__aybSPMax)
        else:
            # 计算并修正轴向误差
            uSP = self.__xPID.compute(xErr, u)
            uSP = clip(uSP, -self.__uSPVecMax, self.__uSPVecMax)
            uErr = uSP - u
            axbSP = self.__vxPID.compute(uErr, axb)
            axbSP = clip(axbSP, -self.__axbSPMax, self.__axbSPMax)

            # 计算并修正侧向误差
            vSP = self.__yPID.compute(yErr, v)
            vSP = clip(vSP, -self.__vSPMax, self.__vSPMax)
            vErr = vSP - v
            aybSP = self.__vyPID.compute(vErr, ayb)
            aybSP = clip(aybSP, -self.__aybSPMax, self.__aybSPMax)

            # 计算并修正航向误差
            psiErr = psiSP - psi
            psiErr = wrapToPi(psiErr)
            rSP = self.__psiPID.compute(psiErr, r)
            rSP = clip(rSP, -self.__rSPMax, self.__rSPMax)
            rErr = rSP - r
            etaSP = self.__rPID.compute(rErr)

            # 侧向加速度要被轴向加速度限幅
            aybSP = clip(aybSP, -axbSP * tan(self.__angleMaxState0), axbSP * tan(self.__angleMaxState0))
            
        # 送入混控
        self.mixer(axbSP, aybSP, etaSP)

        # 发布推力
        self.thrustPub()

        return [uSP, vSP, rSP, axbSP, aybSP, etaSP]

    def mixer(self, axSP, aySP, etaSP):       
        # 计算推力偏角
        self.angleLeftSP = arctan(aySP / axSP)
        self.angleRightSP = arctan(aySP / axSP)

        # 推力偏角限幅
        self.angleLeftSP = clip(self.angleLeftSP, -self.__angleMax, self.__angleMax)
        self.angleRightSP = clip(self.angleRightSP, -self.__angleMax, self.__angleMax)

        # 计算一侧发动机平动和转动所需推力大小的参考值
        if (axSP == 0.0):
            rpmTranslate = self.__MASS * 1.0 * sqrt(axSP ** 2 + aySP ** 2) / 2.0
        else:
            rpmTranslate = self.__MASS * sign(axSP) * sqrt(axSP ** 2 + aySP ** 2) / 2.0
        rpmRotate = self.__INERZ * etaSP / self.__TORQLEN / 2.0
        
        # 计算左右两侧平动所需推力
        # 当推力角的当前值和期望值相差较大时，实际平动推力变小
        rpmTranslateLeft = rpmTranslate * cos(self.angleLeftSP - self.angleLeftEst)
        rpmTranslateRight = rpmTranslate * cos(self.angleRightSP - self.angleRightEst)
        
        # 计算左右两侧转动所需推力
        # 为了避免除以 cos(self.angleLeftEst) 出现除以 0 导致结果为无穷的情况，加一个限幅
        rpmRotateLeft = clip(rpmRotate / cos(self.angleLeftEst), -self.__rpmRotateMax, self.__rpmRotateMax)
        rpmRotateRight = clip(rpmRotate / cos(self.angleRightEst), -self.__rpmRotateMax, self.__rpmRotateMax)

        # 计算左右两侧真实推力设置值的大小
        self.rpmLeftSP = rpmTranslateLeft - rpmRotateLeft
        self.rpmRightSP = rpmTranslateRight + rpmRotateRight

        # 推力大小限幅
        self.rpmLeftSP = clip(self.rpmLeftSP, -self.__rpmMax, self.__rpmMax)
        self.rpmRightSP = clip(self.rpmRightSP, -self.__rpmMax, self.__rpmMax)

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

        self.__lThrustPublisher.publish(lT)
        self.__rThrustPublisher.publish(rT)
        self.__lAnglePublisher.publish(lA)
        self.__rAnglePublisher.publish(rA)

        return
    
    def lThrustCallback(self, msg):
        self.rpmLeftEst = msg.data
        return
    
    def rThrustCallback(self, msg):
        self.rpmRightEst = msg.data
        return
    
    def lAngleCallback(self, msg):
        self.angleLeftEst = deg2rad(-msg.data)
        return
    
    def rAngleCallback(self, msg):
        self.angleRightEst = deg2rad(-msg.data)
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
    usvControl = Control()

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
