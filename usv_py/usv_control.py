#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16, Float32MultiArray

from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2, rad2deg, deg2rad, clip, abs, sign, pi, sqrt, array, max
from numpy.linalg import norm

from usv_math import rotationZ, wrapToPi

from PID import PID

class Control():
     
    # 控制器限幅参数
    __AXB_SP_MAX = 2.0        # 控制器中 axbSP 的最大值
    __AYB_SP_MAX = 2.0        # 矢量控制器中 aybSP 的最大值

    __U_SP_MAX = 4.0          # 差动控制器中 uSP 的最大值
    __U_SP_VEC_MAX = 1.0       # 矢量控制器中 uSP 的最大值
    __V_SP_MAX = 1.2          # 矢量控制器中 vSP 的最大值
    __R_SP_MAX = deg2rad(8)   # 控制器中 rSP 的最大值
    
    # 矢量控制器所处状态
    vecCtrlState = 0        # 0: 正在修正轴向和航向的误差；1: 正在修正侧向的误差

    # USV RPM and pod angle limits
    __RPM_MIN = 35                   # 最小转速
    __RPM_MAX = 1000                 # 最大转速
    __RPM_ROTATE_MAX = 800            # 用于转动的最大转速
    __ANGLE_MAX = deg2rad(136)        # 舵角最大值
    __ANGLE_MIN = deg2rad(-44)
    __ANGLE_MAX_STATE0 = deg2rad(25)  # Control state 0 下的舵角最大值

    __RPM_RATE_MAX = 70

    # 无人船参数
    __MASS = 800.0          # 质量
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
    statusLeft = 0
    statusRight = 0
    driveReadyLeft = 0
    driveReadyRight = 0
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
        self.__lStatusSubscriber = rospy.Subscriber("/usv/torqeedo/left/status", Int16, self.lStatusCallback, queue_size=1)
        self.__rStatusSubscriber = rospy.Subscriber("/usv/torqeedo/right/status", Int16, self.rStatusCallback, queue_size=1)
        self.__lAngleSubscriber = rospy.Subscriber("/usv/pod/left/estimate", Float32, self.lAngleCallback, queue_size=1)
        self.__rAngleSubscriber = rospy.Subscriber("/usv/pod/right/estimate", Float32, self.rAngleCallback, queue_size=1)

        # For USV battery info
        self.__battSOCSubscriber = rospy.Subscriber("/usv/battery/soc", Float32MultiArray, self.battSOCCallback, queue_size=1)
        self.__battCellVoltMinSubscriber = rospy.Subscriber("/usv/battery/min_cell_volt", Float32MultiArray, self.battCellVoltMinCallback, queue_size=1)

        # PID 初始化
        self.__uPID = PID(0.8, 0.06, -0.012, 0.8, -0.8)
        self.__yawPID = PID(0.25, 0.0002, -0.02, 0.5236, -0.5236)
        self.__rPID = PID(14, 0.5, -0.1, 0.05236, -0.05236)

        self.__xPID = PID(0.2, 0.00, 0.0)
        self.__yPID = PID(0.3, 0.00, 0.0)
        self.__vxPID = PID(0.8, 0.00, 0.0)
        self.__vyPID = PID(1.5, 0.00, 0.0)

    def __del__(self):
        pass

    def moveUSV(self, uSP, yawSP, u, axb, yaw, r):
        # 计算朝向角误差
        yawSP = wrapToPi(yawSP)
        yaw = wrapToPi(yaw)
        yawErr = wrapToPi(yawSP) - yaw

        # 朝向角误差限幅
        yawErr = wrapToPi(yawErr)

        # 根据 yawErr 的值，计算可行的 uSP (避免速度太大，转弯转不过来)
        if (abs(yawErr) > pi / 4):
            uSP = 0.1 * uSP
        else:
            uSP = uSP * (0.1 + 0.9 * (1 - abs(yawErr) / (pi / 4)))  

        # 轴向速度限幅
        uSP = clip(uSP, -self.__U_SP_MAX, self.__U_SP_MAX)

        # 计算轴向速度误差
        uErr = uSP - u

        # 计算推力大小
        axbSP = self.__uPID.compute(uErr, axb)

        # 加速度 setpoints 限幅
        axbSP = clip(axbSP, -self.__AXB_SP_MAX, self.__AXB_SP_MAX)

        # 计算期望朝向角速度
        rSP = self.__yawPID.compute(yawErr, r)

        # 期望朝向角速度限幅
        rSP = clip(rSP, -self.__R_SP_MAX, self.__R_SP_MAX)

        # 计算期望角速度误差
        rErr = rSP - r

        # 计算所需角加速度大小
        etaSP = self.__rPID.compute(rErr)

        # 混控计算
        self.mixer(axbSP, 0, etaSP)

        # 发布推力
        self.thrustPub()

        return [uSP, rSP, axbSP, etaSP]

    def moveUSVVec(self, xSP, ySP, yawSP, x, y, u, v, axb, ayb, yaw, r):
        # 计算 x y 误差
        xErr = xSP - x
        yErr = ySP - y
        yawSP = wrapToPi(yawSP)
        yaw = wrapToPi(yaw)
        yawErr = wrapToPi(yawSP - yaw)
        
        # x y 误差转船体系
        [xErr, yErr] = rotationZ(xErr, yErr, yaw)

        # 选择矢量控制器状态
        if ((abs(xErr) > 2.0 or (sign(u * xErr) > 0 and abs(u) > 0.4) or abs(yawErr) > deg2rad(15)) and (self.vecCtrlState == 1)):
            self.vecCtrlState = 0
        elif (abs(xErr) <= 1.0 and abs(yawErr) <= deg2rad(10) and self.vecCtrlState == 0):
            self.vecCtrlState = 1

        uSP = 0.0
        vSP = 0.0
        rSP = 0.0
        axbSP = 0.0
        aybSP = 0.0
        etaSP = 0.0
        if (self.vecCtrlState == 1):
            # 计算并修正轴向误差
            uSP = self.__xPID.compute(xErr, u)
            uSP = clip(uSP, -self.__U_SP_VEC_MAX, self.__U_SP_VEC_MAX)
            uErr = uSP - u
            axbSP = self.__vxPID.compute(uErr, axb)
            axbSP = clip(axbSP, -self.__AXB_SP_MAX, self.__AXB_SP_MAX)

            # 计算并修正侧向误差
            vSP = self.__yPID.compute(yErr, v)
            vSP = clip(vSP, -self.__V_SP_MAX, self.__V_SP_MAX)
            vErr = vSP - v
            aybSP = self.__vyPID.compute(vErr, ayb)
            aybSP = clip(aybSP, -self.__AYB_SP_MAX, self.__AYB_SP_MAX)

            # 限制 axbSP 不能负的太多
            if (arctan(aybSP / axbSP) < self.__ANGLE_MIN):
                axbSP = aybSP / tan(self.__ANGLE_MAX)
        else:
            # 计算并修正轴向误差
            uSP = self.__xPID.compute(xErr, u)
            uSP = clip(uSP, -self.__U_SP_VEC_MAX, self.__U_SP_VEC_MAX)
            uErr = uSP - u
            axbSP = self.__vxPID.compute(uErr, axb)
            axbSP = clip(axbSP, -self.__AXB_SP_MAX, self.__AXB_SP_MAX)

            # 计算并修正侧向误差
            vSP = self.__yPID.compute(yErr, v)
            vSP = clip(vSP, -self.__V_SP_MAX, self.__V_SP_MAX)
            vErr = vSP - v
            aybSP = self.__vyPID.compute(vErr, ayb)
            aybSP = clip(aybSP, -self.__AYB_SP_MAX, self.__AYB_SP_MAX)

            # 计算并修正航向误差
            rSP = self.__yawPID.compute(yawErr, r)
            rSP = clip(rSP, -self.__R_SP_MAX, self.__R_SP_MAX)
            rErr = rSP - r
            etaSP = self.__rPID.compute(rErr)

            # 侧向加速度要被轴向加速度限幅
            aybSP = clip(aybSP, -abs(axbSP) * tan(self.__ANGLE_MAX_STATE0), abs(axbSP) * tan(self.__ANGLE_MAX_STATE0))
            
        # 送入混控
        self.mixer(axbSP, aybSP, etaSP)

        # 发布推力
        self.thrustPub()

        return [uSP, vSP, rSP, axbSP, aybSP, etaSP]

    def mixer(self, axSP, aySP, etaSP):
        if (abs(axSP) < 1e-6) & (abs(aySP) < 1e-6):
            self.angleLeftSP = 0.0
            self.angleRightSP = 0.0
        else:    
            self.angleLeftSP = arctan(aySP / axSP)
            self.angleRightSP = arctan(aySP / axSP)

        # 这里假设 angleLeftSP 在 [-pi/2, pi/2] 之间
        if (self.angleLeftSP < self.__ANGLE_MIN):
            self.angleLeftSP = self.angleLeftSP + pi

        if (self.angleRightSP < self.__ANGLE_MIN):
            self.angleRightSP = self.angleRightSP + pi

        # 推力偏角限幅
        self.angleLeftSP = clip(self.angleLeftSP, self.__ANGLE_MIN, self.__ANGLE_MAX)
        self.angleRightSP = clip(self.angleRightSP, self.__ANGLE_MIN, self.__ANGLE_MAX)

        # 现在，电机角度确定，需要确定推力大小
        # 计算一侧发动机平动和转动所需推力大小的参考值
        # 如果发动机推力方向与 atan2(aySP, axSP) 的方向相同，那么平动推力是正的
        # 如果发动机推力方向与 atan2(aySP, axSP) 的方向相反，那么平动推力是负的
        # assert self.angleLeftSP = self.angleRightSP!!!!
        # sign(self.angleLeftSP) * sign(atan2(aySP, axSP)) == 1
        # sign(self.angleLeftSP) * sign(atan2(aySP, axSP)) == -1
        # sign(self.angleLeftSP) * sign(atan2(aySP, axSP)) == 0 <-> aySP == 0
        if (sign(self.angleLeftSP) * sign(arctan2(aySP, axSP)) == 0):
            rpmTranslate = self.__MASS * sign(axSP) * sqrt(axSP ** 2) / 2.0
        elif (sign(self.angleLeftSP) * sign(arctan2(aySP, axSP)) > 0):
            rpmTranslate = self.__MASS * sqrt(axSP ** 2 + aySP ** 2) / 2.0
        else:
            rpmTranslate = -self.__MASS * sqrt(axSP ** 2 + aySP ** 2) / 2.0

        rpmRotate = self.__INERZ * etaSP / self.__TORQLEN / 2.0
        
        # 计算左右两侧平动所需推力
        # 当推力角的当前值和期望值相差较大时，实际平动推力变小
        rpmTranslateLeft = rpmTranslate * cos(self.angleLeftSP - self.angleLeftEst)
        rpmTranslateRight = rpmTranslate * cos(self.angleRightSP - self.angleRightEst)
        
        # 计算左右两侧转动所需推力
        # 为了避免除以 cos(self.angleLeftEst) 出现除以 0 导致结果为无穷的情况，加一个限幅
        rpmRotateLeft = clip(rpmRotate / cos(self.angleLeftEst), -self.__RPM_ROTATE_MAX, self.__RPM_ROTATE_MAX)
        rpmRotateRight = clip(rpmRotate / cos(self.angleRightEst), -self.__RPM_ROTATE_MAX, self.__RPM_ROTATE_MAX)

        # 计算左右两侧真实推力设置值的大小
        rpmLeftErr = rpmTranslateLeft - rpmRotateLeft - self.rpmLeftSP
        rpmRightErr = rpmTranslateRight + rpmRotateRight - self.rpmRightSP
        if (abs(rpmLeftErr) > self.__RPM_RATE_MAX):
            self.rpmLeftSP = self.rpmLeftSP + sign(rpmLeftErr) * self.__RPM_RATE_MAX
        else:
            self.rpmLeftSP = self.rpmLeftSP + rpmLeftErr
        
        if (abs(rpmRightErr) > self.__RPM_RATE_MAX):
            self.rpmRightSP = self.rpmRightSP + sign(rpmRightErr) * self.__RPM_RATE_MAX
        else:
            self.rpmRightSP = self.rpmRightSP + rpmRightErr         

        # 推力大小限幅
        self.rpmLeftSP = clip(self.rpmLeftSP, -self.__RPM_MAX, self.__RPM_MAX)
        self.rpmRightSP = clip(self.rpmRightSP, -self.__RPM_MAX, self.__RPM_MAX)

        return
    
    def thrustSet(self, rpmLeft, rpmRight, angleLeft, angleRight):
        self.rpmLeftSP = rpmLeft
        self.rpmRightSP = rpmRight
        self.angleLeftSP = angleLeft
        self.angleRightSP = angleRight

        return

    def thrustPub(self):
        if (self.driveReadyLeft == 0) | (self.driveReadyRight == 0):
            rospy.logerr(f"Motor[{self.driveReadyLeft}, {self.driveReadyRight}] not ready!")
            lT = Int16(data=0)
            rT = Int16(data=0)
            lA = Float32(data=0)
            rA = Float32(data=0)
        else:
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
    
    def lStatusCallback(self, msg):
        self.statusLeft = msg.data
        self.driveReadyLeft = (msg.data & 0b10000000) >> 7
        return  
    
    def rStatusCallback(self, msg):
        self.statusRight = msg.data
        self.driveReadyRight = (msg.data & 0b10000000) >> 7
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

    lastTime = 600
    loopTimes = round(lastTime / (1 / rate))

    rpmValue = 80
    angleValue = deg2rad(94.8)
    
    rospy.loginfo("Moving left torq...")
    for i in range(loopTimes):
        usvControl.thrustSet(rpmValue, rpmValue, angleValue, angleValue)
        usvControl.thrustPub()
        rosRate.sleep()

    # rospy.loginfo("Moving right torq...")
    # for i in range(loopTimes):
    #     usvControl.thrustSet(rpmValue, rpmValue, angleValue, angleValue)
    #     usvControl.thrustPub()
    #     rosRate.sleep()

    # rospy.loginfo("Moving left angle...")
    # for i in range(loopTimes):
    #     usvControl.thrustSet(0, 0, angleValue, 0)
    #     usvControl.thrustPub()
    #     rosRate.sleep()

    # rospy.loginfo("Moving right angle...")
    # for i in range(loopTimes):
    #     usvControl.thrustSet(0, 0, angleValue, angleValue)
    #     usvControl.thrustPub()
    #     rosRate.sleep()

    rospy.loginfo("Stopping...")
    for i in range(loopTimes):
        usvControl.thrustSet(0, 0, 0, 0)
        usvControl.thrustPub()
        rosRate.sleep()
