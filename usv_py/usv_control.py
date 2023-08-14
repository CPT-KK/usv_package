import rospy
from std_msgs.msg import Float64

from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2, rad2deg, deg2rad, clip, abs, sign
from numpy.linalg import norm

from usv_math import rotationZ, wrapToPi

from PID import PID

class Control():
     
    # 控制器限幅参数
    xErrMax = 10.0
    yErrMax = 10.0

    uSPMax = 3.0
    rSPMax = deg2rad(45)
    vxSPMax = 3
    vySPMax = 3

    rpmMax = 1200.0
    angleMax = 1.047198 # 60 deg

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
        self.lThrustPublisher_ = rospy.Publisher("/usv/left/thrust/cmd_thrust", Float64, queue_size=10)
        self.rThrustPublisher_ = rospy.Publisher("/usv/right/thrust/cmd_thrust", Float64, queue_size=10)
        self.lAnglePublisher_ = rospy.Publisher("/usv/left/thrust/joint/cmd_pos", Float64, queue_size=10)
        self.rAnglePublisher_ = rospy.Publisher("/usv/right/thrust/joint/cmd_pos", Float64, queue_size=10)

        # PID 初始化
        self.uPID = PID(0.675, 0.078, 0.02, control_frequency)
        self.psiPID = PID(1.25, 0.0, 0.15, control_frequency)
        self.rPID = PID(0.03, 0.000, 0.3, control_frequency)

        self.xPID = PID(0.2, 0.000, 0.000, control_frequency)
        self.yPID = PID(0.3, 0.000, 0.000, control_frequency)
        self.vxPID = PID(0.4, 0.005, 0.01, control_frequency)
        self.vyPID = PID(0.5, 0.005, 0.01, control_frequency)

    def __del__(self):
        # 析构时，关闭无人船推力输出
        
        for i in range(20):
            self.thrustPub(0.0, 0.0, 0.0, 0.0)


    def moveUSV(self, uSP, psiSP, x, y, vx, vy, axb, ayb, psi, r):
        # 计算船体系速度
        [u, v] = rotationZ(vx, vy, psi)

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
    
    def moveUSVVec(self, xSP, ySP, psiSP, x, y, vx, vy, axb, ayb, psi, r):
        # 计算 x y 误差
        xErr = xSP - x
        yErr = ySP - y
        
        # 计算 vx vy setpoints
        vxSP = self.xPID.compute(xErr, vx)
        vySP = self.yPID.compute(yErr, vy)

        # vx vy setpoints 限幅
        vxSP = clip(vxSP, - self.vxSPMax, self.vxSPMax)
        vySP = clip(vySP, - self.vySPMax, self.vySPMax)
       
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
        angleL = clip(angleL, - self.angleMax, self.angleMax)
        angleR = clip(angleR, - self.angleMax, self.angleMax)

        # 计算推力大小
        rpmL = self.usvMass * sign(axSP) * norm([axSP, aySP]) / 2.0 - self.usvInerZ * etaSP / (2.0 * self.usvThrust2Center * cos(angleL))
        rpmR = self.usvMass * sign(axSP) * norm([axSP, aySP]) / 2.0 + self.usvInerZ * etaSP / (2.0 * self.usvThrust2Center * cos(angleR))

        # 推力大小限幅
        rpmL = clip(rpmL, - self.rpmMax, self.rpmMax)
        rpmR = clip(rpmR, - self.rpmMax, self.rpmMax)
   
        return [rpmL, rpmR, angleL, angleR]
    
    def thrustPub(self, rpmL, rpmR, angleL, angleR):
        lT = Float64(data=rpmL)
        rT = Float64(data=rpmR)
        lA = Float64(data=angleL)
        rA = Float64(data=angleR)

        self.lThrustPublisher_.publish(lT)
        self.rThrustPublisher_.publish(rT)
        self.lAnglePublisher_.publish(lA)
        self.rAnglePublisher_.publish(rA)

        return
    