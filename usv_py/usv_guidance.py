from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2, rad2deg, deg2rad
from numpy import clip, size, zeros, array
from numpy.linalg import norm
from usv_math import rotationZ, wrapToPi


class Guidance(Node):
    uSPMax = 3.0
    dist2NextMax = 30
    dist2NextMin = 8
    vel2NextMax = 1.5
    vel2NextMin = 0.1

    psiSP = 0.0
    uSP = 0.0
    vSP = 0.0
    delta = 3.5 * 6.0

    path = zeros((2000, 2))
    currentIdx = 0
    endIdx = 0
    pathSize = 2000
    isPathInit = False

    def __init__(self):
        super().__init__('usv_guidance_node')
        self.publisher_ = self.create_publisher(PointStamped, "/usv/guidance/guidanceSP", 10)

    def setPath(self, inPath, inEndIdx=0):
        # 设置路径
        self.path = inPath
        self.pathSize = size(inPath, 0)

        # 设置索引
        self.currentIdx = 0
        if inEndIdx <= 0:
            self.endIdx = self.pathSize - 1
        else:
            self.endIdx = inEndIdx

        self.isPathInit = True


    def guidance(self, uSP, dist2Next, x, y, vx, vy, psi):
        if (self.isPathInit == False):
            return [None, None]
        
        if (self.currentIdx >= self.endIdx):
            return [None, None]
        
        # 限幅
        dist2Next = clip(dist2Next, self.dist2NextMin, self.dist2NextMax)
        uSP = clip(uSP, - self.uSPMax, self.uSPMax)

        # 更新当前的跟踪点
        while (norm(self.path[self.currentIdx, :] - [x, y]) <= dist2Next) & (self.currentIdx < self.endIdx):
            self.currentIdx = self.currentIdx + 1
        xSP = self.path[self.currentIdx, 0]
        ySP = self.path[self.currentIdx, 1]

        # 求船体系下速度与侧滑角 beta
        [u, v] = rotationZ(vx, vy, psi)
        if (norm([vx, vy]) < 0.1):
            beta = 0
        else:
            beta = arctan2(v, u)

        # 求跟踪点的切线角
        if (self.currentIdx == self.endIdx):
            tanAngle = arctan2(ySP - self.path[self.currentIdx - 1, 1], xSP - self.path[self.currentIdx - 1, 0])
        else :
            tanAngle = arctan2(self.path[self.currentIdx + 1, 1] - ySP, self.path[self.currentIdx + 1, 0] - xSP)

        # 计算无人船相对于当前跟踪点切线的垂直方向的误差
        yErr = -(x - xSP) * sin(tanAngle) + (y - ySP) * cos(tanAngle)

        # 计算期望的朝向角
        psiSP = tanAngle - beta + arctan2(-yErr, self.delta)

        # Debug 用输出
        # print("=============================================================================")
        # print("Guidance 输出: 更新半径 = %.2f" % dist2Next)
        # print("此段路径当前跟踪点: No.%d, [%.2f, %.2f]. 此段路径终点: No.%d, [%.2f, %.2f]." % (self.currentIdx, xSP, ySP, self.endIdx, self.path[self.endIdx, 0], self.path[self.endIdx, 1]))
        # print("USV 船体系速度 u: %.2f, v: %.2f，合速度: %.2f." % (u, v, norm((u,v))))
        # print("theta: %5.2f, beta: %5.2f, yErr: %5.2f" % (tanAngle, beta, yErr))

        # ROS2 内发送制导指令
        self.pubSetpoints(xSP, ySP, psiSP)

        # 返回制导指令
        return [uSP, psiSP]

    def guidanceVec(self, dist2Next, vel2Next, x, y, vx, vy, psi):
        if (self.isPathInit == False):
            return [None, None, None]
        
        if (self.currentIdx >= self.endIdx):
            return [None, None, None]
        
        # 限幅
        dist2Next = clip(dist2Next, self.dist2NextMin, self.dist2NextMax)
        vel2Next = clip(vel2Next, - self.vel2NextMax, self.vel2NextMin)

        # 更新当前的跟踪点
        while (norm(self.path[self.currentIdx, :] - array([[x, y]])) <= dist2Next) & (self.currentIdx < self.endIdx):
            self.currentIdx = self.currentIdx + 1
        xSP = self.path[self.currentIdx, 0]
        ySP = self.path[self.currentIdx, 1]

        # 求跟踪点的切线角
        if (self.currentIdx == self.endIdx):
            tanAngle = arctan2(ySP - self.path[self.endIdx - 1, 1], xSP - self.path[self.endIdx - 1, 0])
        else :
            tanAngle = arctan2(self.path[self.currentIdx + 1, 1] - ySP, self.path[self.currentIdx + 1, 0] - xSP)

        # 计算无人船相对于当前跟踪点切线的垂直方向的误差
        yErr = -(x - xSP) * sin(tanAngle) + (y - ySP) * cos(tanAngle)

        # 计算期望的朝向角
        psiSP = wrapToPi(tanAngle + arctan2(-yErr, self.delta))

        # Debug 用输出
        # print("=============================================================================")
        # print("Guidance 输出:")
        # print("此段路径当前跟踪点: No.%d, [%.2f, %.2f]. 此段路径终点: No.%d, [%.2f, %.2f]." % (self.currentIdx, xSP, ySP, self.endIdx, self.path[self.endIdx, 0], self.path[self.endIdx, 1]))
        # print("USV 船体系速度 u: %.2f, v: %.2f，合速度: %.2f." % (u, v, norm((u,v))))
        # print("期望 psi: %.2f" % rad2deg(psiSP))

        # ROS2 内发送制导指令
        self.pubSetpoints(xSP, ySP, psiSP)

        # 返回制导指令
        return [xSP, ySP, psiSP]

    def pubSetpoints(self, xSP, ySP, psiSP):
        msg = PointStamped()
        msg.point.x = xSP
        msg.point.y = ySP
        msg.point.z = psiSP

        msg.header.frame_id = "usv_enu_frame"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
