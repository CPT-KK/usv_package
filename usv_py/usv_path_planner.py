#!/usr/bin/env python3

from numpy import pi
from numpy import vstack
from numpy import linspace
from numpy.linalg import norm
from numpy import round, delete
from numpy import sin, cos, tan, arcsin, arccos, arctan, arctan2
from usv_math import wrapTo2Pi

def planLinePath(startX, startY, endX, endY, ds):
    # 规划直线
    if (ds < 0) :
        raise ValueError("Input distance between points is less or equal zero.")
    else:
        pNum = int(round(norm([startX - endX, startY - endY]) / ds))

    if (pNum <= 10) :
        pNum = 10

    path = vstack((linspace(startX, endX, pNum), linspace(startY, endY, pNum))).T

    return path

def planCirclePath(cirX, cirY, cirR, startAngle, endAngle, ds):
    # 规划圆
    if (startAngle > endAngle):
        endAngle = endAngle + 2 * pi

    if (ds < 0):
        raise ValueError("Input distance between points is less or equal zero.")
    else:
        pNum = int(round(abs(endAngle - startAngle) * cirR / ds))
    
    if (pNum <= 16):
        pNum = 16
    
    angle = linspace(startAngle, endAngle, pNum)
    path = vstack((cirX + cirR * cos(angle), cirY + cirR * sin(angle))).T

    return path

class PathPlanner:
    r = 12
    l = 12
    d = 5
    R = 0
    theta = 0

    ds = 1.0

    def __init__(self):
        self.R = self.r + norm([self.r - self.d, self.l])
        self.theta = arctan((self.r - self.d) / self.l)

    def planPursue(self, usvX, usvY, tvEstX, tvEstY):
        return planLinePath(usvX, usvY, tvEstX, tvEstY, self.ds)
    
    def planDockApproach(self, usvX, usvY, tvX, tvY):
        usvTVAngle = arctan2(usvY - tvY, usvX - tvX)
        usvTVDist = norm([usvX - tvX, usvY - tvY])

        tangentPointAngle = 0.0
        if (self.R <= usvTVDist):
            tangentPointAngle = usvTVAngle + arccos(self.R / usvTVDist)  

        tangentPointX = tvX + self.R * cos(tangentPointAngle)
        tangentPointY = tvY + self.R * sin(tangentPointAngle)

        return planLinePath(usvX, usvY, tangentPointX, tangentPointY, self.ds)

    def planDockMeasure(self, usvX, usvY, tvX, tvY):
        usvTVAngle = arctan2(usvY - tvY, usvX - tvX)
        return planCirclePath(tvX, tvY, self.R, usvTVAngle, usvTVAngle + 1.25 * pi, self.ds)

    def planDockTransfer(self, usvX, usvY, tvX, tvY, tvAngle):
        usvTVAngle = wrapTo2Pi(arctan2(usvY - tvY, usvX - tvX))
        phi1 = wrapTo2Pi(tvAngle - self.theta)
        phi2 = wrapTo2Pi(wrapTo2Pi(tvAngle - self.theta) + pi)

        flag = 0
        if (phi1 < phi2) & (usvTVAngle > phi1) & (usvTVAngle <= phi2):
            flag = 2
        elif (phi1 < phi2):
            flag = 1
        elif (phi1 > phi2) & (usvTVAngle > phi2) & (usvTVAngle <= phi1):
            flag = 1
        else:
            flag = 2
        
        if (flag == 2):
            # 走第二个变轨点，方位角 tvAngle - theta + pi
            path0 = planCirclePath(tvX, tvY, self.R, usvTVAngle, phi2, self.ds)

            cirXTransfer = tvX + (self.R - self.r) * cos(tvAngle - self.theta + pi)
            cirYTransfer = tvY + (self.R - self.r) * sin(tvAngle - self.theta + pi)
            path1 = planCirclePath(cirXTransfer, cirYTransfer, self.r, tvAngle - self.theta + pi, tvAngle + 1.5*pi, self.ds)
            
            l1X = cirXTransfer + self.r * cos(tvAngle + 1.5*pi)
            l1Y = cirYTransfer + self.r * sin(tvAngle + 1.5*pi)
            l2X = l1X - 2.0 * self.l * cos(tvAngle + 1.5*pi - 0.5*pi)
            l2Y = l1Y - 2.0 * self.l * sin(tvAngle + 1.5*pi - 0.5*pi)
            path2 = planLinePath(l1X, l1Y, l2X, l2Y, self.ds)

        else:
            # 走第一个变轨点，方位角 tvAngle - theta
            path0 = planCirclePath(tvX, tvY, self.R, usvTVAngle, phi1, self.ds)

            cirXTransfer = tvX + (self.R - self.r) * cos(tvAngle - self.theta)
            cirYTransfer = tvY + (self.R - self.r) * sin(tvAngle - self.theta)
            path1 = planCirclePath(cirXTransfer, cirYTransfer, self.r, tvAngle - self.theta, tvAngle + 0.5*pi, self.ds)
        
            l1X = cirXTransfer + self.r * cos(tvAngle + 0.5*pi)
            l1Y = cirYTransfer + self.r * sin(tvAngle + 0.5*pi)
            l2X = l1X - 2.0 * self.l * cos(tvAngle + 0.5*pi - 0.5*pi)
            l2Y = l1Y - 2.0 * self.l * sin(tvAngle + 0.5*pi - 0.5*pi)
            path2 = planLinePath(l1X, l1Y, l2X, l2Y, self.ds)

        # 返回时，去除 path0 和 path1 最后一个点，避免造成路径点重复
        return vstack((path0[:-1], path1[:-1], path2))

    def planDockTransfer2(self, usvX, usvY, tvX, tvY, tvAngle):
        usvTVAngle = wrapTo2Pi(arctan2(usvY - tvY, usvX - tvX))
        phi1 = wrapTo2Pi(tvAngle - self.theta)
        phi2 = wrapTo2Pi(wrapTo2Pi(tvAngle - self.theta) + pi)

        flag = 0
        if (phi1 < phi2) & (usvTVAngle > phi1) & (usvTVAngle <= phi2):
            flag = 2
        elif (phi1 < phi2):
            flag = 1
        elif (phi1 > phi2) & (usvTVAngle > phi2) & (usvTVAngle <= phi1):
            flag = 1
        else:
            flag = 2
        
        if (flag == 2):
            # 走第二个变轨点，方位角 tvAngle - theta + pi
            path0 = planCirclePath(tvX, tvY, self.R, usvTVAngle, phi2, self.ds)

            cirXTransfer = tvX + (self.R - self.r) * cos(tvAngle - self.theta + pi)
            cirYTransfer = tvY + (self.R - self.r) * sin(tvAngle - self.theta + pi)
            path1 = planCirclePath(cirXTransfer, cirYTransfer, self.r, tvAngle - self.theta + pi, tvAngle + 1.5*pi, self.ds)
            
            l1X = cirXTransfer + self.r * cos(tvAngle + 1.5*pi)
            l1Y = cirYTransfer + self.r * sin(tvAngle + 1.5*pi)
            l2X = l1X - 2.0 * self.l * cos(tvAngle + 1.5*pi - 0.5*pi)
            l2Y = l1Y - 2.0 * self.l * sin(tvAngle + 1.5*pi - 0.5*pi)
            path2 = planLinePath(l1X, l1Y, l2X, l2Y, self.ds)

        else:
            # 走第一个变轨点，方位角 tvAngle - theta
            path0 = planCirclePath(tvX, tvY, self.R, usvTVAngle, phi1, self.ds)

            cirXTransfer = tvX + (self.R - self.r) * cos(tvAngle - self.theta)
            cirYTransfer = tvY + (self.R - self.r) * sin(tvAngle - self.theta)
            path1 = planCirclePath(cirXTransfer, cirYTransfer, self.r, tvAngle - self.theta, tvAngle + 0.5*pi, self.ds)
        
            l1X = cirXTransfer + self.r * cos(tvAngle + 0.5*pi)
            l1Y = cirYTransfer + self.r * sin(tvAngle + 0.5*pi)
            l2X = l1X - 2.0 * self.l * cos(tvAngle + 0.5*pi - 0.5*pi)
            l2Y = l1Y - 2.0 * self.l * sin(tvAngle + 0.5*pi - 0.5*pi)
            path2 = planLinePath(l1X, l1Y, l2X, l2Y, self.ds)

        # 返回时，去除 path0 和 path1 最后一个点，避免造成路径点重复
        return vstack((path0[:-1], path1[:-1], path2))