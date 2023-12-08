#!/usr/bin/env python3
import rospy

class PID:

    def __init__(self, kp, ki, kd, intMax=None, intMin=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.__intMax = intMax
        self.__intMin = intMin      
        self.__errIntegral = 0.0
        self.__errPrevious = 0.0
        self.__timePrevious = None # unit: sec

    def compute(self, errProportion, errPhysicalDiff=None):
        
        # 计算时间差
        if (self.__timePrevious is None):         
            dt = 0.0   
        else:
            dt = rospy.Time.now().to_sec() - self.__timePrevious
        
        # 计算误差积分项 self.__errIntegral
        self.__errIntegral = self.__errIntegral + errProportion * dt

        # 防止积分项饱和
        if (self.__intMax is not None and self.__errIntegral > self.__intMax):
            self.__errIntegral = self.__intMax
        elif (self.__intMin is not None and self.__errIntegral < self.__intMin):
            self.__errIntegral = self.__intMin

        # 计算误差微分项 errDiff
        if (dt != 0.0):
            # 如果有物理上的微分项输入，则使用物理上的微分项，否则使用数值微分项
            if (errPhysicalDiff is None):
                errDiff = (errProportion - self.__errPrevious) / dt
            else:
                errDiff = errPhysicalDiff / dt
        else:
            errDiff = 0.0
        
        # PID 计算
        output = self.kp * errProportion + self.ki * self.__errIntegral + self.kd * errDiff

        # 保存这一时刻的误差和时间戳
        self.__errPrevious = errProportion
        self.__timePrevious = rospy.Time.now().to_sec() 

        return output

    def clearIntResult(self):
        self.__errIntegral = 0
