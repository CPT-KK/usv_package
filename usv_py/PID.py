#!/usr/bin/env python3

class PID:

    def __init__(self, kp, ki, kd, frequency, intMax=None, intMin=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        if (frequency <= 0.0):
            raise ValueError("PID frequency must greater than zero")
        else:
            self.frequency = frequency

        self.intMax = intMax
        self.intMin = intMin      
        self.errIntegral = 0.0
        self.errPrevious = 0.0

    def compute(self, err, errDiff=None):
        # 计算误差积分项
        self.errIntegral = self.errIntegral + err / self.frequency

        # 防止积分项饱和
        if (self.intMax is not None and self.errIntegral > self.intMax):
            self.errIntegral = self.intMax
        elif (self.intMin is not None and self.errIntegral < self.intMin):
            self.errIntegral = self.intMin

        # 如果有物理上的微分项输入，则使用物理上的微分项，否则使用数值微分项
        if (errDiff is None):
            output = self.kp * err + self.ki * (self.errIntegral + err / self.frequency) + self.kd * (err - self.errPrevious) * self.frequency   
        else:
            output = self.kp * err + self.ki * (self.errIntegral + err / self.frequency) + self.kd * errDiff * self.frequency

        # 保存这一时刻的误差
        self.errPrevious = err

        return output
    
    def clearIntResult(self):
        self.errIntegral = 0

# class PID:
#     def __init__(self, P=0.2, I=0.0, D=0.0):
#         self.kp = P
#         self.ki = I
#         self.kd = D
#         self.uPrevious = 0
#         self.uCurent = 0
#         self.setValue = 0
#         self.lastErr = 0
#         self.preLastErr = 0
#         self.errSum = 0
#         self.errSumLimit = 10
    
#     # 位置式PID
#     def pidPosition(self, curValue):
#         err = self.setValue - curValue
#         dErr = err - self.lastErr
#         self.preLastErr = self.lastErr
#         self.lastErr = err
#         self.errSum += err
#         outPID = self.kp * err + (self.ki * self.errSum) + (self.kd * dErr)
#         return outPID

#     # 增量式PID
#     def pidIncrease(self, curValue):
#         self.uCurent = self.pidPosition(curValue)
#         outPID = self.uCurent - self.uPrevious
#         self.uPrevious = self.uCurent
#         return outPID
