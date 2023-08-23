#!/usr/bin/env python3

class PID:
    Kp = 1.0
    Ki = 0.0
    Kd = 0.0

    controlFrequency = 0.0

    t = 0.0
    tLast = 0.0

    errIntResult = 0.0
    errLast = 0.0

    def __init__(self, inP, inI, inD, inFreq):
        self.Kp = inP
        self.Ki = inI
        self.Kd = inD
        
        if (inFreq <= 0.0):
            raise ValueError("PID frequency must greater than zero")
        else:
            self.controlFrequency = inFreq

        
    def compute(self, errValue, errDiffValue=None):
        if errDiffValue == None:
            out = self.Kp * errValue + self.Ki * (self.errIntResult + errValue / self.controlFrequency) + self.Kd * (errValue - self.errLast) * self.controlFrequency   
        else:
            out = self.Kp * errValue + self.Ki * (self.errIntResult + errValue / self.controlFrequency) + self.Kd * errDiffValue * self.controlFrequency

        self.errIntResult = self.errIntResult + errValue / self.controlFrequency
        self.errLast = errValue

        return out
    
    def clearIntResult(self):
        self.errIntResult = 0

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
