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