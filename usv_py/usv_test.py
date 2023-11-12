from numpy import zeros, rad2deg, median, deg2rad, sin, cos, pi, abs, min, argmin, mean, tan, arctan, arctan2, std

from usv_path_planner import planCirclePath

def test(usvState, usvPathPlanner, usvGuidance, usvControl, usvPose):
    # 测试部分
    if usvState == "TEST_LINE":
        if (isTestLinePlan == False):
            endX = -50
            endY = 50
            currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
            usvGuidance.setPath(currPath)
            latestMsg = "USV 测试-直线路径已规划. 前往 [%d, %d]." % (endX, endY)
            isTestLinePlan = True       

        if (usvGuidance.currentIdx >= usvGuidance.endIdx):    
            latestMsg = "USV 测试-直线结束."
            return 1

        [psiSP, xSP, ySP] = usvGuidance.guidance(20.0, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
        uSP = usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

    elif usvState == "TEST_CIRCLE":
        if (isTestCirclePlan == False):
            R = 15
            circleTimes = 3
            cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
            cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
            currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
            usvGuidance.setPath(currPath)
            latestMsg = "USV 测试-圆路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes)
            isTestCirclePlan = True

        if (usvGuidance.currentIdx >= usvGuidance.endIdx):  
            latestMsg = "USV 测试-圆结束."
            return 1
        
        # R = 30m, dist2Next = 15m, uSP = 3m/s
        # R = 15m, dist2Next = 7m, uSP = 2.6m/s
        [psiSP, xSP, ySP] = usvGuidance.guidance(7, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
        uSP = usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

    elif usvState == "TEST_BOTH": 
        if (isTestLinePlan == False) & (isTestCirclePlan == False):
            lineLength = 200
            endX = usvPose.x + lineLength*cos(usvPose.psi);
            endY = usvPose.y + lineLength*sin(usvPose.psi);
            currPath = usvPathPlanner.planPursue(usvPose.x, usvPose.y, endX, endY)
            usvGuidance.setPath(currPath)
            latestMsg = "USV 测试-直线+圆的直线段路径已规划. 前往 [%.2f, %.2f]m." % (endX, endY)
            isTestLinePlan = True
            theSpeed = 4
            theDist2Next = 20

        if (usvGuidance.currentIdx >= usvGuidance.endIdx) & (isTestLinePlan == True) & (isTestCirclePlan == False):  
            R = 30
            circleTimes = 3
            cirCenX = usvPose.x - R * cos(usvPose.psi - pi/2)
            cirCenY = usvPose.y - R * sin(usvPose.psi - pi/2)
            currPath = planCirclePath(cirCenX, cirCenY, R, usvPose.psi - pi/2, usvPose.psi - pi/2 + circleTimes * 2 * pi, 4)
            usvGuidance.setPath(currPath)
            latestMsg = "USV 测试-直线+圆的圆段路径已规划. 圆心 [%.2f, %.2f]m. 半径 %.2fm. 环绕次数 %d." % (cirCenX, cirCenY, R, circleTimes) 
            isTestCirclePlan = True   
            theDist2Next = 16

        if (usvGuidance.currentIdx >= usvGuidance.endIdx) & (isTestCirclePlan == True) & (isTestLinePlan == True):  
            latestMsg ="USV 测试-直线+圆结束."
            return 1
        
        uSP = 3
        [psiSP, xSP, ySP] = usvGuidance.guidance(theDist2Next, usvPose.x, usvPose.y, usvPose.psi, usvPose.beta)
        uSP = usvControl.moveUSV(uSP, psiSP, usvPose.u, usvPose.axb, usvPose.psi, usvPose.r)

    return 0