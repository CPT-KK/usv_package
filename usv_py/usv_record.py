#!/usr/bin/env python3
import time

from numpy import zeros, rad2deg, median, deg2rad, sin, cos, pi, abs, min, argmin

from rich.table import Column, Table
from rich import box

def genTable(usvState, latestMsg, usvPose, usvComm, dt, uSP):    
    if (usvComm.isSearchFindTV):
        sUAVOutput = rad2deg(usvComm.tvAngleEst)
    else:
        sUAVOutput = float("nan")

    if (usvPose.isPodFindTV):
        podOutput = rad2deg(usvPose.tvAnglePod)
    else:
        podOutput = float("nan")

    if (usvPose.isLidarFindTV):
        lidarOutPutDist = usvPose.tvDist
        lidarOutPutAngle = rad2deg(usvPose.tvAngleLidar)
        xLidarOutput = usvPose.xLidar
        yLidarOutput = usvPose.yLidar
    else:
        lidarOutPutDist = float("nan")
        lidarOutPutAngle = float("nan")
        xLidarOutput = float("nan")
        yLidarOutput = float("nan")

    theTable = Table(show_header=True, header_style="bold", title_justify="center", title_style="bold magenta", caption_justify="left", box=box.HORIZONTALS)
    theTable.title = "USV Info @ t = %.3f s" % dt
    theTable.caption = " Message: " + latestMsg + "\n"
    theTable.add_column("Stage: [reverse]%s" % usvState, justify="left")
    theTable.add_column("Current state", justify="left")
    theTable.add_column("Input command", justify="left")
    theTable.add_column("Other", justify="left")
    theTable.add_row(
        "t: %.3f s" % dt,
        "",
        "uSP: %.2f m/s" % uSP,
        "",
    )
    theTable.add_row(
        "GPS: %d" % usvPose.isGPSValid, 
        "u: %.2f m/s" % usvPose.uDVL,
        "sUAV yaw: %.2f deg" % sUAVOutput,
        "x(GPS): %.2f m" % usvPose.x,
    )
    theTable.add_row(
        "Imu: %d" % usvPose.isImuValid, 
        "v: %.2f m/s" % usvPose.vDVL,
        "pod yaw: %.2f deg" % podOutput,
        "y(GPS): %.2f m" % usvPose.y,
    )
    theTable.add_row(
        "Dvl: %d" % usvPose.isDvlValid, 
        "psi: %.2f deg" % rad2deg(usvPose.psi),
        "Lidar yaw: %.2f deg" % lidarOutPutAngle,
        "x(Lidar): %.2f m" % xLidarOutput,
    )
    theTable.add_row(
        "Pod: %d" % usvPose.isPodValid, 
        "r: %.2f deg/s" % rad2deg(usvPose.r),
        "Lidar dist: %.2f m" % lidarOutPutDist,
        "y(Lidar): %.2f m" % yLidarOutput,
    )

    return theTable

class USVData():
    element = ["t", "x_GPS", "y_GPS", "psi", "u", "v", "r", "uSP", "vSP", "psiSP","x_DVL", "y_DVL", "u_DVL", "v_DVL", "ax", "ay", "az", "roll", "pitch", "x_Lidar", "y_Lidar","search_angle", "pod_angle", "lidar_angle"]
    elementStr = " ".join(element)
    elementTemplate = "%.5f " * (len(element) - 1) + "%.5f"

    def __init__(self, rate):
        timeStr = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.fileNameStr = "usv_data_" + timeStr + ".txt"
        with open(self.fileNameStr, 'w') as f:
            f.write(self.elementStr + '\n')

        self.saveDataTimer = 1
        self.recordRate = rate

    def saveData(self, dt, usvPose, usvComm, uSP, vSP, psiSP):       
        with open(self.fileNameStr, 'a') as f:          
            if (self.saveDataTimer >= 0.5 * self.recordRate):
                f.write(self.elementTemplate % (dt, usvPose.x, usvPose.y, usvPose.psi, usvPose.u, usvPose.v, usvPose.r, uSP, vSP, psiSP, usvPose.xDVL, usvPose.yDVL, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.azb, usvPose.roll, usvPose.pitch, usvPose.xLidar, usvPose.yLidar, usvComm.tvAngleEst, usvPose.tvAnglePod, usvPose.tvAngleLidar) + '\n')

                self.saveDataTimer = 1
            else:
                self.saveDataTimer = self.saveDataTimer + 1

