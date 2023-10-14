#!/usr/bin/env python3
import rospy, time, threading

from usv_can import CAN
from usv_pose import Pose
from usv_communication import Communication

from numpy import rad2deg
from rich.table import Table
from rich import box
from rich.console import Console

def genTable(usvState, latestMsg, usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP):    
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

    if (usvPose.isLidarFindObs):
        obsOutPut = rad2deg(usvPose.obsAngleLidar)
    else:
        obsOutPut = float("nan")

    theTable = Table(show_header=True, header_style="bold", title_justify="center", title_style="bold magenta", caption_justify="left", box=box.HORIZONTALS)
    theTable.title = "USV Info @ t = %.3f s" % dt
    theTable.caption = " Message: " + latestMsg + "\n"
    theTable.add_column("Stage: [reverse]%s" % usvState, justify="left")
    theTable.add_column("Motions", justify="left")
    theTable.add_column("Setpoints", justify="left")
    theTable.add_column("Sensors", justify="left")
    theTable.add_column("Sensors", justify="left")
    theTable.add_column("Batteries", justify="left")
    theTable.add_column("Motors", justify="left")
    theTable.add_row(
        "GPS: %d" % usvPose.isGPSValid, 
        "u: %.2f m/s" % usvPose.uDVL,
        "uSP: %.2f m/s" % uSP,
        "x(GPS): %.2f m" % usvPose.x,
        "sUAV yaw: %.2f deg" % sUAVOutput,
        "1: %.1f %% | ↓%.2f v" % (usvCAN.battSOC[0], usvCAN.battCellVoltMin[0]),
        "L: %d RPM | %.2f deg" % (usvCAN.motorRPM[0], rad2deg(usvCAN.motorAngle[0])),
    )
    theTable.add_row(
        "Imu: %d" % usvPose.isImuValid, 
        "v: %.2f m/s" % usvPose.vDVL,
        "vSP: %.2f m/s" % vSP,
        "y(GPS): %.2f m" % usvPose.y,
        "Pod yaw: %.2f deg" % podOutput,
        "2: %.1f %% | ↓%.2f v" % (usvCAN.battSOC[1], usvCAN.battCellVoltMin[1]),
        "R: %d RPM | %.2f deg" % (usvCAN.motorRPM[1], rad2deg(usvCAN.motorAngle[1])),
    )
    theTable.add_row(
        "Dvl: %d" % usvPose.isDvlValid, 
        "psi: %.2f deg" % rad2deg(usvPose.psi),        
        "psiSP: %.2f deg" % rad2deg(psiSP),
        "x(Lidar): %.2f m" % xLidarOutput,
        "Lidar yaw: %.2f deg" % lidarOutPutAngle,
        "3: %.1f %% | ↓%.2f v" % (usvCAN.battSOC[2], usvCAN.battCellVoltMin[2]),
        "",
    )
    theTable.add_row(
        "Pod: %d" % usvPose.isPodValid, 
        "r: %.2f deg/s" % rad2deg(usvPose.r),
        "xSP: %.2f m" % xSP,
        "y(Lidar): %.2f m" % yLidarOutput,
        "Lidar dist: %.2f m" % lidarOutPutDist,
        "4: %.1f %% | ↓%.2f v" % (usvCAN.battSOC[3], usvCAN.battCellVoltMin[3]),
        "",
    )
    theTable.add_row(
        "Lidar: %d" % usvPose.isLidarValid,
        "",
        "ySP: %.2f m" % ySP,     
        "",
        "Obs yaw: %.2f deg" % obsOutPut,
        "",
        "",
    )

    return theTable

class USVData():
    element = ["t", "x_GPS", "y_GPS", "psi", "u", "v", "r", "uSP", "vSP", "psiSP", "xSP", "ySP", "x_DVL", "y_DVL", "u_DVL", "v_DVL", "ax", "ay", "az", "roll", "pitch", "x_Lidar", "y_Lidar","search_angle", "pod_angle", "lidar_angle", "obs_x", "obs_y", "obs_angle", "rpm_left", "rpm_right", "angle_left", "angle_right", "battery_1_SOC", "battery_2_SOC", "battery_3_SOC", "battery_4_SOC", "battery_1_cell_volt_min", "battery_2_cell_volt_min", "battery_3_cell_volt_min", "battery_4_cell_volt_min"]
    elementStr = " ".join(element)
    elementTemplate = "%.5f " * (len(element) - 1) + "%.5f"

    def __init__(self, rate):
        timeStr = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.fileNameStr = "usv_data_" + timeStr + ".txt"
        with open(self.fileNameStr, 'w') as f:
            f.write(self.elementStr + '\n')

        self.saveDataTimer = 1
        self.recordRate = rate

    def saveData(self, usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP):       
        with open(self.fileNameStr, 'a') as f:          
            if (self.saveDataTimer >= 0.5 * self.recordRate):
                f.write(self.elementTemplate % (dt, usvPose.x, usvPose.y, usvPose.psi, usvPose.u, usvPose.v, usvPose.r, uSP, vSP, psiSP, xSP, ySP, usvPose.xDVL, usvPose.yDVL, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.azb, usvPose.roll, usvPose.pitch, usvPose.xLidar, usvPose.yLidar, usvComm.tvAngleEst, usvPose.tvAnglePod, usvPose.tvAngleLidar, usvPose.obsX, usvPose.obsY, usvPose.obsAngleLidar, usvCAN.motorRPM[0], usvCAN.motorRPM[1], usvCAN.motorAngle[0], usvCAN.motorAngle[1], usvCAN.battSOC[0], usvCAN.battSOC[1], usvCAN.battSOC[2], usvCAN.battSOC[3], usvCAN.battCellVoltMin[0], usvCAN.battCellVoltMin[1], usvCAN.battCellVoltMin[2], usvCAN.battCellVoltMin[3]) + '\n')

                self.saveDataTimer = 1
            else:
                self.saveDataTimer = self.saveDataTimer + 1

if __name__ == '__main__':
    # 以下代码为测试代码
    console = Console()
    latestMsg = "Waiting USV self-check to complete..."

    rospy.init_node('usv_record_test_node')
    rosRate = rospy.Rate(2)
    t0 = rospy.Time.now().to_sec()

    usvCAN = CAN()
    usvPose = Pose()
    usvComm = Communication()

    usvState = "STARTUP"
    uSP = float("nan")
    vSP = float("nan")
    psiSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP) 
            console.print(theTable)

            rosRate.sleep()
        except KeyboardInterrupt:
            break
