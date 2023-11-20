#!/usr/bin/env python3
import rospy, time, threading

from usv_pose import Pose
from usv_communication import Communication

from numpy import rad2deg
from rich.table import Table
from rich import box
from rich.console import Console

def genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP):    
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

    # 构造表格数据
    tableData = {
        "[reverse]%s" % usvState: ["GPS: %d" % usvPose.isGPSValid, "Imu: %d" % usvPose.isImuValid, "Dvl: %d" % usvPose.isDvlValid, "Pod: %d" % usvPose.isPodValid, "Lidar: %d" % usvPose.isLidarValid],
        "Motions": ["u: %.2f m/s" % usvPose.uDVL, "v: %.2f m/s" % usvPose.vDVL, "psi: %.2f deg" % rad2deg(usvPose.psi), "r: %.2f deg/s" % rad2deg(usvPose.r), "ax: %.2f m/s^2" % usvPose.axb, "ay: %.2f m/s^2" % usvPose.ayb],
        "Setpoints": ["uSP: %.2f m/s" % uSP, "vSP: %.2f m/s" % vSP, "psiSP: %.2f deg" % rad2deg(psiSP), "rSP: %.2f deg/s" % rad2deg(rSP), "[bold bright_yellow]xSP: %.2f m" % xSP, "[bold bright_yellow]ySP: %.2f m" % ySP, "axbSP: %.2f m/s^2" % axbSP, "aybSP: %.2f m/s^2" % aybSP, "etaSP: %.2f deg/s^2" % rad2deg(etaSP)],
        "Sensors": ["x(GPS): %.2f m" % usvPose.x, "y(GPS): %.2f m" % usvPose.y, "Lidar dist: %.2f m" % lidarOutPutDist, "[bold bright_yellow]x(Lidar): %.2f m" % xLidarOutput, "[bold bright_yellow]y(Lidar): %.2f m" % yLidarOutput, "[bold yellow]sUAV yaw: %.2f deg" % sUAVOutput, "[bold yellow]Pod yaw: %.2f deg" % podOutput, "[bold yellow]Lidar yaw: %.2f deg" % lidarOutPutAngle, "TV Heading: %.2f deg" % rad2deg(usvPose.tvHeading), "Obs yaw: %.2f deg" % obsOutPut],
        "Power": ["L_cmd: %.2f RPM | %.2f deg" % (usvControl.rpmLeftSP, rad2deg(usvControl.angleLeftSP)), "R_cmd: %.2f RPM | %.2f deg" % (usvControl.rpmRightSP, rad2deg(usvControl.angleRightSP)), "L: %.2f RPM | %.2f deg" % (usvControl.rpmLeftEst, rad2deg(usvControl.angleLeftEst)), "R: %.2f RPM | %.2f deg" % (usvControl.rpmRightEst, rad2deg(usvControl.angleRightEst)), "1: %.1f %% | ↓%.3f v" % (usvControl.battSOC[0], usvControl.battCellVoltMin[0]), "2: %.1f %% | ↓%.3f v" % (usvControl.battSOC[1], usvControl.battCellVoltMin[1]), "3: %.1f %% | ↓%.3f v" % (usvControl.battSOC[2], usvControl.battCellVoltMin[2]), "4: %.1f %% | ↓%.3f v" % (usvControl.battSOC[3], usvControl.battCellVoltMin[3])],
    }

    theTable = Table(show_header=True, header_style="bold", title_justify="center", title_style="bold magenta", caption_justify="left", box=box.HORIZONTALS)
    theTable.title = "USV Info @ t = %.3f s" % dt
    theTable.caption = " Message: " + latestMsg + "\n"

    maxRows = max(len(columnData) for columnData in tableData.values())

    # 添加列头
    for i in tableData.keys():
        theTable.add_column(i)

    # 对于每一行
    for i in range(maxRows):
        # 使用列表推导快速生成行数据
        rowData = [columnData[i] if i < len(columnData) else "" for columnData in tableData.values()]
        theTable.add_row(*rowData)


    return theTable

class USVData():
    element = ["t", "x_GPS", "y_GPS", "psi", "u", "v", "r", "uSP", "vSP", "psiSP", "rSP", "xSP", "ySP", "axbSP", "aybSP", "etaSP", "x_DVL", "y_DVL", "u_DVL", "v_DVL", "ax", "ay", "az", "roll", "pitch", "x_Lidar", "y_Lidar","search_angle", "pod_angle", "lidar_angle", "target_vessel_heading", "obs_x", "obs_y", "obs_angle", "rpm_left_cmd", "rpm_right_cmd", "angle_left_cmd", "angle_right_cmd", "rpm_left", "rpm_right", "angle_left", "angle_right", "battery_1_SOC", "battery_2_SOC", "battery_3_SOC", "battery_4_SOC", "battery_1_cell_volt_min", "battery_2_cell_volt_min", "battery_3_cell_volt_min", "battery_4_cell_volt_min"]
    elementStr = " ".join(element)
    elementTemplate = "%.5f " * (len(element) - 1) + "%.5f"

    def __init__(self):
        timeStr = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.fileNameStr = "usv_data_" + timeStr + ".txt"
        with open(self.fileNameStr, 'w') as f:
            f.write(self.elementStr + '\n')

        self.recordInterval = 0.25 # unit: sec
        self.thisTime = rospy.Time.to_sec()

    def saveData(self, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP):       
        # 如果时间大于记录间隔，才记录
        if (rospy.Time.to_sec() - self.thisTime >= self.recordInterval):
            self.thisTime = rospy.Time.to_sec()
            
            with open(self.fileNameStr, 'a') as f:          
                f.write(self.elementTemplate % (dt, usvPose.x, usvPose.y, usvPose.psi, usvPose.u, usvPose.v, usvPose.r, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP, usvPose.xDVL, usvPose.yDVL, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.azb, usvPose.roll, usvPose.pitch, usvPose.xLidar, usvPose.yLidar, usvComm.tvAngleEst, usvPose.tvAnglePod, usvPose.tvAngleLidar, usvPose.tvHeading, usvPose.obsX, usvPose.obsY, usvPose.obsAngleLidar, usvControl.rpmLeftSP, usvControl.rpmRightSP, usvControl.angleLeftSP, usvControl.angleRightSP, usvControl.rpmLeftEst, usvControl.rpmRightEst, usvControl.angleLeftEst, usvControl.angleRightEst, usvControl.battSOC[0], usvControl.battSOC[1], usvControl.battSOC[2], usvControl.battSOC[3], usvControl.battCellVoltMin[0], usvControl.battCellVoltMin[1], usvControl.battCellVoltMin[2], usvControl.battCellVoltMin[3]) + '\n')
            
            
                

if __name__ == '__main__':
    # 以下代码为测试代码
    console = Console()
    latestMsg = "Waiting USV self-check to complete..."

    rospy.init_node('usv_record_test_node')
    rosRate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()

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
            theTable = genTable(usvState, latestMsg, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP) 
            console.print(theTable)

            rosRate.sleep()
        except KeyboardInterrupt:
            break
