#!/usr/bin/env python3
import rospy, time, threading

from usv_pose import Pose
from usv_path_planner import PathPlanner
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication

from numpy import rad2deg
from rich.table import Table
from rich import box
from rich.console import Console

def genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP):    
    if (usvPose.isSearchFindTV):
        sUAVOutput = rad2deg(usvPose.tvAngleEst)
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
        f"[reverse]{usvState}" : [f"GPS: {usvPose.isGPSValid}", f"Imu: {usvPose.isImuValid}", 
                                  f"Dvl: {usvPose.isDvlValid}", f"Pod: {usvPose.isPodValid}", 
                                  f"Lidar: {usvPose.isLidarValid}", f"sUAV: {usvComm.suavState}", f"tUAV1: {usvComm.tuav1State}"],
        # 
        "Motions": [f"[purple]u: {usvPose.uDVL:.2f} m/s", f"[reverse purple]v: {usvPose.vDVL:.2f} m/s", 
                    f"psi: {rad2deg(usvPose.psi):.2f} deg", f"r: {rad2deg(usvPose.r):.2f} deg/s", 
                    f"ax: {usvPose.axb:.2f} m/s^2", f"ay: {usvPose.ayb:.2f} m/s^2"],
        # 
        "Setpoints": [f"[purple]uSP: {uSP:.2f} m/s", f"[reverse purple]vSP: {vSP:.2f} m/s", 
                      f"psiSP: {rad2deg(psiSP):.2f} deg", f"rSP: {rad2deg(rSP):.2f} deg/s", 
                      f"[bold bright_yellow]xSP: {xSP:.2f} m", f"[bold bright_yellow]ySP: {ySP:.2f} m", 
                      f"axbSP: {axbSP:.2f} m/s^2", f"aybSP: {aybSP:.2f} m/s^2", 
                      f"etaSP: {rad2deg(etaSP):.2f} deg/s^2"],
        # 
        "Sensors": [f"x(GPS): {usvPose.x:.2f} m", f"y(GPS): {usvPose.y:.2f} m", 
                    f"Lidar dist: {lidarOutPutDist:.2f} m", 
                    f"[bold bright_yellow]x(Lidar): {xLidarOutput:.2f} m", 
                    f"[bold bright_yellow]y(Lidar): {yLidarOutput:.2f} m", 
                    f"x(sUAV): {usvPose.tvEstPosX:.2f} m", f"y(sUAV): {usvPose.tvEstPosY:.2f} m", 
                    f"[bold yellow]sUAV yaw: {sUAVOutput:.2f} deg", 
                    f"[bold yellow]Pod yaw: {podOutput:.2f} deg", 
                    f"[bold yellow]Lidar yaw: {lidarOutPutAngle:.2f} deg", 
                    f"TV Heading: {rad2deg(usvPose.tvHeading):.2f} deg", 
                    f"Obs yaw: {obsOutPut:.2f} deg"],
        # 
        "Power": ["[bold]Engine",
                  f"L_cmd: {usvControl.rpmLeftSP:.2f} RPM | {rad2deg(usvControl.angleLeftSP):.2f} deg", 
                  f"R_cmd: {usvControl.rpmRightSP:.2f} RPM | {rad2deg(usvControl.angleRightSP):.2f} deg", 
                  f"L: {usvControl.rpmLeftEst:.2f} RPM | {rad2deg(usvControl.angleLeftEst):.2f} deg", 
                  f"R: {usvControl.rpmRightEst:.2f} RPM | {rad2deg(usvControl.angleRightEst):.2f} deg",
                  "",
                  "[bold]Battery",
                  f"1: {usvControl.battSOC[0]:.1f} % | ↓{usvControl.battCellVoltMin[0]:.3f} v", 
                  f"2: {usvControl.battSOC[1]:.1f} % | ↓{usvControl.battCellVoltMin[1]:.3f} v", 
                  f"3: {usvControl.battSOC[2]:.1f} % | ↓{usvControl.battCellVoltMin[2]:.3f} v", 
                  f"4: {usvControl.battSOC[3]:.1f} % | ↓{usvControl.battCellVoltMin[3]:.3f} v"],
    }

    theTable = Table(show_header=True, header_style="bold", title_justify="center", title_style="bold magenta", caption_justify="left", box=box.HORIZONTALS)
    theTable.title = f"USV Info @ t = {dt:.3f} s"
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
    element = ["t", "x_GPS", "y_GPS", "psi", "u", "v", "r", "uSP", "vSP", "psiSP", "rSP", "xSP", "ySP", "axbSP", "aybSP", "etaSP", "x_DVL", "y_DVL", "u_DVL", "v_DVL", "ax", "ay", "az", "roll", "pitch", "x_Lidar", "y_Lidar", "search_tv_x", "search_tv_y", "search_angle", "pod_angle", "lidar_angle", "target_vessel_heading", "target_vessel_length", "target_vessel_width", "obs_x", "obs_y", "obs_angle", "rpm_left_cmd", "rpm_right_cmd", "angle_left_cmd", "angle_right_cmd", "rpm_left", "rpm_right", "angle_left", "angle_right", "battery_1_SOC", "battery_2_SOC", "battery_3_SOC", "battery_4_SOC", "battery_1_cell_volt_min", "battery_2_cell_volt_min", "battery_3_cell_volt_min", "battery_4_cell_volt_min"]
    elementStr = " ".join(element)
    elementTemplate = "%.5f " * (len(element) - 1) + "%.5f"

    def __init__(self):
        timeStr = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.fileNameStr = "usv_data_" + timeStr + ".txt"
        with open(self.fileNameStr, 'w') as f:
            f.write(self.elementStr + '\n')

        self.recordInterval = 0.25 # unit: sec
        self.thisTime = rospy.Time.now().to_sec()

    def saveData(self, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP):       
        # 如果时间大于记录间隔，才记录
        if (rospy.Time.now().to_sec() - self.thisTime >= self.recordInterval):
            self.thisTime = rospy.Time.now().to_sec()
            
            with open(self.fileNameStr, 'a') as f:          
                f.write(self.elementTemplate % (dt, usvPose.x, usvPose.y, usvPose.psi, usvPose.u, usvPose.v, usvPose.r, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP, usvPose.xDVL, usvPose.yDVL, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.azb, usvPose.roll, usvPose.pitch, usvPose.xLidar, usvPose.yLidar, usvPose.tvEstPosX, usvPose.tvEstPosY, usvPose.tvAngleEst, usvPose.tvAnglePod, usvPose.tvAngleLidar, usvPose.tvHeading, usvPose.tvLength, usvPose.tvWidth,usvPose.obsX, usvPose.obsY, usvPose.obsAngleLidar, usvControl.rpmLeftSP, usvControl.rpmRightSP, usvControl.angleLeftSP, usvControl.angleRightSP, usvControl.rpmLeftEst, usvControl.rpmRightEst, usvControl.angleLeftEst, usvControl.angleRightEst, usvControl.battSOC[0], usvControl.battSOC[1], usvControl.battSOC[2], usvControl.battSOC[3], usvControl.battCellVoltMin[0], usvControl.battCellVoltMin[1], usvControl.battCellVoltMin[2], usvControl.battCellVoltMin[3]) + '\n')
       

if __name__ == '__main__':
    # 以下代码为测试代码
    console = Console()
    latestMsg = "Waiting USV self-check to complete..."

    rospy.init_node('usv_record_test_node')
    rosRate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()

    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control()

    usvState = "STARTUP"
    uSP = float("nan")
    vSP = float("nan")
    psiSP = float("nan")
    rSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")
    axbSP = float("nan")
    aybSP = float("nan")
    etaSP = float("nan")

    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()

    while True:
        try:
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
            console.print(theTable)

            rosRate.sleep()
        except KeyboardInterrupt:
            break
