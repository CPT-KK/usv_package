#!/usr/bin/env python3
import rospy, time, threading, logging

from usv_pose import Pose
from usv_path_planner import PathPlanner
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication

from numpy import rad2deg, array, isnan
from rich.table import Table
from rich import box
from rich.console import Console
from rich.logging import RichHandler


def bool2okstr(boolVal):
    if (boolVal):
        return "[bold green]√[/bold green]"
    else:
        return "[bold red]X[/bold red]"

def genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP):    
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

    isMotorValid = ~isnan(array([usvControl.rpmLeftEst, usvControl.rpmRightEst, usvControl.angleLeftEst, usvControl.angleRightEst]))

    # 构造表格数据
    tableData = {
        f"[reverse]{usvState}" : [
            f"GPS: {bool2okstr(usvPose.isGPSValid)}, Imu: {bool2okstr(usvPose.isImuValid)}", 
            f"Dvl: {bool2okstr(usvPose.isDvlValid)}, Pod: {bool2okstr(usvPose.isPodValid)}", 
            f"Lidar: {bool2okstr(usvPose.isLidarValid)}",
            "",
            f"Thrust: {bool2okstr(isMotorValid[0])}, {bool2okstr(isMotorValid[1])}",
            f"Angle: {bool2okstr(isMotorValid[2])}, {bool2okstr(isMotorValid[3])}",
            "",
            "[bold]S.S state:",
            f"sUAV: {usvComm.suavState}", f"sUAV: {usvComm.suavminiState}", f"tUAV1: {usvComm.tuav1State}"
        ],
        # 
        "Motions": [
            f"[reverse purple]u: {usvPose.uDVL:.2f} m/s", f"[reverse purple]v: {usvPose.vDVL:.2f} m/s", 
            "",
            f"[reverse cyan]yaw: {rad2deg(usvPose.yaw):.2f} deg", f"[reverse cyan]r: {rad2deg(usvPose.r):.2f} deg/s",
            "",
            f"[reverse green]x: {xLidarOutput:.2f} m", f"[reverse green]y: {yLidarOutput:.2f} m", 
            "",
            f"ax: {usvPose.axb:.2f} m/s^2", f"ay: {usvPose.ayb:.2f} m/s^2"
        ],
        # 
        "Setpoints": [
            f"[reverse purple]uSP: {uSP:.2f} m/s", f"[reverse purple]vSP: {vSP:.2f} m/s",
            "",
            f"[reverse cyan]yawSP: {rad2deg(yawSP):.2f} deg", f"[reverse cyan]rSP: {rad2deg(rSP):.2f} deg/s", 
            "",
            f"[reverse green]xSP: {xSP:.2f} m", f"[reverse green]ySP: {ySP:.2f} m", 
            "",
            f"axbSP: {axbSP:.2f} m/s^2", f"aybSP: {aybSP:.2f} m/s^2"
        ],
# 
        "Sensors": [
            f"[reverse magenta]sUAV yaw: {sUAVOutput:.2f} deg", 
            f"[reverse magenta]sUAV XY: ({usvPose.tvEstPosX:.2f}, {usvPose.tvEstPosY:.2f})m", 
            "",
            f"Pod lock: {bool2okstr(usvPose.isPodFindTV)}",
            f"[reverse red]Pod yaw: {podOutput:.2f} deg", 
            "",
            f"Lidar lock: {bool2okstr(usvPose.isLidarFindTV)}",
            f"[reverse yellow]Lidar yaw: {lidarOutPutAngle:.2f} deg",   
            f"[reverse yellow]Lidar dist: {lidarOutPutDist:.2f} m", 
            "",
            f"[reverse blue]TV L&W: {usvPose.tvLength:.2f} m, {usvPose.tvWidth:.2f} m",
            f"[reverse blue]TV Heading: {rad2deg(usvPose.tvHeading):.2f} deg", 
            "",
            f"Obs yaw: {obsOutPut:.2f} deg"
        ],
        # 
        "Power": [
            "[bold]Engine",
            f"L_cmd: {usvControl.rpmLeftSP:.2f} RPM | {rad2deg(usvControl.angleLeftSP):.2f} deg", 
            f"R_cmd: {usvControl.rpmRightSP:.2f} RPM | {rad2deg(usvControl.angleRightSP):.2f} deg", 
            f"L [{bool2okstr(usvControl.driveReadyLeft)}]: {usvControl.rpmLeftEst:.2f} RPM | {rad2deg(usvControl.angleLeftEst):.2f} deg", 
            f"R [{bool2okstr(usvControl.driveReadyRight)}]: {usvControl.rpmRightEst:.2f} RPM | {rad2deg(usvControl.angleRightEst):.2f} deg",
            "",
            "[bold]Battery",
            f"1: {usvControl.battSOC[0]:.1f} % | ↓{usvControl.battCellVoltMin[0]:.3f} v", 
            f"2: {usvControl.battSOC[1]:.1f} % | ↓{usvControl.battCellVoltMin[1]:.3f} v", 
            f"3: {usvControl.battSOC[2]:.1f} % | ↓{usvControl.battCellVoltMin[2]:.3f} v", 
            f"4: {usvControl.battSOC[3]:.1f} % | ↓{usvControl.battCellVoltMin[3]:.3f} v"
        ],
    }

    theTable = Table(show_header=True, header_style="bold", title_justify="center", title_style="bold white", caption_justify="left", box=box.HORIZONTALS, caption_style="white")
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
    element = ["USV_State", "t", "x_GPS", "y_GPS", "yaw", "u", "v", "r", "uSP", "vSP", "yawSP", "rSP", "xSP", "ySP", "axbSP", "aybSP", "etaSP", "x_DVL", "y_DVL", "u_DVL", "v_DVL", "ax", "ay", "az", "roll", "pitch", "x_Lidar", "y_Lidar", "search_tv_x", "search_tv_y", "search_angle", "pod_angle", "pod_state", "lidar_object_num", "lidar_angle", "target_vessel_heading", "target_vessel_length", "target_vessel_width", "target_vessel_highest_x", "target_vessel_highest_y", "target_vessel_highest_z", "lidar_target_body_x", "lidar_target_body_y", "obs_x", "obs_y", "obs_angle", "rpm_left_cmd", "rpm_right_cmd", "angle_left_cmd", "angle_right_cmd", "rpm_left", "rpm_right", "status_left", "status_right", "angle_left", "angle_right", "battery_1_SOC", "battery_2_SOC", "battery_3_SOC", "battery_4_SOC", "battery_1_cell_volt_min", "battery_2_cell_volt_min", "battery_3_cell_volt_min", "battery_4_cell_volt_min"]
    elementStr = " ".join(element)
    elementTemplate = "%s " + "%.5f " * (len(element) - 2) + "%.5f"

    def __init__(self, currTimeStamp):
        self.fileNameStr = "usv_data_" + currTimeStamp + ".txt"
        with open(self.fileNameStr, 'w') as f:
            f.write(self.elementStr + '\n')

        self.recordInterval = 0.25 # unit: sec
        self.thisTime = rospy.Time.now().to_sec()

    def saveData(self, usvState, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP):       
        # 如果时间大于记录间隔，才记录
        if (rospy.Time.now().to_sec() - self.thisTime >= self.recordInterval):
            self.thisTime = rospy.Time.now().to_sec()
            
            with open(self.fileNameStr, 'a') as f:          
                f.write(self.elementTemplate % (usvState, dt, usvPose.x, usvPose.y, usvPose.yaw, usvPose.u, usvPose.v, usvPose.r, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP, usvPose.xDVL, usvPose.yDVL, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.azb, usvPose.roll, usvPose.pitch, usvPose.xLidar, usvPose.yLidar, usvPose.tvEstPosX, usvPose.tvEstPosY, usvPose.tvAngleEst, usvPose.tvAnglePod, usvPose.podState, usvPose.objectNum, usvPose.tvAngleLidar, usvPose.tvHeading, usvPose.tvLength, usvPose.tvWidth, usvPose.tvHighestX, usvPose.tvHighestY, usvPose.tvHighestZ, usvPose.tvXBody, usvPose.tvYBody, usvPose.obsX, usvPose.obsY, usvPose.obsAngleLidar, usvControl.rpmLeftSP, usvControl.rpmRightSP, usvControl.angleLeftSP, usvControl.angleRightSP, usvControl.rpmLeftEst, usvControl.rpmRightEst, usvControl.statusLeft, usvControl.statusRight, usvControl.angleLeftEst, usvControl.angleRightEst, usvControl.battSOC[0], usvControl.battSOC[1], usvControl.battSOC[2], usvControl.battSOC[3], usvControl.battCellVoltMin[0], usvControl.battCellVoltMin[1], usvControl.battCellVoltMin[2], usvControl.battCellVoltMin[3]) + '\n')
       

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
    yawSP = float("nan")
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
            theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
            console.print(theTable)

            rosRate.sleep()
        except KeyboardInterrupt:
            break
