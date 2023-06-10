#pragma once
#include <functional>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "usefulMathFunc.h"
#include "usv_pose.h"

using namespace std;
class USVControllerPID : public rclcpp::Node {
   public:
    double uPout = 0;
    double uIout = 0;
    double uDout = 0;

    double vPout = 0;
    double vIout = 0;
    double vDout = 0;

    double psiPout = 0;
    double psiIout = 0;
    double psiDout = 0;

    double angleR = 0;
    double angleL = 0;
    double Tx = 0;
    double Ty = 0;
    double TN = 0;
    double TL = 0;
    double TR = 0;

    shared_ptr<USVPose> usvPosePtr;
    double controlFreq = 0;

    USVControllerPID(shared_ptr<USVPose> usvPosePtr_, double controlFreq_ = 20) : Node("USVControllerPID") {
        lThrustCmdPub = this->create_publisher<std_msgs::msg::Float64>("/usv/left/thrust/cmd_thrust", 10);
        rThrustCmdPub = this->create_publisher<std_msgs::msg::Float64>("/usv/right/thrust/cmd_thrust", 10);
        lAngleCmdPub = this->create_publisher<std_msgs::msg::Float64>("/usv/left/thrust/joint/cmd_pos", 10);
        rAngleCmdPub = this->create_publisher<std_msgs::msg::Float64>("/usv/right/thrust/joint/cmd_pos", 10);
        psiErrPub = this->create_publisher<std_msgs::msg::Float64>("/usv/error/psi", 10);

        usvPosePtr = usvPosePtr_;
        controlFreq = controlFreq_;

        // 文本读取PID
        updatePID();

    }
    ~USVControllerPID() {
        setUSVThrust(0.0, 0.0, 0.0, 0.0);
        printf("Controller terminated.\n");
        return;
    }

    void moveUSV(double uErr, double psiErr) {
        if (!usvPosePtr->isValid) {
            printf("Controller cannot trust current USV pose. Its isAvailable flag is false.\n");
            return;
        } else {
            updatePID();
            
            Tx = usvMass * PID(uErr, Uerr0, kp_u, ki_u, kd_u, uPout, uIout, uDout);
            Ty = 0;
            TN = usvInerZ * PID(psiErr, Psierr0, kp_psi, ki_psi, kd_psi, psiPout, psiIout, psiDout);

            Uerr0 = uErr;
            Psierr0 = psiErr;

            mixer(Tx, Ty, TN, TL, TR, angleL, angleR);

            // Set thrust
            setUSVThrust(TL, TR, 0.0, 0.0);

            psiMsg.data = psiErr;
            psiErrPub->publish(psiMsg);

            printf("=============================================================================\n");
            printf("控制器输出: \n");
            printf("USV 位置：[%08.2f, %08.2f]m | USV 速度：[%05.2f, %05.2f]m/s\n", usvPosePtr->x, usvPosePtr->y, usvPosePtr->vx, usvPosePtr->vy);
            printf("u 速度误差：%06.2fm/s, 角度误差：%06.2f\n", uErr, psiErr);
            printf("左右转速 Setpoint：[%07.3f, %07.3f] rpm\n", TL, TR);
        }
    }

   private:
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> lThrustCmdPub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> rThrustCmdPub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> lAngleCmdPub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> rAngleCmdPub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> psiErrPub;

    std_msgs::msg::Float64 lTCmdMsg = std_msgs::msg::Float64();
    std_msgs::msg::Float64 rTCmdMsg = std_msgs::msg::Float64();
    std_msgs::msg::Float64 lACmdMsg = std_msgs::msg::Float64();
    std_msgs::msg::Float64 rACmdMsg = std_msgs::msg::Float64();
    std_msgs::msg::Float64 psiMsg = std_msgs::msg::Float64();

    // 无人船参数
    double usvMass = 775;
    double usvInerZ = 2072.475;

    // 前向速度控制参数
    double kp_u = 0.65;
    double ki_u = 0.1;
    double kd_u = 0.0025;

    // 侧向速度控制参数
    double kp_v = 1;
    double ki_v = 0;
    double kd_v = 0;

    // 朝向角控制参数
    double kp_psi = 0.055;
    double ki_psi = 0.0005;
    double kd_psi = 0.4125;

    // 初始误差
    double Uerr0 = 0;
    double Verr0 = 0;
    double Psierr0 = 0;

    // 推力限幅 最大推力
    double Tmax = 1200;

    void updatePID() {
        ifstream pidFile("/home/kk/mbzirc_ws/pid.txt", ios::in);
        double pidCoeff[6] = {};
        if (pidFile) {
            for (int i = 0; i < 6; i++) {
                pidFile >> pidCoeff[i];
            }
            pidFile.close();
        }

        kp_u = pidCoeff[0];
        ki_u = pidCoeff[1];
        kd_u = pidCoeff[2];
        kp_psi = pidCoeff[3];
        ki_psi = pidCoeff[4];
        kd_psi = pidCoeff[5];

        return;
    }

    double PID(double error, double error0, double kp, double ki, double kd, double &p_out, double &i_out, double &d_out) {
        p_out = kp * error;
        i_out += ki * error / controlFreq;
        d_out = kd * (error - error0) * controlFreq;
        return p_out + i_out + d_out;
    }

    void mixer(double TxSP, double TySP, double TN, double &TL, double &TR, double &angleL, double &angleR) {
        TL = calcNorm2(TxSP, TySP) / 2 - TN / 2;
        TR = calcNorm2(TxSP, TySP) / 2 + TN / 2;
        // 推力限幅
        if (abs(TL) > Tmax) {
            TR = Tmax / TL * sign(TL) * TR;
            TL = Tmax * sign(TL);
        } else {
            TL = TL;
            TR = TR;
        }
        if (abs(TR) > Tmax) {
            TL = Tmax / TR * sign(TR) * TL;
            TR = Tmax * sign(TR);
        } else {
            TL = TL;
            TR = TR;
        }
        angleL = atan(TySP / TxSP);
        angleR = atan(TySP / TxSP);
        // 电机偏角限幅
        if (abs(angleL) > 0.5 * PI) {
            angleL = 0.45 * sign(angleL) * PI;
            angleR = 0.45 * sign(angleR) * PI;
        } else {
            angleL = angleL;
            angleR = angleR;
        }
        return;
    }

    void setUSVThrust(double TL, double TR, double angleL, double angleR) {
        lTCmdMsg.data = TL;
        rTCmdMsg.data = TR;
        lACmdMsg.data = 0;
        rACmdMsg.data = 0;

        lThrustCmdPub->publish(lTCmdMsg);
        rThrustCmdPub->publish(rTCmdMsg);
        lAngleCmdPub->publish(lACmdMsg);
        rAngleCmdPub->publish(rACmdMsg);
        return;
    }
};