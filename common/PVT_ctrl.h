/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

//-------------------------------------------NOTE------------------------------------//
//
// The damping(Kd) in the joint_ctrl_config.json is relatively large, and they may not match the real ones.
//
//-----------------------------------------------------------------------------------//
#pragma once
#include <fstream>
#include "json/json.h"
#include <string>
#include "LPF_fst.h"
#include <vector>
#include <cmath>
#include "data_bus.h"

class PVT_Ctr {
public:
    int jointNum;
    std::vector<double> motor_pos_cur;
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_out_link; // final tau output
    std::vector<double> motor_tor_out_motor; // final tau output
    PVT_Ctr(double timeStepIn, const char * jsonPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void enablePV(); // enable PV control item
    void disablePV(); // disable PV control item
    void enablePV(int jtId); // enable PV control item
    void disablePV(int jtId); // disable PV control item
    void setJointPD(double kp, double kd, const char * jointName);
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    std::vector<double> motor_pos_des; // P des
    std::vector<double> motor_vel_des; // V des
    std::vector<double> motor_tor_des; // T des

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> gear;

private:
    std::vector<LPF_Fst> tau_out_lpf;
    std::vector<int> PV_enable;
    double sign(double in);
    const std::vector<std::string> motorName={"J_arm_l_01","J_arm_l_02","J_arm_l_03", "J_arm_l_04", "J_arm_l_05",
                                              "J_arm_l_06","J_arm_l_07","J_arm_r_01", "J_arm_r_02", "J_arm_r_03",
                                              "J_arm_r_04","J_arm_r_05","J_arm_r_06", "J_arm_r_07",
                                              "J_head_yaw","J_head_pitch","J_waist_pitch","J_waist_roll", "J_waist_yaw",
                                              "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch",
                                              "J_ankle_l_pitch", "J_ankle_l_roll", "J_hip_r_roll", "J_hip_r_yaw",
                                              "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch", "J_ankle_r_roll"}; // joint name in urdf and jason config files
};


