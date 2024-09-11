/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "data_bus.h"
#include <string>
#include "json/json.h"
#include <vector>

class Pin_KinDyn
{
public:
    std::vector<bool> motorReachLimit;
    const std::vector<std::string> motorName = {"J_arm_l_01", "J_arm_l_02", "J_arm_l_03", "J_arm_l_04", "J_arm_l_05",
                                                "J_arm_l_06", "J_arm_l_07", "J_arm_r_01", "J_arm_r_02", "J_arm_r_03",
                                                "J_arm_r_04", "J_arm_r_05", "J_arm_r_06", "J_arm_r_07",
                                                "J_head_yaw", "J_head_pitch", "J_waist_pitch", "J_waist_roll", "J_waist_yaw",
                                                "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch",
                                                "J_ankle_l_pitch", "J_ankle_l_roll", "J_hip_r_roll", "J_hip_r_yaw",
                                                "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch", "J_ankle_r_roll"}; // joint name in urdf and jason config files
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    Eigen::VectorXd tauJointOld;
    std::string urdf_path;
    pinocchio::Model model_biped;
    pinocchio::Model model_biped_fixed;
    int model_nv;
    pinocchio::JointIndex r_ankle_joint, l_ankle_joint, base_joint, r_hip_joint, l_hip_joint, r_hip_roll_joint, l_hip_roll_joint, waist_yaw_joint;
    pinocchio::JointIndex r_ankle_joint_fixed, l_ankle_joint_fixed, r_hip_joint_fixed, l_hip_joint_fixed;
    pinocchio::JointIndex r_hand_joint, l_hand_joint, r_hand_joint_fixed, l_hand_joint_fixed;
    Eigen::VectorXd q, dq, ddq;
    Eigen::Matrix3d Rcur;
    Eigen::Quaternion<double> quatCur;
    Eigen::Matrix<double, 6, -1> J_r, J_l, J_hd_r, J_hd_l, J_base, J_hip_link, J_r_body, J_l_body;
    Eigen::Matrix<double, 6, -1> dJ_r, dJ_l, dJ_hd_r, dJ_hd_l, dJ_base, dJ_hip_link;
    Eigen::Matrix<double, 3, -1> Jcom;
    Eigen::Vector3d fe_r_pos, fe_l_pos, base_pos; // foot-end position in world frame
    Eigen::Vector3d fe_r_pos_body, fe_l_pos_body; // foot-end position in body frame
    Eigen::Vector3d hd_r_pos, hd_l_pos;           // hand position in world frame
    Eigen::Vector3d fe_r_vel_body, fe_l_vel_body; // foot-end velcity in body frame
    Eigen::Vector3d hd_r_pos_body, hd_l_pos_body; // hand position in body frame
    Eigen::Vector3d hip_r_pos, hip_l_pos, hip_link_pos;
    Eigen::Vector3d hip_r_pos_body, hip_l_pos_body;
    Eigen::Matrix3d hip_link_rot;
    Eigen::Matrix3d fe_r_rot, fe_l_rot, base_rot;
    Eigen::Matrix3d fe_r_rot_body, fe_l_rot_body;
    Eigen::Matrix3d hd_r_rot, hd_l_rot;
    Eigen::Matrix3d hd_r_rot_body, hd_l_rot_body;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::Vector3d CoM_pos;
    Eigen::Matrix3d inertia;
    enum legIdx
    {
        left,
        right
    };
    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPosRes;
    };

    Pin_KinDyn(std::string urdf_pathIn);
    void dataBusRead(DataBus const &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeJ_dJ();
    void computeDyn();
    IkRes computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    IkRes computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);

private:
    pinocchio::Data data_biped, data_biped_fixed;
};
