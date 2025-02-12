/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include "iomanip"

struct DataBus
{
    const int model_nv; // number of dq

    // const values for frame mismatch
    const Eigen::Matrix3d fe_L_rot_L_off = (Eigen::MatrixXd(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished(); // left foot-end R w.r.t to the body frame in offset posture
    const Eigen::Matrix3d fe_R_rot_L_off = (Eigen::MatrixXd(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished();

    // motors, sensors and states feedback
    double rpy[3];
    double fL[3];
    double fR[3];
    double basePos[3];
    double baseLinVel[3]; // velocity of the basePos
    double baseAcc[3];    // baseAcc of the base link
    double baseAngVel[3]; // angular velocity of the base link
    std::vector<double> motors_pos_cur;
    std::vector<double> motors_vel_cur;
    std::vector<double> motors_tor_cur;
    Eigen::VectorXd FL_est, FR_est;
    bool isdqIni;

    // PVT controls
    std::vector<double> motors_pos_des;
    std::vector<double> motors_vel_des;
    std::vector<double> motors_tor_des;
    std::vector<double> motors_tor_out;

    // states and key variables
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd qOld;
    Eigen::MatrixXd J_base, J_l, J_r, J_hd_l, J_hd_r, J_hip_link;
    Eigen::MatrixXd dJ_base, dJ_l, dJ_r, dJ_hd_l, dJ_hd_r;
    Eigen::MatrixXd Jcom_W; // jacobian of CoM, in world frame
    Eigen::Vector3d pCoM_W;
    Eigen::Vector3d fe_r_pos_W, fe_l_pos_W, base_pos, base_vel;
    Eigen::Matrix3d fe_r_rot_W, fe_l_rot_W, base_rot; // in world frame
    Eigen::Vector3d fe_r_pos_L, fe_l_pos_L;           // in Body frame
    Eigen::Vector3d fe_r_vel_L, fe_l_vel_L;           // linear velocity in Body frame
    Eigen::Vector3d hip_link_pos;
    Eigen::Vector3d hip_r_pos_L, hip_l_pos_L;
    Eigen::Vector3d hip_r_pos_W, hip_l_pos_W;
    Eigen::Matrix3d fe_r_rot_L, fe_l_rot_L;
    Eigen::Matrix3d hip_link_rot;
    Eigen::Vector3d fe_r_pos_L_cmd, fe_l_pos_L_cmd;
    Eigen::Matrix3d fe_r_rot_L_cmd, fe_l_rot_L_cmd;

    Eigen::Vector3d hd_r_pos_W, hd_l_pos_W; // in world frame
    Eigen::Matrix3d hd_r_rot_W, hd_l_rot_W;
    Eigen::Vector3d hd_r_pos_L, hd_l_pos_L; // in body frame
    Eigen::Matrix3d hd_r_rot_L, hd_l_rot_L;
    Eigen::VectorXd qCmd, dqCmd;
    Eigen::VectorXd tauJointCmd;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_G, dyn_Non;
    Eigen::Vector3d base_omega_L, base_omega_W, base_rpy;

    Eigen::Vector3d slop;
    Eigen::Matrix<double, 3, 3> inertia;

    // state EST
    Eigen::Matrix<double, 3, 1> base_pos_est, base_vel_est;
    Eigen::Matrix<double, 3, 1> eul_est, omegaW_est;
    Eigen::Matrix<double, 3, 1> fe_l_pos_W_est, fe_r_pos_W_est;
    Eigen::Matrix<double, 3, 1> delta_acc;
    Eigen::Matrix<double, 3, 1> freeAcc;

    Eigen::Matrix<double, 15, 1> AX, BU;
    Eigen::Matrix<double, 14, 1> CX;
    Eigen::Matrix<double, 14, 1> Y;
    Eigen::Matrix<double, 6, 1> pbW;

    // cmd value from the joystick interpreter
    Eigen::Vector3d js_eul_des;
    Eigen::Vector3d js_pos_des;
    Eigen::Vector3d js_omega_des;
    Eigen::Vector3d js_vel_des;

    // cmd values for MPC
    Eigen::VectorXd Xd;
    Eigen::VectorXd X_cur;
    //    Eigen::Vector3d     mpc_eul_des;
    //    Eigen::Vector3d     mpc_pos_des;
    //    Eigen::Vector3d     mpc_omega_des;
    //    Eigen::Vector3d     mpc_vel_des;
    Eigen::VectorXd X_cal;
    Eigen::VectorXd dX_cal;
    Eigen::VectorXd fe_react_tau_cmd;

    int qp_nWSR_MPC;
    double qp_cpuTime_MPC;
    int qpStatus_MPC;

    // cmd values for WBC
    Eigen::Vector3d base_rpy_des;
    Eigen::Vector3d base_pos_des;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;
    Eigen::Vector3d swing_fe_pos_des_W;
    Eigen::Vector3d swing_fe_rpy_des_W;
    Eigen::Vector3d stance_fe_pos_cur_W;
    Eigen::Matrix3d stance_fe_rot_cur_W;
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final;
    Eigen::VectorXd wbc_tauJointRes;
    Eigen::VectorXd wbc_FrRes;
    Eigen::VectorXd Fr_ff;
    int qp_nWSR;
    double qp_cpuTime;
    int qp_status;

    // values for foot-placement
    Eigen::Vector3d swingStartPos_W;
    Eigen::Vector3d swingDesPosCur_W;
    Eigen::Vector3d swingDesPosCur_L;
    Eigen::Vector3d swingDesPosFinal_W;
    Eigen::Vector3d stanceDesPos_W;
    Eigen::Vector3d posHip_W, posST_W;
    Eigen::Vector3d desV_W; // desired linear velocity
    double desWz_W;         // desired angular velocity
    double theta0;          // offset yaw angle of the swing leg, w.r.t body frame
    double width_hips;      // distance between the left and right hip
    double tSwing;
    double phi;
    enum MotionState
    {
        Stand,
        Walk,
        Walk2Stand
    };
    enum LegState
    {
        LSt,
        RSt,
        DSt // no use but reserverd
    };
    bool leg_contact[2];
    double thetaZ_des{0};
    LegState legState{DataBus::DSt};
    LegState legStateNext{DataBus::DSt};
    MotionState motionState{DataBus::Stand};

    // for jump
    Eigen::Vector3d base_pos_stand;
    Eigen::Matrix<double, 6, 1> pfeW_stand, pfeW0;
    // Eigen::Vector3d mpc_eul_des, mpc_omega_des, mpc_vel_des, mpc_pos_des;

    DataBus(int model_nvIn) : model_nv(model_nvIn)
    {
        motors_pos_cur.assign(model_nv - 6, 0);
        motors_vel_cur.assign(model_nv - 6, 0);
        motors_tor_out.assign(model_nv - 6, 0);
        motors_tor_cur.assign(model_nv - 6, 0);
        motors_tor_des.assign(model_nv - 6, 0);
        motors_vel_des.assign(model_nv - 6, 0);
        motors_pos_des.assign(model_nv - 6, 0);
        q = Eigen::VectorXd::Zero(model_nv + 1);
        qOld = Eigen::VectorXd::Zero(model_nv + 1);
        dq = Eigen::VectorXd::Zero(model_nv);
        ddq = Eigen::VectorXd::Zero(model_nv);
        qCmd = Eigen::VectorXd::Zero(model_nv + 1);
        dqCmd = Eigen::VectorXd::Zero(model_nv);
        tauJointCmd = Eigen::VectorXd::Zero(model_nv - 6);
        FL_est = Eigen::VectorXd::Zero(6);
        FR_est = Eigen::VectorXd::Zero(6);
        Xd = Eigen::VectorXd::Zero(12 * 10);
        X_cur = Eigen::VectorXd::Zero(12);
        X_cal = Eigen::VectorXd::Zero(12);
        dX_cal = Eigen::VectorXd::Zero(12);
        fe_react_tau_cmd = Eigen::VectorXd::Zero(13 * 3);
        Fr_ff = Eigen::VectorXd::Zero(12);
        des_ddq = Eigen::VectorXd::Zero(model_nv);
        des_dq = Eigen::VectorXd::Zero(model_nv);
        des_delta_q = Eigen::VectorXd::Zero(model_nv);
        base_rpy_des.setZero();
        base_pos_des.setZero();
        js_eul_des.setZero();
        js_pos_des.setZero();
        js_omega_des.setZero();
        js_vel_des.setZero();
        motionState = Stand;
        base_vel << 0, 0, 0;
    };

    // update q according to sensor values, must update sensor values before
    void updateQ()
    {
        base_omega_W << baseAngVel[0], baseAngVel[1], baseAngVel[2];
        auto Rcur = eul2Rot(rpy[0], rpy[1], rpy[2]);
        base_omega_W = Rcur * base_omega_W;

        //  q = [global_base_position, global_base_quaternion, joint_positions]
        //  dq = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]

        auto quatNow = eul2quat(rpy[0], rpy[1], rpy[2]);
        q(0) = basePos[0];
        q(1) = basePos[1];
        q(2) = basePos[2];
        q(3) = quatNow.x();
        q(4) = quatNow.y();
        q(5) = quatNow.z();
        q(6) = quatNow.w();
        for (int i = 0; i < model_nv - 6; i++)
            q(i + 7) = motors_pos_cur[i];

        Eigen::Vector3d vCoM_W;
        vCoM_W << baseLinVel[0], baseLinVel[1], baseLinVel[2];
        dq.block<3, 1>(0, 0) = vCoM_W;
        dq.block<3, 1>(3, 0) << base_omega_W[0], base_omega_W[1], base_omega_W[2];
        //        dq.block<3,1>(3,0) << baseAngVel[0],baseAngVel[1],baseAngVel[2];
        for (int i = 0; i < model_nv - 6; i++)
        {
            dq(i + 6) = motors_vel_cur[i];
        }

        base_pos << q(0), q(1), q(2);
        base_rpy << rpy[0], rpy[1], rpy[2];
        base_rot = Rcur;
        qOld = q;
    }

    static void printdq(const Eigen::VectorXd &q)
    {
        std::cout << std::setprecision(5) << q.block<6, 1>(0, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(6, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(13, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<4, 1>(20, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(24, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(30, 0).transpose() << std::endl;
    }

    static void printq(const Eigen::VectorXd &q)
    {
        std::cout << std::setprecision(5) << q.block<7, 1>(0, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(7, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<7, 1>(14, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<4, 1>(21, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(25, 0).transpose() << std::endl;
        std::cout << std::setprecision(5) << q.block<6, 1>(31, 0).transpose() << std::endl;
    }

    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw)
    {
        Eigen::Matrix<double, 3, 3> Rx, Ry, Rz;
        Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
        Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
        Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
        return Rz * Ry * Rx;
    }

    Eigen::Quaterniond eul2quat(double roll, double pitch, double yaw)
    {
        Eigen::Matrix3d R = eul2Rot(roll, pitch, yaw);
        Eigen::Quaternion<double> quatCur;
        quatCur = R; // rotation matrix converted to quaternion
        Eigen::Quaterniond resQuat;
        resQuat = quatCur;
        return resQuat;
    }
};
