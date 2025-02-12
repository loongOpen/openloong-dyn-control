/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "data_bus.h"
#include <Eigen/Dense>
#include "useful_math.h"

class GaitScheduler {
public:
    bool isIni{false};
	bool start_walk{true};
    double phi{0};
    double tSwing{0.4};
    double dt{0.001};
    double FzThrehold{100};
    double Fz_L_m{0}, Fz_R_m{0};
    DataBus::LegState firstleg, legState, legStateNext;
    DataBus::MotionState motionState;
    GaitScheduler(double tSwingIn, double dtIn);
    void dataBusRead(const DataBus &robotState);
    void dataBusWrite(DataBus &robotState);
    void step();
    void stop();
	void start();
    Eigen::VectorXd FLest,FRest;
    Eigen::VectorXd torJoint;

    bool enableNextStep;
    bool touchDown; // touch down event indicator
    int stepNumDes{1}, stepNumCur{0};
private:
    Eigen::VectorXd fe_r_pos_W, fe_l_pos_W, swingStartPos_W, posHip_W, posST_W, hip_r_pos_W, hip_l_pos_W, dq;
    Eigen::VectorXd stanceStartPos_W;
    Eigen::MatrixXd fe_r_rot_W, fe_l_rot_W;
    Eigen::MatrixXd dyn_M, dyn_Non, J_l, J_r, dJ_l, dJ_r;
    double theta0;
    int model_nv;

};



