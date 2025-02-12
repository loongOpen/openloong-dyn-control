/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "foot_placement.h"
#include "bezier_1D.h"

void FootPlacement::dataBusRead(DataBus &robotState)
{
    posStart_W = robotState.swingStartPos_W;
    desV_W = robotState.js_vel_des;
    desWz_W = robotState.js_omega_des(2);
    curV_W = robotState.dq.block<3, 1>(0, 0);
    phi = robotState.phi;
    hipPos_W = robotState.posHip_W;
    STPos_W = robotState.posST_W;
    base_pos = robotState.base_pos;
    tSwing = robotState.tSwing;
    theta0 = robotState.theta0;
    yawCur = robotState.rpy[2];
    omegaZ_W = robotState.base_omega_W(2);
    hip_width = robotState.width_hips;
    legState = robotState.legState;
}

void FootPlacement::dataBusWrite(DataBus &robotState)
{
    robotState.swingDesPosCur_W << pDesCur[0], pDesCur[1], pDesCur[2];
    robotState.swingDesPosFinal_W = posDes_W;
    robotState.swing_fe_rpy_des_W << 0, 0, robotState.base_rpy_des(2); // WARNING! ThetaZ!
    robotState.swing_fe_pos_des_W << pDesCur[0], pDesCur[1], pDesCur[2];
}
void FootPlacement::getSwingPos()
{
    Eigen::Matrix<double, 4, 1> b;
    b.setZero();
    Eigen::Matrix<double, 1, 4> xNow;
    xNow << 1, phi, pow(phi, 2), pow(phi, 3);

    Eigen::Matrix3d KP, Rz;
    KP.setZero();
    KP(0, 0) = kp_vx;
    KP(1, 1) = kp_vy;
    KP(2, 2) = 0;
    Rz << cos(yawCur), -sin(yawCur), 0,
        sin(yawCur), cos(yawCur), 0,
        0, 0, 1;
    KP = Rz * KP * Rz.transpose();

    // for linear velocity
    posDes_W = hipPos_W + KP * (desV_W - curV_W) * (-1) + 0.5 * tSwing * curV_W +
               curV_W * (1 - phi) * tSwing;

    // for angular veloctity
    double thetaF;
    thetaF = yawCur + theta0 + omegaZ_W * (1 - phi) * tSwing + 0.5 * omegaZ_W * tSwing + kp_wz * (omegaZ_W - desWz_W);
    posDes_W(0) += 0.5 * hip_width * (cos(thetaF) - cos(yawCur + theta0));
    posDes_W(1) += 0.5 * hip_width * (sin(thetaF) - sin(yawCur + theta0));

    double xOff_L = -0.07;  //-0.01; // foot-end position offset in x direction in body frame
    double yOff_L = 0.04;   // 0.01; // foot-end position offset in y direction in body frame, positive for moving the leg inside
    double zOff_W = -0.035; // foot-end position offset in z direction in world frame

    //    posDes_W(2)=STPos_W(2)-0.04;
    posDes_W(2) = base_pos(2) - legLength + zOff_W;

    double xOff_W(0), yOff_W(0);
    if (legState == DataBus::LSt)
    {
        xOff_W = cos(yawCur) * xOff_L - sin(yawCur) * yOff_L;
        yOff_W = sin(yawCur) * xOff_L + cos(yawCur) * yOff_L;
        // yOff_W = 0.05;
    }
    else if (legState == DataBus::RSt)
    {
        xOff_W = cos(yawCur) * xOff_L - sin(yawCur) * (-yOff_L);
        yOff_W = sin(yawCur) * xOff_L + cos(yawCur) * (-yOff_L);
        // yOff_W = -0.05;
    }

    posDes_W(0) += xOff_W;
    posDes_W(1) += yOff_W;
    //
    //    double yOff=0.005; // positive for moving the leg inside
    //    if (legState==DataBus::LSt)
    //        posDes_W(1)+=yOff;
    //    else if (legState==DataBus::RSt)
    //        posDes_W(1)-=yOff;

    // cycloid trajectories
    if (phi < 1.0)
    {
        pDesCur[0] = posStart_W(0) + (posDes_W(0) - posStart_W(0)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
        pDesCur[1] = posStart_W(1) + (posDes_W(1) - posStart_W(1)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi));
    }

    if (phi >= 0.98)
    {
        zStretch += -0.002;

        // std::cout << "---------------- " << zStretch << std::endl;
    }
    else
        zStretch = 0;
    if (zStretch < -0.05)
    {
        zStretch = -0.05;
        // finish_Stretch = true;
    }

    // if (phi < 1e-3)
    // {
    //     finish_Stretch = false;
    // }

    // pDesCur[2] = posStart_W(2) + stepHeight * 0.5 * (1 - cos(2 * 3.1415 * phi)) + (posDes_W(2) - posStart_W(2)) / (2 * 3.1415) * (2 * 3.1415 * phi - sin(2 * 3.1415 * phi))+ zStretch;
    pDesCur[2] = posStart_W(2) + Trajectory(0.2, stepHeight, posDes_W(2) - posStart_W(2)) + zStretch;
}

double FootPlacement::Trajectory(double phase, double hei, double len)
{
    Bezier_1D Bswpid;
    double para0 = 5, para1 = 3;
    for (int i = 0; i < para0; i++)
    {
        Bswpid.P.push_back(0.0);
    }
    for (int i = 0; i < para1; i++)
    {
        Bswpid.P.push_back(1.0);
    }

    double output;
    if (phi < phase)
    {
        output = hei * Bswpid.getOut(phi / phase);
    }
    else
    {
        double s = Bswpid.getOut((1.4 - phi) / (1.4 - phase));
        if (s > 0)
        {
            output = hei * s + len * (1.0 - s);
        }
        else
        {
            output = len;
        }
    }
    return output;
}
