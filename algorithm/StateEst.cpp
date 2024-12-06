//
// Created by boxingwang on 22-12-1.
//

#include "StateEst.h"
#include "iostream"
#include "data_bus.h"
#include <chrono>

using namespace Eigen;

StateEst::StateEst(double dtIn):
		eul_w_filter(dtIn)
{
    dt = dtIn;

    Matrix<double, 6, 6> blkA;
    Matrix<double, 6, 3> blkA_a;
    A.setZero();
    B.setZero();
    C.setZero();

    A.block<3, 3>(0, 0) = Matrix<double, 3, 3>::Identity();
    A.block<3, 3>(0, 3) = Matrix<double, 3, 3>::Identity() * dt;
    A.block<3, 3>(3, 3) = Matrix<double, 3, 3>::Identity();
    A.block<6, 6>(6, 6) = Matrix<double, 6, 6>::Identity();
    A.block<3, 3>(12, 12) = Matrix<double, 3, 3>::Identity();
    A.block<3, 3>(3, 12) = Matrix<double, 3, 3>::Identity() * dt;

    B.block<3, 3>(3, 0) = Matrix<double, 3, 3>::Identity() * dt;

    Matrix<double, 2, 6> e;

    e.setZero();
    e(0, 2) = 1;
    e(1, 5) = 1;
    C.block<3, 3>(0, 0) = Matrix<double, 3, 3>::Identity();
    C.block<3, 3>(3, 0) = Matrix<double, 3, 3>::Identity();
    C.block<3, 3>(0, 6) = -Matrix<double, 3, 3>::Identity();
    C.block<3, 3>(3, 9) = -Matrix<double, 3, 3>::Identity();

    C.block<3, 3>(6, 3) = -Matrix<double, 3, 3>::Identity();
    C.block<3, 3>(9, 3) = -Matrix<double, 3, 3>::Identity();
    C.block<2, 6>(12, 6) = e;

    //    std::cout<<"A"<<std::endl<<A;
    //    std::cout<<"B"<<std::endl<<B;
    //    std::cout<<"C"<<std::endl<<C;

    P.setIdentity();
    X.setZero();
    P0.setIdentity();
    X0.setZero();
    Q.setZero();
    Qu.setZero();
    R.setZero();
    peB_old.setZero();
    peW_old.setZero();

    //    std::cout<<"A"<<std::endl<<A<<std::endl;
    //    std::cout<<"B"<<std::endl<<B<<std::endl;
    //    std::cout<<"C"<<std::endl<<C<<std::endl;

    vCoM_LP[0] = 0;
    vCoM_LP[1] = 0;
    vCoM_LP[2] = 0;

    eul_w_filter.Q.setIdentity();
    eul_w_filter.Q(0, 0) = 1e-8;
    eul_w_filter.Q(1, 1) = 1e-8;
    eul_w_filter.Q(2, 2) = 1e-8;
    eul_w_filter.Q(3, 3) = 5e-7;
    eul_w_filter.Q(4, 4) = 5e-7;
    eul_w_filter.Q(5, 5) = 2e-5;

    eul_w_filter.R.setIdentity();
    eul_w_filter.R(0, 0) = 1e-6;
    eul_w_filter.R(1, 1) = 1e-6;
    eul_w_filter.R(2, 2) = 1e-6;
    eul_w_filter.R(3, 3) = 1e-4; // 1e-4;//
    eul_w_filter.R(4, 4) = 1e-4; // 1e-3;//
    eul_w_filter.R(5, 5) = 1e-5; // 1e-3;//
}

void StateEst::init(DataBus &Data)
{
    offYaw = Data.rpy[2];
    printf("yawoffset: %.4f, %.4f, %.4f\n", Data.rpy[0], Data.rpy[1], Data.rpy[2]);
    Eigen::Matrix3d R;
    R = eul2Rot(Data.base_rpy[0], Data.base_rpy[1], 0);
    Eigen::Vector3d pWL, pWR;
    pWL = R * fe_l_pos_L;
    pWR = R * fe_r_pos_L;
    double zOff = -(pWR(2) + pWL(2)) / 2.0 + 0.07;
    X0 << 0, 0, zOff,
        0, 0, 0,
        pWL(0), pWL(1), 0.07,
        pWR(0), pWR(1), 0.07,
        0, 0, 0;
    P0.setZero();
    for (int i = 0; i < 15; i++)
        P0(i, i) = 0.1; //????????????????????????
    flag_init = false;
    //    startFlag = true;
    X = X0;
    P = P0;
    // kalman filter para settings
    TR_kh = 100;
    TR_k = 100;

    KF_Q_wPCoM << 5e-6, 5e-6, 5e-6;
    // KF_Q_wVCoM << 1e-7,1e-7,1e-6;
    KF_Q_wVCoM << 1e-8, 2e-8, 1e-6;
    KF_Q_wfeW << 1e-6, 1e-6, 1e-6,
        1e-6, 1e-6, 1e-6;
    //	KF_Q_wfeW <<  5e-7,5e-7,5e-7,
    //			5e-7,5e-7,5e-7;

    KF_Q_waL << 1e-8, 1e-8, 1e-8;

    KF_R_wfeL << 1e-7, 1e-7, 1e-10,
        6e-7, 8e-8, 1e-11;
    //	KF_R_wfeL << 1e-2, 1e-2, 1e-5,
    //				1e-2, 8e-3, 1e-6;

    KF_R_wdfeL << 1e-5, 2e-5, 1e-6,
        1e-5, 2e-5, 1e-6;

    KF_R_wh = Eigen::Matrix<double, 2, 1>::Ones() * 1e-3;

    KF_Q_waU << 2e-4, 2e-4, 8e-4;
}

void StateEst::set(DataBus &Data)
{
    if (flag_init)
        offYaw = Data.rpy[2];
    acc << Data.baseAcc[0], Data.baseAcc[1], Data.baseAcc[2];
    // printf("acc: %.4f, %.4f, %.4f\n", acc[0], acc[1], acc[2]);
    eul << Data.rpy[0], Data.rpy[1] - d2r(0.0), Data.rpy[2];
    eul_woOff << eul[0], eul[1], eul[2] - offYaw;
    Rrpy_woOff = eul2Rot(eul_woOff(0), eul_woOff(1), eul_woOff(2));

    omegaL << Data.baseAngVel[0], Data.baseAngVel[1], Data.baseAngVel[2];
    omegaW = Data.base_omega_W;

    //--------vel, omega filter--------------------
    // double eul_ary[3] = {eul(0), eul(1), eul(2)};
    double eul_woOff_ary[3] = {eul_woOff(0), eul_woOff(1), eul_woOff(2)};
    double omegaL_ary[3] = {omegaL(0), omegaL(1), omegaL(2)};
    eul_w_filter.run(eul_woOff_ary, omegaL_ary);
    eul_w_filter.getData(Eul_filtered, wL_filtered);

    eul_woOff << Eul_filtered[0], Eul_filtered[1], Eul_filtered[2]; // without offset
    eul << eul_woOff[0], eul_woOff[1], eul_woOff[2] + offYaw;       // have offset
    Rrpy_woOff = eul2Rot(eul_woOff(0), eul_woOff(1), eul_woOff(2));
    Rrpy = eul2Rot(eul(0), eul(1), eul(2));
    omegaL << wL_filtered[0], wL_filtered[1], wL_filtered[2];
    omegaW = Rrpy_woOff * omegaL;
    //--------vel, omega filter--------------------

    Vector3d accTmp = {acc[0], acc[1], acc[2]};
    Vector3d g = {0.0, 0.0, 9.81};
    freeAcc = Rrpy_woOff * accTmp - g;

    phi = Data.phi;
    legState = Data.legState;
    fe_l_pos_L = Data.fe_l_pos_L;
    fe_r_pos_L = Data.fe_r_pos_L;
    fe_l_vel_L = Data.fe_l_vel_L;
    fe_r_vel_L = Data.fe_r_vel_L;
    fe_l_pos_W = Data.fe_l_pos_W;
    fe_r_pos_W = Data.fe_r_pos_W;
}

void StateEst::getTrustRegion_wt_h()
{
    Matrix<double, 6, 1> C, Cv;
    Vector2d Ch;
    double aa[2];
    double bb[2]{5.0, 5.0};
    for (int i = 0; i < 2; i++)
    {
        if (legState == DataBus::Stand)
            aa[i] = 1.0;
        else
        {
            aa[i] = phi * bb[i];
            if (aa[i] > 1.0)
                aa[i] = 1.0;
            aa[i] = (1.0 - aa[i]) * 10000.0;
            aa[i] = aa[i] + 1.0;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        C(i * 3) = leg_contact[i] + (1 - leg_contact[i]) * TR_k;
        C(i * 3 + 1) = leg_contact[i] + (1 - leg_contact[i]) * TR_k;
        C(i * 3 + 2) = leg_contact[i] + (1 - leg_contact[i]) * TR_k;
        Cv(i * 3) = leg_contact[i] * aa[0] + (1 - leg_contact[i]) * TR_k;
        Cv(i * 3 + 1) = leg_contact[i] + (1 - leg_contact[i]) * TR_k;
        Cv(i * 3 + 2) = leg_contact[i] + (1 - leg_contact[i]) * TR_k;
        Ch(i) = leg_contact[i] + (1 - leg_contact[i]) * TR_kh;
    }
    Xi = C.asDiagonal();
    Xih = Ch.asDiagonal();
    Xiv = Cv.asDiagonal();
}

// for qFB_all:=[qL[1-4],qPas[1-2],qL[5],qR[1-4],qPas[3-4],qR[5]]; feedback positions whose offset is defined in URDF
void StateEst::update()
{
    // if (FRest[2] > FcontactUpp[1])
    //     leg_contact[1] = true;
    // else if (FRest[2] < FcontactLow[1])
    //     leg_contact[1] = false;
    // if (FLest[2] > FcontactUpp[0])
    //     leg_contact[0] = true;
    // else if (FLest[2] < FcontactLow[0])
    //     leg_contact[0] = false;

    if (legState == DataBus::LSt){// && FRest[2] >= FzThrehold[1] && phi>=0.6) {
        leg_contact[0] = true;
        leg_contact[1] = false;
    }
    else if (legState == DataBus::RSt){// && FLest[2] >= FzThrehold[0] && phi>=0.6) {
        leg_contact[1] = true;
        leg_contact[0] = false;
    }
    else {
        leg_contact[0] = true;
        leg_contact[1] = true;
    }

    peB.block<3, 1>(0, 0) = fe_l_pos_L;
    peB.block<3, 1>(0, 1) = fe_r_pos_L;

    getTrustRegion_wt_h();

    // prediction
    Eigen::Matrix<double, 15, 1> diagQ;
    Eigen::Matrix<double, 14, 1> diagR;
    diagQ.setZero();
    diagR.setZero();
    diagQ.segment<3>(0) = KF_Q_wPCoM;
    diagQ.segment<3>(3) = KF_Q_wVCoM;
    diagQ.segment<6>(6) = Xi * KF_Q_wfeW;
    diagQ.segment<3>(12) = KF_Q_waL;

    diagR.segment<6>(0) = KF_R_wfeL;
    diagR.segment<6>(6) = Xiv * KF_R_wdfeL; //??????
    diagR.segment<2>(12) = Xih * KF_R_wh;

    Qu = B * KF_Q_waU.asDiagonal() * B.transpose();

    Q = diagQ.asDiagonal().toDenseMatrix() + Qu;
    // Q=diagQ.asDiagonal().toDenseMatrix();
    R = diagR.asDiagonal();

    Matrix<double, 14, 14> S_inv;

    X = A * X + B * freeAcc;
    P = A * P * A.transpose() + Q;

    // measurement evaluation
    Matrix<double, 3, 1> omegaLVec;
    omegaLVec << wL_filtered[0], wL_filtered[1], wL_filtered[2];
    pbW = Rrpy_woOff * peB;
    Eigen::Matrix<double, 3, 2> velTmp;
    velTmp.block(0, 0, 3, 1) = fe_l_vel_L;
    velTmp.block(0, 1, 3, 1) = fe_r_vel_L;
    for (int i = 0; i < 2; i++)
    {
        // vbW.col(i) = leg_contact[i] * Rrpy_woOff * ((peB.col(i) - peB_old.col(i)) / dt + omegaLVec.cross(peB.col(i)))
        //      + (1 - leg_contact[i]) * (-base_vel);
        vbW.col(i) = leg_contact[i] * Rrpy_woOff * (velTmp.col(i) + omegaLVec.cross(peB.col(i))) + (1 - leg_contact[i]) * (-base_vel);
        Y(12 + i) = leg_contact[i] * 0.07 + (1 - leg_contact[i]) * (base_pos(2) + pbW.col(i)(2));
    }
    if (peB_old.isZero() || peW_old.isZero())
        vbW.setZero();
    Y.segment<3>(0) = -pbW.block<3, 1>(0, 0);
    Y.segment<3>(3) = -pbW.block<3, 1>(0, 1);
    Y.segment<3>(6) = vbW.block<3, 1>(0, 0);
    Y.segment<3>(9) = vbW.block<3, 1>(0, 1);
    //    Y.segment<6>(0)=-pbW.reshaped();
    //    Y.segment<6>(6)=vbW.reshaped();

    peB_old = peB;
    peW_old = pbW;

    // correction according to measurement
    S_inv = C * P * C.transpose() + R;
    S_inv = S_inv.inverse();

    K = P * C.transpose() * S_inv;

    P = (Matrix<double, 15, 15>::Identity() - K * C) * P;
    X = X + K * (Y - C * X);

    if (!startFlag)
    {
        X = X0;
        P = P0;
    }

    if (!flag_init)
        startFlag = true;

    //    peW=X.segment<6>(6).reshaped(3,2);
    peW.block<3, 1>(0, 0) = X.segment<3>(6);
    peW.block<3, 1>(0, 1) = X.segment<3>(9);
    base_pos = X.segment<3>(0);
    base_vel = X.segment<3>(3);
    delta_acc = X.segment<3>(12);
    fe_l_pos_W = peW.block<3, 1>(0, 0);
    fe_r_pos_W = peW.block<3, 1>(0, 1);

    //    vCoM<<vxLP_1O.run(X(3)),vyLP_1O.run(X(4)),vzLP_1O.run(X(5));//?????????????????

    //    // reduce position drift
    //    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
    //        P.block<2, 10>(0, 2).setZero();
    //        P.block<10, 2>(2, 0).setZero();
    //        P.block<2, 2>(0, 0) /= 10.0;
    //    }
}

void StateEst::get(DataBus &Data)
{
    Data.base_pos_est = base_pos;
    Data.base_vel_est = base_vel; // Rrpy.transpose()*
    Data.fe_l_pos_W_est = fe_l_pos_W;
    Data.fe_r_pos_W_est = fe_r_pos_W;
    Data.delta_acc = delta_acc;
    Data.eul_est = eul_woOff; // attention!!!! without yaw offset!!!!
    Data.omegaW_est = omegaW;

    Data.base_pos = Data.base_pos_est;
    Data.base_vel = Data.base_vel_est;
    // Data.fe_l_pos_W = Data.fe_l_pos_W_est;
    // Data.fe_r_pos_W = Data.fe_r_pos_W_est;
    Data.baseAcc[0] += delta_acc[0];
    Data.baseAcc[1] += delta_acc[1];
    Data.baseAcc[2] += delta_acc[2];
    Data.base_rpy = Data.eul_est;
    Data.base_omega_W = Data.omegaW_est;
    Data.base_rot = Rrpy_woOff; // calculate by  eul_woOff

    Data.q.block<3, 1>(0, 0) = Data.base_pos;
    Data.dq.block<3, 1>(0, 0) = Data.base_vel;

    auto quatNow = eul2quat(Data.base_rpy[0], Data.base_rpy[1], Data.base_rpy[2]);
    Data.q(3) = quatNow.x();
    Data.q(4) = quatNow.y();
    Data.q(5) = quatNow.z();
    Data.q(6) = quatNow.w();
    Data.dq.block<3, 1>(3, 0) = Data.base_omega_W;
    //======================TEST EST===================
    Data.AX = A * X;
    Data.BU = B * freeAcc;
    Data.freeAcc = freeAcc;
    Data.CX = C * X;
    Data.Y = Y;

    Data.pbW = pbW.reshaped();

    Data.leg_contact[0] = leg_contact[0];
    Data.leg_contact[1] = leg_contact[1];
    //=================================================
}

void StateEst::setF(DataBus &Data)
{
    model_nv = Data.model_nv;
    torJoint = Eigen::VectorXd::Zero(model_nv - 6);
    for (int i = 0; i < model_nv - 6; i++)
    {
        torJoint[i] = Data.motors_tor_cur[i];
    }
    dyn_M = Data.dyn_M;
    dyn_Non = Data.dyn_Non;
    J_l = Data.J_l;
    dJ_l = Data.dJ_l;
    J_r = Data.J_r;
    dJ_r = Data.dJ_r;
    dq = Data.dq;
}
void StateEst::updateF()
{
    Eigen::VectorXd tauAll;
    tauAll = Eigen::VectorXd::Zero(model_nv);
    tauAll.block(6, 0, model_nv - 6, 1) = torJoint;
    FLest = -pseudoInv_SVD(J_l * dyn_M.inverse() * J_l.transpose()) * (J_l * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_l * dq);
    FRest = -pseudoInv_SVD(J_r * dyn_M.inverse() * J_r.transpose()) * (J_r * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_r * dq);
}

void StateEst::getF(DataBus &Data)
{
    Data.FL_est = FLest;
    Data.FR_est = FRest;
}

double StateEst::CphiFun(double phi, double s, double W)
{
    return s * (erf(12 * phi / W - 6) + erf(12 * (1 - phi) / W - 6) - 1);
}
double StateEst::CzFun(double pz, double kp, double kn)
{
    double out;
    if (pz >= 0)
        out = exp(-kp * pz * pz);
    else
        out = exp(-kn * pz * pz);

    return out;
}
double StateEst::erf(double xIn)
{
    return 1 / (1 + exp(-xIn));
}

bool StateEst::get_init()
{
    return flag_init;
}
