//
// Created by boxingwang on 22-12-1.
//

#ifndef BIPED_STATEEST_STATEEST_H
#define BIPED_STATEEST_STATEEST_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "useful_math.h"
#include "data_bus.h"
#include "Eul_W_filter.h"

class StateEst {
public:
    StateEst(double dtIn);
    bool get_init();
    void init(DataBus &Data); // set X0 and P0
    void set(DataBus &Data);
    void update();
    void get(DataBus &Data);
    void setF(DataBus &Data);
    void updateF();
    void getF(DataBus &Data);
//private:
    double dt;
    bool flag_init = true;
    bool startFlag = false;

    double phi;
    double legState;
    Eigen::Matrix<double, 3,1>  fe_l_pos_L, fe_r_pos_L;
    Eigen::Matrix<double, 3,1>  fe_l_vel_L, fe_r_vel_L;
    Eigen::Matrix<double, 3,1>  fe_l_pos_W, fe_r_pos_W;
    Eigen::Matrix<double, 3,1>  delta_acc;

//    Eigen::Matrix<double, 3,1> pBR, pBL;
    Eigen::Matrix<double, 3,1> pCoM,vCoM;
    Eigen::Matrix<double, 3,1> base_pos,base_vel;
    Eigen::Matrix<double, 3,1> acc, eul;
    Eigen::Matrix<double, 3,1> freeAcc, eul_woOff, omegaW; // free acceleration, euler angle wo offset, angular speed in world frame
    Eigen::Matrix<double, 3,1> omegaL;
    Eigen::Matrix<double, 3,3> Rrpy_woOff, Rrpy; // woOff: without yaw offset !!!!!
    Eigen::Matrix<double, 3,1> pbl2com_bl;
    Eigen::Matrix<double,3,2> pbW, vbW;
    Eigen::Matrix<double,3,2> peB, peB_old, peW, peW_old; // foot-end position w.r.t the body and the world frame

    Eigen::Matrix<double,2,2> Xih;
    Eigen::Matrix<double,6,6> Xi;
    Eigen::Matrix<double,6,6> Xiv;

    Eigen::Matrix<double,15,15> A;
    Eigen::Matrix<double,15,3> B;
    Eigen::Matrix<double,14,15> C;
    Eigen::Matrix<double,15,1> X, X0;
    Eigen::Matrix<double,15,15> P, P0;
    Eigen::Matrix<double,14,1> Y;

    Eigen::Matrix<double,15,15> Q,Qu;
    Eigen::Matrix<double,14,14> R;
    Eigen::Matrix<double,15,14> K;

    void getTrustRegion_wt_h();

    static double CphiFun(double phi,double s,double W);
    static double CzFun(double pz,double kp,double kn);
    static double erf(double xIn);


//    //LowPass_filter vxLP,vyLP,vzLP;
//    LowPass_filter_1O vxLP_1O{dt,50};
//    LowPass_filter_1O vyLP_1O{dt,50};
//    LowPass_filter_1O vzLP_1O{dt,50};

    Eul_W_filter eul_w_filter{dt};
    double Eul_filtered[3],wL_filtered[3];

    double vCoM_LP[3];

    double offYaw=0;
    bool leg_contact[2]={true,true}; //first for right, phase-based leg touch-down indicator

    // trust_region para
    double TR_Cphi_w=0.1; // para for trust region
    double TR_Cz_kp=100;  // para for trust region
    double TR_Cz_kn=20;  // para for trust region
    double TR_k=100; // for stance hold still and foot-end velocity
    double TR_kh=100; // for stance foot-end height

    // kalman filter para
    Eigen::Vector3d KF_Q_wPCoM; // [1,1,1]*pow(10.,-7.)
    Eigen::Vector3d KF_Q_wVCoM; // [1,1,1]*pow(10.,-8.)
    Eigen::Matrix<double,6,1> KF_Q_wfeW; // ones(6,1)*pow(10.,-10.)
    Eigen::Vector3d KF_Q_waL; // [1.5298;1.5967;2.5534] * (1e-4)
    Eigen::Vector3d KF_Q_waU;
    Eigen::Matrix<double,6,1> KF_R_wfeL; // ones( 6, 1 ) * (1e-7)
    Eigen::Matrix<double,6,1> KF_R_wdfeL; // ones( 6, 1 ) * (1e-7)
    Eigen::Matrix<double,2,1> KF_R_wh; // ones( 2, 1 ) * (1e-5)

    Eigen::VectorXd dq;
    Eigen::Vector<double,6> FLest,FRest;
    Eigen::VectorXd torJoint;
    int model_nv;
    Eigen::MatrixXd dyn_M, dyn_Non, J_l, J_r, dJ_l, dJ_r;
    bool    legcontact[2]{false, false};
    double  FcontactUpp[2]{280, 280};
    double  FcontactLow[2]{10, 10};
};

#endif //BIPED_STATEEST_STATEEST_H
