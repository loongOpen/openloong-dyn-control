//
// Created by boxing on 23-7-10.
//

#include "Eul_W_filter.h"

using namespace Eigen;

Eul_W_filter::Eul_W_filter(double dtIn) {
    dt=dtIn;
    P.setIdentity();
    H.setIdentity();
    F.setIdentity();
}

void Eul_W_filter::run(double *eulIn, double *wLIn) {
    kal_Z<<eulIn[0],eulIn[1],eulIn[2],wLIn[0],wLIn[1],wLIn[2];
    if (isIni== false)
    {
        kal_X<<kal_Z;
        Eul_filtered[0]=kal_X(0);
        Eul_filtered[1]=kal_X(1);
        Eul_filtered[2]=kal_X(2);
        wL_filtered[0]=kal_X(3);
        wL_filtered[1]=kal_X(4);
        wL_filtered[2]=kal_X(5);
        isIni= true;
        return;
    }
    double roll_Old, pitch_Old,yaw_Old;
    roll_Old=kal_X(0);
    pitch_Old=kal_X(1);
    yaw_Old=kal_X(2);
    F.setIdentity();
    F(0,3)=dt;
    F(0,4)=dt*sin(roll_Old)*sin(pitch_Old)/ cos(pitch_Old);
    F(0,5)=dt*cos(roll_Old)*sin(pitch_Old)/ cos(pitch_Old);
    F(1,4)=dt*cos(roll_Old);
    F(1,5)=-dt*sin(roll_Old);
    F(2,4)=dt*sin(roll_Old)/cos(pitch_Old);
    F(2,5)=dt*cos(roll_Old)/cos(pitch_Old);

    kal_X = F * kal_X;
    P=F*P*F.transpose()+Q;

    Matrix<double,6,6> S,K;
    kal_Y=kal_Z-H*kal_X;
    S=H*P*H.transpose()+R;
    K=P*H.transpose()*S.inverse();
    kal_X=kal_X+K*kal_Y;
    P=(Matrix<double,6,6>::Identity()-K*H)*P;

    Eul_filtered[0]=kal_X(0);
    Eul_filtered[1]=kal_X(1);
    Eul_filtered[2]=kal_X(2);
    wL_filtered[0]=kal_X(3);
    wL_filtered[1]=kal_X(4);
    wL_filtered[2]=kal_X(5);
}

void Eul_W_filter::getData(double *eulOut, double *wLOut) {
    for (int i=0;i<3;i++)
    {
        eulOut[i]=Eul_filtered[i];
        wLOut[i]=wL_filtered[i];
    }
}