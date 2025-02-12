/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "PVT_ctrl.h"

PVT_Ctr::PVT_Ctr(double timeStepIn, const char *jsonPath) {
    jointNum=motorName.size();

    tau_out_lpf.assign(jointNum,LPF_Fst());
    motor_vel.assign(jointNum,0);
    motor_pos_cur.assign(jointNum,0);
    motor_pos_des_old.assign(jointNum,0);
    motor_tor_out_link.assign(jointNum,0);
    motor_tor_out_motor.assign(jointNum,0);
    pvt_Kp.assign(jointNum,0);
    pvt_Kd.assign(jointNum,0);
    maxTor.assign(jointNum,400);
    maxVel.assign(jointNum,50);
    maxPos.assign(jointNum,3.14);
    minPos.assign(jointNum,-3.14);
    PV_enable.assign(jointNum,1);
    gear.assign(jointNum,1.0);

    // read joint pvt parameters
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in(jsonPath,std::ios::binary);

    reader.parse(in,root_read);
    for (int i=0;i<jointNum;i++){
        pvt_Kp[i]=root_read[motorName[i]]["kp"].asDouble();
        pvt_Kd[i]=root_read[motorName[i]]["kd"].asDouble();
        maxTor[i]=root_read[motorName[i]]["maxTorque"].asDouble();
        maxVel[i]=root_read[motorName[i]]["maxSpeed"].asDouble();
        maxPos[i]=root_read[motorName[i]]["maxPos"].asDouble();
        minPos[i]=root_read[motorName[i]]["minPos"].asDouble();
        double fc=root_read[motorName[i]]["PVT_LPF_Fc"].asDouble();
        gear[i] = root_read[motorName[i]]["gear"].asDouble();
        tau_out_lpf[i].setPara(fc, timeStepIn);
        tau_out_lpf[i].ftOut(0);
    }
}

void PVT_Ctr::dataBusRead(DataBus &busIn) {
    for (int i=0;i<jointNum;i++)
    {
        motor_pos_cur[i]=busIn.motors_pos_cur[i];
        motor_vel[i]=busIn.motors_vel_cur[i];
    }
    motor_pos_des=busIn.motors_pos_des;
    motor_vel_des=busIn.motors_vel_des;
    motor_tor_des=busIn.motors_tor_des;
}

void PVT_Ctr::dataBusWrite(DataBus &busIn) {
    busIn.motors_tor_out=motor_tor_out_motor;
    busIn.motors_tor_cur=motor_tor_out_link;
}

void PVT_Ctr::setJointPD(double kp, double kd, const char *jointName) {
    auto it = std::find(motorName.begin(), motorName.end(), jointName);

    int id=-1;
    if (it != motorName.end()) {
        id = std::distance(motorName.begin(), it);
    } else {
        std::cout << jointName << " NOT found!" << std::endl;
    }
    pvt_Kp[id]=kp;
    pvt_Kd[id]=kd;
}

// joint pvt control
void PVT_Ctr::calMotorsPVT() {
    for (int i=0;i<jointNum;i++)
    {
        double tauDes{0};
        tauDes=PV_enable[i]*pvt_Kp[i]*(motor_pos_des[i]-motor_pos_cur[i])+PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        tauDes=tau_out_lpf[i].ftOut(tauDes)+motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes= sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
        motor_pos_des_old[i]=motor_pos_des[i];
    }
}

// joint pvt control with delta position limit
void PVT_Ctr::calMotorsPVT(double deltaP_Lim) {
    for (int i=0;i<jointNum;i++)
    {
        double tauDes{0};
        double delta=motor_pos_des[i]-motor_pos_des_old[i];
        if (fabs(delta)>= fabs(deltaP_Lim))
            delta=deltaP_Lim * sign(delta);
        double pDes=delta+motor_pos_des_old[i];
        tauDes=PV_enable[i]*pvt_Kp[i]*(pDes-motor_pos_cur[i])+PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        tauDes=tau_out_lpf[i].ftOut(tauDes)+motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes= sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
        motor_pos_des_old[i]=pDes;
    }
}

double PVT_Ctr::sign(double in) {
    if (in>=0)
        return 1.0;
    else
        return -1.0;
}

// Enable PV control item for all joints. Note the PV control item is default enabled, no need for calling this function
void PVT_Ctr::enablePV() {
    PV_enable.assign(jointNum,1);
}

void PVT_Ctr::enablePV(int jtId) {
    PV_enable[jtId]=1;
}

// Disable PV control item for all joints.
void PVT_Ctr::disablePV() {
    PV_enable.assign(jointNum,0);
}

void PVT_Ctr::disablePV(int jtId) {
    PV_enable[jtId]=0;
}