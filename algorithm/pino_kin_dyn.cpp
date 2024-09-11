/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "pino_kin_dyn.h"

#include <utility>

Pin_KinDyn::Pin_KinDyn(std::string urdf_pathIn)
{
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_biped);
    pinocchio::urdf::buildModel(urdf_pathIn, model_biped_fixed);
    data_biped = pinocchio::Data(model_biped);
    data_biped_fixed = pinocchio::Data(model_biped_fixed);
    model_nv = model_biped.nv;
    J_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_r = Eigen::MatrixXd::Zero(6, model_nv);
    J_l_body = Eigen::MatrixXd::Zero(6, model_biped_fixed.nv);
    J_r_body = Eigen::MatrixXd::Zero(6, model_biped_fixed.nv);
    J_hd_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_hd_r = Eigen::MatrixXd::Zero(6, model_nv);
    J_base = Eigen::MatrixXd::Zero(6, model_nv);
    J_hip_link = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_l = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_r = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_hd_l = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_hd_r = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_base = Eigen::MatrixXd::Zero(6, model_nv);
    q.setZero();
    dq.setZero();
    ddq.setZero();
    Rcur.setIdentity();
    dyn_M = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_M_inv = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_C = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_G = Eigen::MatrixXd::Zero(model_nv, 1);

    // get joint index for Pinocchio Lib, need to redefined the joint name for new model
    r_ankle_joint = model_biped.getJointId("J_ankle_r_roll");
    l_ankle_joint = model_biped.getJointId("J_ankle_l_roll");
    r_hand_joint = model_biped.getJointId("J_arm_r_07");
    l_hand_joint = model_biped.getJointId("J_arm_l_07");
    r_hand_joint_fixed = model_biped_fixed.getJointId("J_arm_r_07");
    l_hand_joint_fixed = model_biped_fixed.getJointId("J_arm_l_07");
    r_hip_joint = model_biped.getJointId("J_hip_r_yaw");
    l_hip_joint = model_biped.getJointId("J_hip_l_yaw");
    r_hip_roll_joint = model_biped.getJointId("J_hip_r_roll");
    l_hip_roll_joint = model_biped.getJointId("J_hip_l_roll");
    r_ankle_joint_fixed = model_biped_fixed.getJointId("J_ankle_r_roll");
    l_ankle_joint_fixed = model_biped_fixed.getJointId("J_ankle_l_roll");
    r_hip_joint_fixed = model_biped_fixed.getJointId("J_hip_r_yaw");
    l_hip_joint_fixed = model_biped_fixed.getJointId("J_hip_l_yaw");
    base_joint = model_biped.getJointId("root_joint");
    waist_yaw_joint = model_biped.getJointId("J_waist_yaw");

    // read joint pvt parameters
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in("joint_ctrl_config.json", std::ios::binary);

    motorMaxTorque = Eigen::VectorXd::Zero(motorName.size());
    motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
    motorMinPos = Eigen::VectorXd::Zero(motorName.size());
    reader.parse(in, root_read);
    for (int i = 0; i < motorName.size(); i++)
    {
        motorMaxTorque(i) = (root_read[motorName[i]]["maxTorque"].asDouble());
        motorMaxPos(i) = (root_read[motorName[i]]["maxPos"].asDouble());
        motorMinPos(i) = (root_read[motorName[i]]["minPos"].asDouble());
    }
    motorReachLimit.assign(motorName.size(), false);
    tauJointOld = Eigen::VectorXd::Zero(motorName.size());
}

void Pin_KinDyn::dataBusRead(const DataBus &robotState)
{
    //  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
    //  while its velocity is expressed in the body coordinate system.
    //  https://github.com/stack-of-tasks/pinocchio/issues/1137
    //  q = [global_base_position, global_base_quaternion, joint_positions]
    //  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
    q = robotState.q;
    dq = robotState.dq;
    dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(0, 0, 3, 1);
    dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(3, 0, 3, 1);
    ddq = robotState.ddq;
}

void Pin_KinDyn::dataBusWrite(DataBus &robotState)
{
    robotState.J_l = J_l;
    robotState.J_r = J_r;
    robotState.J_base = J_base;
    robotState.dJ_l = dJ_l;
    robotState.dJ_r = dJ_r;
    robotState.J_hd_l = J_hd_l;
    robotState.J_hd_r = J_hd_r;
    robotState.dJ_hd_l = dJ_hd_l;
    robotState.dJ_hd_r = dJ_hd_r;
    robotState.dJ_base = dJ_base;
    robotState.J_hip_link = J_hip_link;
    robotState.fe_l_pos_W = fe_l_pos;
    robotState.fe_r_pos_W = fe_r_pos;
    robotState.fe_l_pos_L = fe_l_pos_body;
    robotState.fe_r_pos_L = fe_r_pos_body;
    robotState.fe_l_rot_W = fe_l_rot;
    robotState.fe_r_rot_W = fe_r_rot;
    robotState.fe_l_rot_L = fe_l_rot_body;
    robotState.fe_r_rot_L = fe_r_rot_body;
    robotState.fe_l_vel_L = fe_l_vel_body;
    robotState.fe_r_vel_L = fe_r_vel_body;
    robotState.hip_r_pos_L = hip_r_pos_body;
    robotState.hip_l_pos_L = hip_l_pos_body;
    robotState.hip_r_pos_W = hip_r_pos;
    robotState.hip_l_pos_W = hip_l_pos;
    robotState.hd_l_pos_L = hd_l_pos_body;
    robotState.hd_l_rot_L = hd_l_rot_body;
    robotState.hd_l_pos_W = hd_l_pos;
    robotState.hd_l_rot_W = hd_l_rot;
    robotState.hd_r_pos_L = hd_r_pos_body;
    robotState.hd_r_rot_L = hd_r_rot_body;
    robotState.hd_r_pos_W = hd_r_pos;
    robotState.hd_r_rot_W = hd_r_rot;
    robotState.hip_link_pos = hip_link_pos;
    robotState.hip_link_rot = hip_link_rot;

    robotState.dyn_M = dyn_M;
    robotState.dyn_M_inv = dyn_M_inv;
    robotState.dyn_C = dyn_C;
    robotState.dyn_G = dyn_G;
    robotState.dyn_Ag = dyn_Ag;
    robotState.dyn_dAg = dyn_dAg;
    robotState.dyn_Non = dyn_Non;

    robotState.pCoM_W = CoM_pos;
    robotState.Jcom_W = Jcom;

    robotState.inertia = inertia; // w.r.t body frame
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_dJ()
{
    pinocchio::forwardKinematics(model_biped, data_biped, q);
    pinocchio::jacobianCenterOfMass(model_biped, data_biped, q, true);
    //    pinocchio::computeJointJacobians(model_biped,data_biped,q);
    pinocchio::computeJointJacobiansTimeVariation(model_biped, data_biped, q, dq);
    pinocchio::updateGlobalPlacements(model_biped, data_biped);
    pinocchio::getJointJacobian(model_biped, data_biped, r_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_r);
    pinocchio::getJointJacobian(model_biped, data_biped, l_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
    pinocchio::getJointJacobian(model_biped, data_biped, r_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hd_r);
    pinocchio::getJointJacobian(model_biped, data_biped, l_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hd_l);
    pinocchio::getJointJacobian(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_base);

    Eigen::Matrix<double, 6, -1> J_hip_roll_l, J_hip_roll_r;
    J_hip_roll_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_hip_roll_r = Eigen::MatrixXd::Zero(6, model_nv);
    pinocchio::getJointJacobian(model_biped, data_biped, l_hip_roll_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_roll_l);
    pinocchio::getJointJacobian(model_biped, data_biped, r_hip_roll_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_roll_r);
    //    J_hip_link=J_hip_roll_l;
    //    std::cout<<"J_hip_roll_l"<<std::endl<<J_hip_roll_l<<std::endl;
    //    std::cout<<"J_hip_roll_r"<<std::endl<<J_hip_roll_r<<std::endl;
    pinocchio::getJointJacobian(model_biped, data_biped, waist_yaw_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_link);

    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, r_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_r);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, l_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_l);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, r_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_hd_r);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, l_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_hd_l);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_base);
    fe_l_pos = data_biped.oMi[l_ankle_joint].translation();
    fe_l_rot = data_biped.oMi[l_ankle_joint].rotation();
    hip_l_pos = data_biped.oMi[l_hip_joint].translation();
    fe_r_pos = data_biped.oMi[r_ankle_joint].translation();
    fe_r_rot = data_biped.oMi[r_ankle_joint].rotation();
    hip_r_pos = data_biped.oMi[r_hip_joint].translation();
    base_pos = data_biped.oMi[base_joint].translation();
    base_rot = data_biped.oMi[base_joint].rotation();
    hd_l_pos = data_biped.oMi[l_hand_joint].translation();
    hd_l_rot = data_biped.oMi[l_hand_joint].rotation();
    hd_r_pos = data_biped.oMi[r_hand_joint].translation();
    hd_r_rot = data_biped.oMi[r_hand_joint].rotation();
    //    hip_link_pos=(data_biped.oMi[l_hip_roll_joint].translation()+data_biped.oMi[r_hip_roll_joint].translation())*0.5;
    //    hip_link_rot=data_biped.oMi[l_hip_roll_joint].rotation();
    hip_link_pos = data_biped.oMi[waist_yaw_joint].translation();
    hip_link_rot = data_biped.oMi[waist_yaw_joint].rotation();
    Jcom = data_biped.Jcom;

    Eigen::MatrixXd Mpj; // transform into world frame, and accept dq that in world frame
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    J_l = J_l * Mpj;
    J_r = J_r * Mpj;
    J_base = J_base * Mpj;
    dJ_l = dJ_l * Mpj;
    dJ_r = dJ_r * Mpj;
    J_hd_l = J_hd_l * Mpj;
    J_hd_r = J_hd_r * Mpj;
    dJ_hd_l = dJ_hd_l * Mpj;
    dJ_hd_r = dJ_hd_r * Mpj;
    dJ_base = dJ_base * Mpj;
    J_hip_link = J_hip_link * Mpj;
    Jcom = Jcom * Mpj;

    Eigen::VectorXd q_fixed, dq_fixed;
    q_fixed = q.block(7, 0, model_biped_fixed.nv, 1);
    dq_fixed = dq.block(6, 0, model_biped_fixed.nv, 1);
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, q_fixed);
    pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, q_fixed);
    pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);
    pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, r_ankle_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_r_body);
    pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, l_ankle_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_l_body);
    fe_l_pos_body = data_biped_fixed.oMi[l_ankle_joint_fixed].translation();
    fe_r_pos_body = data_biped_fixed.oMi[r_ankle_joint_fixed].translation();
    fe_l_rot_body = data_biped_fixed.oMi[l_ankle_joint_fixed].rotation();
    fe_r_rot_body = data_biped_fixed.oMi[r_ankle_joint_fixed].rotation();
    hip_l_pos_body = data_biped_fixed.oMi[l_hip_joint_fixed].translation();
    hip_r_pos_body = data_biped_fixed.oMi[r_hip_joint_fixed].translation();
    hd_l_pos_body = data_biped_fixed.oMi[l_hand_joint_fixed].translation();
    hd_l_rot_body = data_biped_fixed.oMi[l_hand_joint_fixed].rotation();
    hd_r_pos_body = data_biped_fixed.oMi[r_hand_joint_fixed].translation();
    hd_r_rot_body = data_biped_fixed.oMi[r_hand_joint_fixed].rotation();
    fe_l_vel_body = (J_l_body * dq_fixed).block(0, 0, 3, 1);
    fe_r_vel_body = (J_r_body * dq_fixed).block(0, 0, 3, 1);
}

Eigen::Quaterniond Pin_KinDyn::intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w)
{
    Eigen::Matrix3d Rcur = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta > 1e-8)
    {
        Eigen::Vector3d w_norm;
        w_norm = w / theta;
        Eigen::Matrix3d a;
        a << 0, -w_norm(2), w_norm(1),
            w_norm(0), 0, -w_norm(0),
            -w_norm(1), w_norm(0), 0;
        Rinc = Eigen::Matrix3d::Identity() + a * sin(theta) + a * a * (1 - cos(theta));
    }
    Eigen::Matrix3d Rend = Rcur * Rinc;
    Eigen::Quaterniond quatRes;
    quatRes = Rend;
    return quatRes;
}

// intergrate the q with dq, for floating base dynamics
Eigen::VectorXd Pin_KinDyn::integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI)
{
    Eigen::VectorXd qRes = Eigen::VectorXd::Zero(model_nv + 1);
    Eigen::Vector3d wDes;
    wDes << dqI(3), dqI(4), dqI(5);
    Eigen::Quaterniond quatNew, quatNow;
    quatNow.x() = qI(3);
    quatNow.y() = qI(4);
    quatNow.z() = qI(5);
    quatNow.w() = qI(6);
    quatNew = intQuat(quatNow, wDes);
    qRes = qI;
    qRes(0) += dqI(0);
    qRes(1) += dqI(1);
    qRes(2) += dqI(2);
    qRes(3) = quatNew.x();
    qRes(4) = quatNew.y();
    qRes(5) = quatNew.z();
    qRes(6) = quatNew.w();
    for (int i = 0; i < model_nv - 6; i++)
        qRes(7 + i) += dqI(6 + i);
    return qRes;
}

// update dynamic parameters, M*ddq+C*dq+G=tau
void Pin_KinDyn::computeDyn()
{
    // cal M
    pinocchio::crba(model_biped, data_biped, q);
    // Pinocchio only gives half of the M, needs to restore it here
    data_biped.M.triangularView<Eigen::Lower>() = data_biped.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = data_biped.M;

    // cal Minv
    pinocchio::computeMinverse(model_biped, data_biped, q);
    data_biped.Minv.triangularView<Eigen::Lower>() = data_biped.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv = data_biped.Minv;

    // cal C
    pinocchio::computeCoriolisMatrix(model_biped, data_biped, q, dq);
    dyn_C = data_biped.C;

    // cal G
    pinocchio::computeGeneralizedGravity(model_biped, data_biped, q);
    dyn_G = data_biped.g;

    // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three rows: angular
    pinocchio::dccrba(model_biped, data_biped, q, dq);
    pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, dq);
    dyn_Ag = data_biped.Ag;
    dyn_dAg = data_biped.dAg;

    // cal nonlinear item
    dyn_Non = dyn_C * dq + dyn_G;

    // cal I
    pinocchio::ccrba(model_biped, data_biped, q, dq);
    inertia = data_biped.Ig.inertia().matrix();

    // cal CoM
    CoM_pos = data_biped.com[0];
    //    std::cout<<"CoM_W"<<std::endl;
    //    std::cout<<CoM_pos.transpose()<<std::endl;

    Eigen::MatrixXd Mpj, Mpj_inv; // transform into world frame
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj_inv = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    Mpj_inv.block(0, 0, 3, 3) = base_rot;
    Mpj_inv.block(3, 3, 3, 3) = base_rot;
    dyn_M = Mpj_inv * dyn_M * Mpj;
    dyn_M_inv = Mpj_inv * dyn_M_inv * Mpj;
    dyn_C = Mpj_inv * dyn_C * Mpj;
    dyn_G = Mpj_inv * dyn_G;
    dyn_Non = Mpj_inv * dyn_Non;
}

// Inverse kinematics for leg posture. Note: the Rdes and Pdes are both w.r.t the baselink coordinate in body frame!
Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                           const Eigen::Vector3d &Pdes_R)
{
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv); // initial guess
    qIk[22] = -0.1;
    qIk[28] = -0.1;

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 7e-1;
    const double damp = 5e-3;
    Eigen::MatrixXd JL(6, model_biped_fixed.nv);
    Eigen::MatrixXd JR(6, model_biped_fixed.nv);
    Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_biped_fixed.nv);

    pinocchio::JointIndex J_Idx_l, J_Idx_r;
    J_Idx_l = l_ankle_joint_fixed;
    J_Idx_r = r_ankle_joint_fixed;
    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_biped_fixed.oMi[J_Idx_r].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector(); // in joint frame
        errR = pinocchio::log6(iMdR).toVector(); // in joint frame
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_l, JL); // JL in joint frame
        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_r, JR); // JR in joint frame
        Eigen::MatrixXd W;
        W = Eigen::MatrixXd::Identity(model_biped_fixed.nv, model_biped_fixed.nv); // weighted matrix
        // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
        //        W(16,16)=0.001;  // use a smaller value to make the solver try not to use waist joint
        //        W(17,17)=0.001;
        //        W(18,18)=0.001;
        JL.block(0, 16, 6, 3).setZero();
        JR.block(0, 16, 6, 3).setZero();
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
        JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
        // pinocchio::Data::Matrix6 JJt;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * W * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
    {
        res.status = 0;
    }
    else
    {
        res.status = -1;
    }
    res.jointPosRes = qIk;
    return res;
}

// Inverse Kinematics for hand posture. Note: the Rdes and Pdes are both w.r.t the baselink coordinate in body frame!
Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                            const Eigen::Vector3d &Pdes_R)
{
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv); // initial guess
    // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
    qIk.block<7, 1>(0, 0) << 0.433153883479341, 1.11739345867607, 1.88491913406236,
        0.802378252758275, 1.22726400279662, 0.0249797771339966, -0.0875282610654057;

    qIk.block<7, 1>(7, 0) << -0.433152540054138, -1.11739347975224, -1.88492038240761,
        0.802375980602373, -1.22726323451626, 0.0249795712262396, 0.0875271396314979;

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 6e-1;
    const double damp = 1e-2;
    Eigen::MatrixXd JL(6, model_biped_fixed.nv);
    Eigen::MatrixXd JR(6, model_biped_fixed.nv);
    Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_biped_fixed.nv);

    pinocchio::JointIndex J_Idx_l, J_Idx_r;
    J_Idx_l = l_hand_joint_fixed;
    J_Idx_r = r_hand_joint_fixed;
    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_biped_fixed.oMi[J_Idx_r].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector(); // in joint frame
        errR = pinocchio::log6(iMdR).toVector(); // in joint frame
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_l, JL); // JL in joint frame
        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_r, JR); // JR in joint frame
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
        JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
        // pinocchio::Data::Matrix6 JJt;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
    {
        res.status = 0;
    }
    else
    {
        res.status = -1;
    }

    res.jointPosRes = qIk;
    return res;
}

// must call computeDyn() first!
void Pin_KinDyn::workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT)
{
    for (int i = 0; i < motorName.size(); i++)
        if (qFT(i + 7) > motorMaxPos(i))
        {
            qFT(i + 7) = motorMaxPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else if (qFT(i + 7) < motorMinPos(i))
        {
            qFT(i + 7) = motorMinPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else
            motorReachLimit[i] = false;

    tauJointOld = tauJointFT;
}
