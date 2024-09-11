#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "wbc_priority.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include "StateEst.h"

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/scene_board.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

//************************
int main(int argc, const char **argv)
{
    // ini classes
    UIctr uiController(mj_model, mj_data);                                             // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data);                                      // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../models/AzureLoong.urdf");                              // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv);                                         // data bus
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mj_model->opt.timestep); // WBC solver
    GaitScheduler gaitScheduler(0.4, mj_model->opt.timestep);                          // gait scheduler
    PVT_Ctr pvtCtr(mj_model->opt.timestep, "../common/joint_ctrl_config.json");        // PVT joint control
    FootPlacement footPlacement;                                                       // foot-placement planner
    JoyStickInterpreter jsInterp(mj_model->opt.timestep);                              // desired baselink velocity generator
    DataLogger logger("../record/datalog.log");                                        // data logger
    StateEst StateModule(0.001);

    // variables ini
    double stand_legLength = 1.01; // desired baselink height
    double foot_height = 0.07;     // distance between the foot ankel joint and the bottom
    double xv_des = 0.4;           // desired velocity in x direction

    RobotState.width_hips = 0.229;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.03;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.12;
    footPlacement.legLength = stand_legLength;
    // mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv = kinDynSolver.model_nv;

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);
    Eigen::Vector3d fe_l_pos_L_des = {-0.018, 0.113, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {-0.018, -0.116, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des = {0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    Eigen::VectorXd hd_l_des, hd_r_des;
    hd_l_des.resize(7);
    hd_r_des.resize(7);

    hd_l_des << 0.475, -1.12, 1.9, 0.86, -0.356, 0, 0;
    hd_r_des << -0.475, -1.12, -1.9, 0.86, 0.356, 0, 0;

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes;
    qIniDes.block(7, 0, 7, 1) = hd_l_des;
    qIniDes.block(14, 0, 7, 1) = hd_r_des;
    WBC_solv.setQini(qIniDes, RobotState.q);

    // register variable name for data logger
    logger.addIterm("dyn_time", 1);
    logger.addIterm("motors_pos_cur", model_nv - 6);
    logger.addIterm("motors_pos_des", model_nv - 6);
    logger.addIterm("motors_tor_cur", model_nv - 6);
    logger.addIterm("motors_tor_des", model_nv - 6);
    logger.addIterm("motors_tor_out", model_nv - 6);
    logger.addIterm("motors_vel_des", model_nv - 6);
    logger.addIterm("motors_vel_cur", model_nv - 6);
    logger.addIterm("FL_est", 3);
    logger.addIterm("FR_est", 3);
    logger.addIterm("wbc_FrRes", 12);
    logger.addIterm("base_pos_des", 3);
    logger.addIterm("base_pos", 3);
    logger.addIterm("base_pos_est", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("base_vel_est", 3);
    logger.addIterm("base_rpy", 3);
    logger.addIterm("eul_est", 3);
    logger.finishItermAdding();

    /// ----------------- sim Loop ---------------
    double simEndTime = 1000;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double openLoopCtrTime = 3;
    double startSteppingTime = 7;
    double startWalkingTime = 10;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo", false);
    UIctr::ButtonState buttonState;

    while (!glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim)
        {
            mj_step(mj_model, mj_data);

            simTime = mj_data->time;
            // printf("-------------%.3f s------------\n", simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            if (simTime > 1 && StateModule.flag_init)
            {
                std::cout << "init state module" << std::endl;
                StateModule.init(RobotState);
            }

            // input from joystick
            // space: start and stop stepping (after 3s)
            // w: forward walking
            // s: stop forward walking
            // a: turning left
            // d: turning right
            buttonState = uiController.getButtonState();
            if (simTime > openLoopCtrTime)
            {
                if (buttonState.key_space && RobotState.motionState == DataBus::Stand)
                {
					gaitScheduler.start();
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    RobotState.motionState = DataBus::Walk;
                }
                else if (buttonState.key_space && RobotState.motionState == DataBus::Walk && fabs(jsInterp.vxLGen.y) < 0.01)
                {
                    RobotState.motionState = DataBus::Walk2Stand;
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                }

                if (buttonState.key_a && RobotState.motionState != DataBus::Stand)
                {
                    if (jsInterp.wzLGen.yDes < 0)
                        jsInterp.setWzDesLPara(0, 0.5);
                    else
                        jsInterp.setWzDesLPara(0.35, 1.0);
                }
                if (buttonState.key_d && RobotState.motionState != DataBus::Stand)
                {
                    if (jsInterp.wzLGen.yDes > 0)
                        jsInterp.setWzDesLPara(0, 0.5);
                    else
                        jsInterp.setWzDesLPara(-0.35, 1.0);
                }

                if (buttonState.key_w && RobotState.motionState != DataBus::Stand)
                    jsInterp.setVxDesLPara(xv_des, 2.0);

                if (buttonState.key_s && RobotState.motionState != DataBus::Stand)
                    jsInterp.setVxDesLPara(0, 0.5);

                if (buttonState.key_h)
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            }

            StateModule.set(RobotState);
            StateModule.update();
            StateModule.get(RobotState);

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            // update F EST
            StateModule.setF(RobotState);
            StateModule.updateF();
            StateModule.getF(RobotState);

            if (simTime >= openLoopCtrTime && simTime < openLoopCtrTime + 0.002)
            {
                RobotState.motionState = DataBus::Stand;
            }

            if (RobotState.motionState == DataBus::Walk2Stand || simTime <= openLoopCtrTime)
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

            // switch between walk and stand
            if (RobotState.motionState == DataBus::Walk || RobotState.motionState == DataBus::Walk2Stand)
            {
                jsInterp.step();
                RobotState.js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
                jsInterp.dataBusWrite(RobotState);                        // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

                //                if (simTime <startSteppingTime+0.002)
                //                    RobotState.motionState=DataBus::Walk;
                // gait scheduler
                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);

                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);
            }

            if (simTime <= openLoopCtrTime || RobotState.motionState == DataBus::Walk2Stand)
            {
                WBC_solv.setQini(qIniDes, RobotState.q);
                WBC_solv.fe_l_pos_des_W = RobotState.fe_l_pos_W;
                WBC_solv.fe_r_pos_des_W = RobotState.fe_r_pos_W;
                WBC_solv.fe_l_rot_des_W = RobotState.fe_l_rot_W;
                WBC_solv.fe_r_rot_des_W = RobotState.fe_r_rot_W;
                WBC_solv.pCoMDes = RobotState.pCoM_W;
                WBC_solv.pCoMDes(0) = (RobotState.fe_l_pos_W(0) + RobotState.fe_r_pos_W(0)) * 0.5;
                WBC_solv.pCoMDes(1) = (RobotState.fe_l_pos_W(1) + RobotState.fe_r_pos_W(1)) * 0.5;
            }

            if (RobotState.motionState == DataBus::Stand)
            {
                WBC_solv.pCoMDes(0) = (RobotState.fe_l_pos_W(0) + RobotState.fe_r_pos_W(0)) * 0.5;
                WBC_solv.pCoMDes(1) = (RobotState.fe_l_pos_W(1) + RobotState.fe_r_pos_W(1)) * 0.5;
            }

            //            std::cout<<"pCoM_W"<<std::endl<<RobotState.pCoM_W.transpose()<<std::endl<<"pCoM_Des"<<std::endl<<WBC_solv.pCoMDes.transpose()<<std::endl;

            // ------------- WBC ------------
            // WBC input
            RobotState.Fr_ff = Eigen::VectorXd::Zero(12);
            RobotState.des_ddq = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.des_dq = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.des_delta_q = Eigen::VectorXd::Zero(mj_model->nv);
            RobotState.base_rpy_des << 0, 0, jsInterp.thetaZ;
            RobotState.base_pos_des = RobotState.js_pos_des;
            RobotState.base_pos_des(2) = stand_legLength + foot_height;

            RobotState.Fr_ff << 0, 0, 370, 0, 0, 0,
                0, 0, 370, 0, 0, 0;

            // adjust des_delata_q, des_dq and des_ddq to achieve forward walking
            if (RobotState.motionState == DataBus::Walk)
            {
                RobotState.des_delta_q.block<2, 1>(0, 0) << jsInterp.vx_W * mj_model->opt.timestep, jsInterp.vy_W * mj_model->opt.timestep;
                RobotState.des_delta_q(5) = jsInterp.wz_L * mj_model->opt.timestep;
                RobotState.des_dq.block<2, 1>(0, 0) << jsInterp.vx_W, jsInterp.vy_W;
                RobotState.des_dq(5) = jsInterp.wz_L;

                double k = 5; // 5
                RobotState.des_ddq.block<2, 1>(0, 0) << k * (jsInterp.vx_W - RobotState.dq(0)), k * (jsInterp.vy_W -
                                                                                                     RobotState.dq(1));
                RobotState.des_ddq(5) = k * (jsInterp.wz_L - RobotState.dq(5));
            }
            // printf("js_vx=%.3f js_vy=%.3f wz_L=%.3f px_w=%.3f py_w=%.3f thetaZ=%.3f\n", jsInterp.vx_W, jsInterp.vy_W, jsInterp.wz_L, jsInterp.px_W, jsInterp.py_W, jsInterp.thetaZ);

            // WBC Calculation
            WBC_solv.dataBusRead(RobotState);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(RobotState);

            // get the final joint command
            if (simTime <= openLoopCtrTime)
            {
                Eigen::VectorXd temp = resLeg.jointPosRes;
                temp.block(0, 0, 7, 1) = hd_l_des;
                temp.block(7, 0, 7, 1) = hd_r_des;
                RobotState.motors_pos_des = eigen2std(temp);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
            }
            else
            {
                Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
                RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            }

            pvtCtr.dataBusRead(RobotState);
            if (simTime <= openLoopCtrTime)
            {
                pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
            }
            else
            {
                double kp = 1.;
                double kd = 1.;

                pvtCtr.setJointPD(400 * kp, 15 * kd, "J_hip_l_roll");
                pvtCtr.setJointPD(200 * kp, 10 * kd, "J_hip_l_yaw");
                pvtCtr.setJointPD(300 * kp, 10 * kd, "J_hip_l_pitch");
                pvtCtr.setJointPD(300 * kp, 14 * kd, "J_knee_l_pitch");
                pvtCtr.setJointPD(300 * kp, 18 * kd, "J_ankle_l_pitch");
                pvtCtr.setJointPD(300 * kp, 16 * kd, "J_ankle_l_roll");

                pvtCtr.setJointPD(400 * kp, 15 * kd, "J_hip_r_roll");
                pvtCtr.setJointPD(200 * kp, 10 * kd, "J_hip_r_yaw");
                pvtCtr.setJointPD(300 * kp, 10 * kd, "J_hip_r_pitch");
                pvtCtr.setJointPD(300 * kp, 14 * kd, "J_knee_r_pitch");
                pvtCtr.setJointPD(300 * kp, 18 * kd, "J_ankle_r_pitch");
                pvtCtr.setJointPD(300 * kp, 16 * kd, "J_ankle_r_roll");

                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);
            mj_interface.setMotorsTorque(RobotState.motors_tor_out);
            // data record

            logger.startNewLine();
            logger.recItermData("dyn_time", simTime);
            logger.recItermData("motors_pos_cur", RobotState.motors_pos_cur);
            logger.recItermData("motors_pos_des", RobotState.motors_pos_des);
            logger.recItermData("motors_tor_cur", RobotState.motors_tor_cur);
            logger.recItermData("motors_tor_des", RobotState.motors_tor_des);
            logger.recItermData("motors_tor_out", RobotState.motors_tor_out);
            logger.recItermData("motors_vel_cur", RobotState.motors_vel_cur);
            logger.recItermData("motors_vel_des", RobotState.motors_vel_des);
            logger.recItermData("FL_est", RobotState.FL_est);
            logger.recItermData("FR_est", RobotState.FR_est);
            logger.recItermData("wbc_FrRes", RobotState.wbc_FrRes);
            logger.recItermData("base_pos_des", RobotState.base_pos_des);
            logger.recItermData("base_pos", RobotState.base_pos);
            logger.recItermData("base_pos_est", RobotState.base_pos_est);
            logger.recItermData("baseLinVel", RobotState.baseLinVel);
            logger.recItermData("base_vel_est", RobotState.base_vel_est);
            logger.recItermData("base_rpy", RobotState.base_rpy);
            logger.recItermData("eul_est", RobotState.eul_est);
            logger.finishLine();

            // printf("rpyVal=[%.5f, %.5f, %.5f]\n", RobotState.rpy[0], RobotState.rpy[1], RobotState.rpy[2]);
            // printf("gps=[%.5f, %.5f, %.5f]\n", RobotState.basePos[0], RobotState.basePos[1], RobotState.basePos[2]);
            // printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.baseLinVel[0], RobotState.baseLinVel[1], RobotState.baseLinVel[2]);
            // printf("vel=[%.5f, %.5f, %.5f]\n", RobotState.base_vel(0), RobotState.base_vel(1), RobotState.base_vel(2));
        }

        if (mj_data->time >= simEndTime)
        {
            break;
        }
        uiController.updateScene();
    }

    //    // free visualization storage
    uiController.Close();

    return 0;
}