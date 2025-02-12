/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "Eigen/Dense"
#include <vector>

Eigen::MatrixXd pseudoInv_SVD(const Eigen::MatrixXd &mat);

Eigen::MatrixXd pseudoInv_right(const Eigen::MatrixXd &M);

Eigen::MatrixXd pseudoInv_right_weighted(const Eigen::MatrixXd &M,
                                         const Eigen::DiagonalMatrix<double, -1> &W); // weighted right pseudo inverse

Eigen::MatrixXd dyn_pseudoInv(const Eigen::MatrixXd &M, const Eigen::MatrixXd &dyn_M, bool isMinv);

Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw);

Eigen::Vector3d Rot2eul(const Eigen::Matrix3d &rot);

Eigen::Quaterniond eul2quat(double roll, double pitch, double yaw);

std::vector<double> eigen2std(const Eigen::VectorXd &Vec);

Eigen::Matrix<double, 3, 1> diffRot(const Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes);

Eigen::Matrix<double, 4, 1> quat2axisAngle(const Eigen::Quaternion<double> &quat);

Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);

Eigen::Matrix<double, 3, 3> Rx3(double theta);

Eigen::Matrix<double, 3, 3> Ry3(double theta);

Eigen::Matrix<double, 3, 3> Rz3(double theta);

Eigen::Matrix<double, 3, 3> CrossProduct_A(Eigen::Matrix<double, 3, 1> A);

double Ramp(double u, double tgt, double inc);

void Limit(double &data, double max, double min);

double sign(const double &x);

double d2r(double deg);
double r2d(double rad);