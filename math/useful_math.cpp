/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "useful_math.h"
#include "Eigen/Dense"

using namespace Eigen;

// SVD based pseudo-inverse
Eigen::MatrixXd pseudoInv_SVD(const Eigen::MatrixXd &mat)
{
    typedef typename Eigen::MatrixXd::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv;
    double maxSgVal{0};
    for (unsigned int i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues(i) > maxSgVal)
            maxSgVal = singularValues(i);
    }
    double tolerance = std::numeric_limits<double>::epsilon() * std::max(mat.rows(), mat.cols()) * maxSgVal;
    singularValuesInv = Eigen::MatrixXd::Zero(mat.cols(), mat.rows());
    for (unsigned int i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
};

Eigen::MatrixXd pseudoInv_right(const Eigen::MatrixXd &M)
{
    Eigen::MatrixXd Mres;
    Mres = M * M.transpose();
    Mres = M.transpose() * Mres.ldlt().solve(Eigen::MatrixXd::Identity(Mres.rows(), Mres.cols()));
    //    Mres=M.transpose()* Mres.inverse();
    return Mres;
}

Eigen::MatrixXd pseudoInv_right_weighted(const Eigen::MatrixXd &M, const Eigen::DiagonalMatrix<double, -1> &W)
{
    double damp = 0;
    Eigen::MatrixXd Mres;
    Mres = M * W.inverse() * M.transpose();
    //    Mres=W.inverse()*M.transpose()* pseudoInv_SVD(Mres);
    Mres.diagonal().array() += damp;
    Mres = W.inverse() * M.transpose() * Mres.completeOrthogonalDecomposition().pseudoInverse();
    return Mres;
}

// Eigen::MatrixXd dyn_pseudoInv(const MatrixXd &M, const MatrixXd &dyn_A) {
////--------------------------
//    Eigen::MatrixXd res;
//    Eigen::MatrixXd Minv = dyn_A.ldlt().solve(Eigen::MatrixXd::Identity(dyn_A.rows(), dyn_A.cols()));
////    res = Minv * M.transpose() * pseudoInv_SVD(M * Minv * M.transpose());
//    res = Minv * M.transpose() * (M * Minv * M.transpose()).inverse();
//    return res;
//}

Eigen::MatrixXd dyn_pseudoInv(const Eigen::MatrixXd &M, const Eigen::MatrixXd &dyn_M, bool isMinv)
{
    double damp = 0;
    Eigen::MatrixXd Minv;

    if (isMinv)
        Minv = dyn_M;
    else
        Minv = dyn_M.llt().solve(Eigen::MatrixXd::Identity(dyn_M.rows(), dyn_M.cols()));

    Eigen::MatrixXd temp = M * Minv * M.transpose();

    temp.diagonal().array() += damp;

    Eigen::MatrixXd res = Minv * M.transpose() * temp.completeOrthogonalDecomposition().pseudoInverse();

    //    Eigen::MatrixXd res = Minv * M.transpose() * temp.inverse();
    return res;
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

Eigen::Vector3d Rot2eul(const Eigen::Matrix3d &rot)
{
    Eigen::Vector3d eul;
    eul(0) = std::atan2(rot(2, 1), rot(2, 2));
    eul(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
    eul(2) = std::atan2(rot(1, 0), rot(0, 0));
    return eul;
}

std::vector<double> eigen2std(const Eigen::VectorXd &eigenVector)
{
    std::vector<double> stdVector(eigenVector.data(), eigenVector.data() + eigenVector.size());
    return stdVector;
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

Eigen::Matrix<double, 3, 1> diffRot(const Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes)
{
    Eigen::Matrix3d R = Rcur.transpose() * Rdes;
    Eigen::Vector3d w;

    if (R.isDiagonal(1e-5) && fabs(R(0, 0)) + fabs(R(1, 1)) + fabs(R(2, 2)) - 3 < 1e-3)
    {
        w.setZero();
    }
    else if (R.isDiagonal(1e-5))
    {
        w << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
        w = w * 3.1415 / 2.0;
    }
    else
    {
        Eigen::Vector3d l;
        l << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        double sita = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
        w = sita * l / l.norm();
    }
    w = Rcur * w;
    return w;
}

Eigen::Matrix<double, 4, 1> quat2axisAngle(const Eigen::Quaternion<double> &quat)
{
    // quat must be normolized
    Eigen::Matrix<double, 4, 1> res;
    double angle = 2.0 * acos(quat.w());
    double s = sqrt(1 - quat.w() * quat.w());
    if (s < 1e-8)
    {
        res(0) = quat.x();
        res(1) = quat.y();
        //        res(2)=quat.z();
        res(2) = 1; // no rotation now but the axis must not be [0,0,0]
    }
    else
    {
        res(0) = quat.x() / s;
        res(1) = quat.y() / s;
        res(2) = quat.z() / s;
    }
    res(3) = angle;
    return res;
}

Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w)
{
    Eigen::Matrix3d Rcur = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta > 1e-4)
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

double sign(const double &x)
{
    if (x > 0)
        return 1.0;
    else if (x < 0)
        return -1.0;
    else
        return 0;
}

Eigen::Matrix<double, 3, 3> Rx3(double theta)
{ // local to world
    // for 2D-XY vector, rotation matrix along z axis
    Eigen::Matrix<double, 3, 3> M;
    M << 1, 0, 0,
        0, cos(theta), -sin(theta),
        0, sin(theta), cos(theta);
    return M;
}

Eigen::Matrix<double, 3, 3> Ry3(double theta)
{ // local to world
    // for 2D-XY vector, rotation matrix along z axis
    Eigen::Matrix<double, 3, 3> M;
    M << cos(theta), 0, sin(theta),
        0, 1, 0,
        -sin(theta), 0, cos(theta);
    return M;
}

Eigen::Matrix<double, 3, 3> Rz3(double theta)
{ // local to world
    // for 2D-XY vector, rotation matrix along z axis
    Eigen::Matrix<double, 3, 3> M;
    M << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    return M;
}

Eigen::Matrix<double, 3, 3> CrossProduct_A(Eigen::Matrix<double, 3, 1> A)
{
    Eigen::Matrix<double, 3, 3> M;
    M << 0.0, -A[2], A[1],
        A[2], 0.0, -A[0],
        -A[1], A[0], 0.0;
    return M;
}

double Ramp(double u, double tgt, double inc)
{
    double output;
    if (abs(u - tgt) < inc)
        output = tgt;
    else if (u < tgt - inc)
        output = u + inc;
    else if (u > tgt + inc)
        output = u - inc;
    else
        output = tgt;

    return output;
}

void Limit(double &data, double max, double min)
{
    if (data > max)
        data = max;
    if (data < min)
        data = min;
}

double d2r(double deg)
{
    return deg / 180.0 * 3.1415926;
}
double r2d(double rad)
{
    return rad * 180.0 / 3.1415926;
}