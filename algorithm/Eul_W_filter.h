//
// Created by boxing on 23-7-10.
//

#ifndef BIPED_STATEEST_EUL_W_FILTER_H
#define BIPED_STATEEST_EUL_W_FILTER_H

#include <Eigen/Dense>

class Eul_W_filter {
private:
    double dt;

public:
    bool isIni=false;

    Eigen::Matrix<double,6,1> kal_X, kal_Y, kal_Z; // eul, wL
    Eigen::Matrix<double,6,6> F; // state transition matrix
    Eigen::Matrix<double,6,6> H; // output matrix
    Eigen::Matrix<double,6,6> P; // covirance matrix
    Eigen::Matrix<double,6,6> Q; // process noise matrix
    Eigen::Matrix<double,6,6> R; // output noise matrix

    double Eul_filtered[3], wL_filtered[3];

    Eul_W_filter(double dtIn);
    void run(double *eulIn, double *wLIn);
    void getData(double *eulOut,double *wLOut);
};


#endif //BIPED_STATEEST_EUL_W_FILTER_H
