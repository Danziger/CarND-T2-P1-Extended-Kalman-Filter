#include "EKF.h"

#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;


VectorXd EKF::h(const VectorXd &x) {

    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    float rho = sqrt(px * px + py * py);

    VectorXd polar = VectorXd(3);

    // rho (range), phi (bearing), rho_dot (velocity)
    polar << rho, atan2(py, px), (px * vx + py * vy) / rho;

    return polar;
}


MatrixXd EKF::calculateJacobian() {
    MatrixXd Hj(3, 4);

    // Recover state parameters:

    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // Pre-compute a set of terms to avoid repeated calculation:

    float px2 = px * px;
    float py2 = py * py;
    float px2py2 = px2 + py2;
    float d1 = sqrt(px2py2);
    float d2 = px2py2 * d1;
    float pxd1 = px / d1;
    float pyd1 = py / d1;

    // Check division by zero:

    if (fabs(px2py2) < 0.0001) {
        if (px2py2 < 0) {
            px2py2 = -0.0001;
        } else {
            px2py2 = 0.0001;
        }
    }

    Hj <<
        pxd1, pyd1, 0, 0,
        -py / px2py2, px / px2py2, 0, 0,
        py * (vx*py - vy * px) / d2, px * (vy*px - vx * py) / d2, pxd1, pyd1;

    return Hj;
}


void EKF::estimate(const VectorXd &y, const MatrixXd &H, const MatrixXd &R) {
    MatrixXd Ht = H.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H * PHt + R;
    MatrixXd K = PHt * S.inverse();

    // New estimate:

    x_ = x_ + (K * y);
    P_ = (I_ - K * H) * P_;
}


EKF::EKF() {}


EKF::~EKF() {}


void EKF::initMatrixes(
    MatrixXd &P_in,
    MatrixXd &F_in,
    MatrixXd &H_in,
    MatrixXd &R_laser_in_,
    MatrixXd &R_radar_in_
) {
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_laser_ = R_laser_in_;
    R_radar_ = R_radar_in_;

    Q_ = MatrixXd(4, 4); // Will be updated from EKF::predict

    I_ = MatrixXd::Identity(4, 4);
}


void EKF::initState(float px, float py, float vx, float vy) {
    x_ = VectorXd(4);

    x_ << px, py, vx, vy;
}


void EKF::initNoise(float nx, float ny) {
    noiseAX_ = nx;
    noiseAY_ = ny;
}


VectorXd EKF::getCurrentState() {
    return x_;
}


void EKF::predict(float dt) {
    // Integrates the time in F and Q:

    F_(0, 2) = dt;
    F_(1, 3) = dt;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    Q_ <<
        dt_4 / 4 * noiseAX_, 0, dt_3 / 2 * noiseAX_, 0,
        0, dt_4 / 4 * noiseAY_, 0, dt_3 / 2 * noiseAY_,
        dt_3 / 2 * noiseAX_, 0, dt_2 * noiseAX_, 0,
        0, dt_3 / 2 * noiseAY_, 0, dt_2 * noiseAY_;
    
    // Makes the prediction:

    MatrixXd Ft = F_.transpose();

    x_ = F_ * x_;    
    P_ = F_ * P_ * Ft + Q_;
}


void EKF::update(const VectorXd &z, const MatrixXd &R) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;

    // Estimates:
    estimate(y, H_, R);
}

void EKF::updateEKF(const VectorXd &z, const MatrixXd &R) {
    VectorXd z_pred = h(x_);
    VectorXd y = z - z_pred;

    // Normalize angle in range [-PI, PI]:

    float phi = y(1);
    int times = phi / M_PI;

    y(1) = phi - M_PI * (times >= 0 ? ceil(times) : floor(times));

    // Estimates:
    estimate(y, calculateJacobian(), R);
}


void EKF::updateLaser(const VectorXd &z) {
    update(z, R_laser_);
}


void EKF::updateRadar(const VectorXd &z) {
    updateEKF(z, R_radar_);
}
