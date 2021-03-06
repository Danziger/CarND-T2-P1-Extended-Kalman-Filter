#include "EKF.h"
#include "common/tools.h"
#include "common/helpers.h"
#include "common/Eigen-3.3/Dense"

#include <iostream>


#define EPS 0.0001
#define NEPS -EPS
#define KEEP_IN_RANGE(n) (n < NEPS ? NEPS : (n < EPS ? EPS : n))
#define ATAN00 atan2(EPS, EPS)


// PRIVATE:


VectorXd EKF::h(const VectorXd &x) {
    const float px = x(0);
    const float py = x(1);
    const float vx = x(2);
    const float vy = x(3);

    const float rho = sqrt(px * px + py * py);
    const float phi = px == 0 && py == 0 ? ATAN00 : atan2(py, px);

    VectorXd polar = VectorXd(3);

    // rho (range), phi (bearing), rho_dot (velocity)
    polar << rho, phi, (px * vx + py * vy) / KEEP_IN_RANGE(rho);

    return polar;
}


MatrixXd EKF::calculateJacobian() {
    MatrixXd Hj(3, 4);

    // Recover state parameters:

    const float px = x_(0);
    const float py = x_(1);
    const float vx = x_(2);
    const float vy = x_(3);

    // Pre-compute a set of terms to avoid repeated calculation:

    const float px2 = px * px;
    const float py2 = py * py;
    float px2py2 = px2 + py2;
    const float d1 = sqrt(px2py2);
    float d2 = px2py2 * d1;
    const float pxd1 = px / d1;
    const float pyd1 = py / d1;

    // Check division by zero:

    px2py2 = KEEP_IN_RANGE(px2py2);
    d2 = KEEP_IN_RANGE(d2);

    // Compute Hj:

    Hj <<
        pxd1, pyd1, 0, 0,
        -py / px2py2, px / px2py2, 0, 0,
        py * (vx*py - vy * px) / d2, px * (vy*px - vx * py) / d2, pxd1, pyd1;

    return Hj;
}


double EKF::estimate(const VectorXd &diffZ, const MatrixXd &H, const MatrixXd &R) {
    const MatrixXd Ht = H.transpose();
    const MatrixXd PHt = P_ * Ht;
    const MatrixXd S = H * PHt + R;
    const MatrixXd Sinv = S.inverse();
    const MatrixXd K = PHt * Sinv;

    // New estimate. Update state mean and covariance matrix:
    x_ += K * diffZ;
    P_ -= K * H * P_;

    // Return NIS:
    return diffZ.transpose() * Sinv * diffZ;
}


// PUBLIC:


EKF::EKF() {}


EKF::~EKF() {}


void EKF::setMatrixes(
    const MatrixXd &F,
    const MatrixXd &H,
    const MatrixXd &R_laser,
    const MatrixXd &R_radar
) {
    F_ = F;
    H_ = H;
    R_laser_ = R_laser;
    R_radar_ = R_radar;

    Q_ = MatrixXd(4, 4); // Will be updated from EKF::predict
}


void EKF::setP(const MatrixXd &P) {
    P_ = P;
}


void EKF::setState(const float px, const float py, const float vx, const float vy) {
    x_ = VectorXd(4);

    x_ << px, py, vx, vy;
}


void EKF::setNoise(const float nx, const float ny) {
    noiseAX_ = nx;
    noiseAY_ = ny;
}


VectorXd EKF::getCurrentState() {
    return x_;
}


void EKF::predict(const double dt) {
    // Integrates the time in F and Q:

    F_(0, 2) = dt;
    F_(1, 3) = dt;

    const float dt_2 = dt * dt;
    const float dt_3 = dt_2 * dt;
    const float dt_4 = dt_3 * dt;

    Q_ <<
        dt_4 / 4 * noiseAX_, 0, dt_3 / 2 * noiseAX_, 0,
        0, dt_4 / 4 * noiseAY_, 0, dt_3 / 2 * noiseAY_,
        dt_3 / 2 * noiseAX_, 0, dt_2 * noiseAX_, 0,
        0, dt_3 / 2 * noiseAY_, 0, dt_2 * noiseAY_;
    
    // Makes the prediction:

    const MatrixXd Ft = F_.transpose();

    x_ = F_ * x_;    
    P_ = F_ * P_ * Ft + Q_;
}


double EKF::update(const VectorXd &z, const MatrixXd &R) {
    const VectorXd z_pred = H_ * x_;
    const VectorXd diffZ = z - z_pred;

    return estimate(diffZ, H_, R);
}

double EKF::updateEKF(const VectorXd &z, const MatrixXd &R) {
    const VectorXd z_pred = h(x_);
    VectorXd diffZ = z - z_pred;

    diffZ(1) = helpers::normalizeAngle(diffZ(1)); // Normalize angle in range [-PI, PI]

    return estimate(diffZ, calculateJacobian(), R);
}


double EKF::updateLidar(const VectorXd &z) {
    return update(z, R_laser_);
}


double EKF::updateRadar(const VectorXd &z) {
    return updateEKF(z, R_radar_);
}
