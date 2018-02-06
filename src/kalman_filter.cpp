#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
    MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */

    MatrixXd Hj = tools.CalculateJacobian(x_);

    // TODO: Handle jacobian 0 division

    VectorXd z_pred = h(x_);
    VectorXd y = z - z_pred;

    // Normalize angle in range [-PI, PI]
    float phi = y(1);
    int times = phi / M_PI;

    y(1) = phi - M_PI * (times >= 0 ? ceil(times) : floor(times));

    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj * P_ * Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHjt = P_ * Hjt;
    MatrixXd K = PHjt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;
}


VectorXd KalmanFilter::h(const VectorXd &x) {

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
