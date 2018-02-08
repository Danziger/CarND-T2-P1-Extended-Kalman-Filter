#include "EKFTracker.h"
#include "Eigen/Dense"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


EKFTracker::EKFTracker() {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // State covariance matrix:
    MatrixXd P(4, 4);

    P <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // State transition matrix:
    MatrixXd F(4, 4);

    F <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Measurement matrix (laser only):
    MatrixXd H(2, 4);

    H <<
        1, 0, 0, 0,
        0, 1, 0, 0;

    // Measurement covariance matrixes:
    MatrixXd R_laser(2, 2);
    MatrixXd R_radar(3, 3);

    R_laser <<
        0.0225, 0,
        0, 0.0225;

    R_radar <<
        0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    // INITIALIZE EKF:

    ekf_.initMatrixes(P, F, H, R_laser, R_radar);
    ekf_.initNoise(9, 9); // Default = 9, 9
}


EKFTracker::~EKFTracker() {}


void EKFTracker::processMeasurement(const MeasurementPackage &pack) {

    // INITIALIZATION:

    if (!is_initialized_) {
        initialize(pack);

        return; // Done initializing. No need to predict or update.
    }


    // PREDICTION:

    // Compute the time elapsed between the current and previous measurements:
    const float dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

    previous_timestamp_ = pack.timestamp_;

    // Makes the prediction:
    ekf_.predict(dt);


    // UPDATE:

    if (pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.updateRadar(pack.raw_measurements_);
    } else {
        ekf_.updateLaser(pack.raw_measurements_);
    }

    // OUTPUT current state and state covariance:
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;
}


VectorXd EKFTracker::getCurrentState() {
    return ekf_.getCurrentState();
}


void EKFTracker::initialize(const MeasurementPackage &pack) {
    const VectorXd measurements = pack.raw_measurements_;
    const MeasurementPackage::SensorType type = pack.sensor_type_;

    if (type == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state:

        const float rho = measurements[0]; // Range
        const float phi = measurements[1]; // Bearing

        // rho (range), phi (bearing), rho_dot (velocity)
        ekf_.initState(rho * cos(phi), rho * sin(phi), 0, 0);
    } else if (type == MeasurementPackage::LASER) {
        // Set the state with the initial location and zero velocity:

        ekf_.initState(measurements[0], measurements[1], 0, 0);
    }

    // OUTPUT initial value:
    // std::cout << "INITIAL x = " << ekf_.getCurrentState().transpose() << std::endl;

    previous_timestamp_ = pack.timestamp_;
    is_initialized_ = true;
}