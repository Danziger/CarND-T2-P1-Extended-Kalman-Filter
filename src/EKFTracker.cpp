#include "EKFTracker.h"
#include "Eigen/Dense"

#include <iostream>
#include <iomanip>
#include <limits>


// PRIVATE:


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


// PUBLIC:


EKFTracker::EKFTracker() {
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

    // Adjust these noises (default 9 and 9):

    const float noiseAX = 9;
    const float noiseAY = 9;
    
    ekf_.initNoise(noiseAX, noiseAY); 

    cout
        << setprecision(2) << fixed
        << endl
        << "──────────────────────────────────────────────────────" << endl
        << endl
        << " STD DEV ACC X = " << setfill(' ') << setw(5) << noiseAX << " MET / S ^ 2" << endl
        << " STD DEV ACC Y = " << setfill(' ') << setw(5) << noiseAY << " MET / S ^ 2" << endl
        << endl
        << "──────────────────────────────────────────────────────" << endl;
}


EKFTracker::~EKFTracker() {}


vector<double> EKFTracker::processMeasurement(const MeasurementPackage &pack) {

    // INITIALIZATION:

    if (!is_initialized_) {
        initialize(pack);

        vector<double> empty(5);

        return empty; // Done initializing. No need to predict or update.
    }


    // PREDICTION:

    // Compute the time elapsed between the current and previous measurements:
    const double dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

    previous_timestamp_ = pack.timestamp_;

    // Makes the prediction:
    ekf_.predict(dt);


    // UPDATE:

    // OUTPUT current state and state covariance:
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;

    if (pack.sensor_type_ == MeasurementPackage::RADAR) {
        const double NIS_radar = ekf_.updateRadar(pack.raw_measurements_);

        return updateNIS(NIS_radar, total_radar_, NIS_3_table_, radar_NIS_results_);
    } else {
        const double NIS_lidar = ekf_.updateLidar(pack.raw_measurements_);

        return updateNIS(NIS_lidar, total_lidar_, NIS_2_table_, lidar_NIS_results_);
    }
}


VectorXd EKFTracker::getCurrentState() {
    return ekf_.getCurrentState();
}


vector<double> EKFTracker::updateNIS(
    double current_NIS,
    int &total_measurements,
    vector<double> &NIS_table,
    vector<int> &NIS_results
) {
    vector<double> NIS_stats;

    NIS_stats.push_back(current_NIS);

    ++total_measurements;

    int i = -1;

    for (vector<double>::iterator it = NIS_table.begin(); it != NIS_table.end(); ++it) {
        if (current_NIS > *it) {                
            NIS_stats.push_back(100 * ++NIS_results[++i] / total_measurements);
        } else {
            NIS_stats.push_back(100 * NIS_results[++i] / total_measurements);
        }
    }

    return NIS_stats;
}
