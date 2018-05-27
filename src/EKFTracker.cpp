#include "EKFTracker.h"
#include "common/MeasurementPackage.h"
#include "common/tools.h"
#include "common/format.h"
#include "common/Eigen-3.3/Dense"

#include <iostream>
#include <iomanip>
#include <limits>


#define MAX_DT 2

#define RMSE_X_LIMIT 0.11
#define RMSE_Y_LIMIT 0.11
#define RMSE_VX_LIMIT 0.52
#define RMSE_VY_LIMIT 0.52


// PRIVATE:


void EKFTracker::initialize(const MeasurementPackage &pack) {

    // Execution time stats:
    time_ = 0;

    // Initialize RMSE:

    RMSE_sum_ = VectorXd(4);
    RMSE_ = VectorXd(4);

    RMSE_sum_ << 0, 0, 0, 0;
    RMSE_ << 0, 0, 0, 0;

    // Measurements count:

    total_ = 0;
    total_lidar_ = 0;
    total_radar_ = 0;

    // NIS stats:

    lidar_NIS_results_ = { 0, 0, 0, 0 };
    radar_NIS_results_ = { 0, 0, 0, 0 };


    // Initialize state covariance matrix P based on lidar noises:

    MatrixXd P(4, 4);

    P <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    ekf_.setP(P);


    // Initialize state:

    const VectorXd measurements = pack.raw_measurements_;
    const MeasurementPackage::SensorType type = pack.sensor_type_;

    if (type == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state:

        const float rho = measurements[0]; // Range
        const float phi = measurements[1]; // Bearing

        // rho (range), phi (bearing), rho_dot (velocity)
        ekf_.setState(rho * cos(phi), rho * sin(phi), 0, 0);
    } else if (type == MeasurementPackage::LASER) {
        // Set the state with the initial location and zero velocity:
        ekf_.setState(measurements[0], measurements[1], 0, 0);
    }


    // PROMPT noise and lambda values:
    cout << setprecision(2) << fixed << endl;

    // Process noise standard deviation acceleration X in MET / S ^ 2
    cout << "──────────────────────────────────────────────────────" << endl << endl;
    const double noiseAX = tools::prompt("STD DEV ACC X", "MET / S ^ 2", 9, 0, 100);

    // Process noise standard deviation acceleration Y in MET / S ^ 2
    cout << "──────────────────────────────────────────────────────" << endl << endl;
    const double noiseAY = tools::prompt("STD DEV ACC Y", "MET / S ^ 2", 9, 0, 100);

    // Set them in the EKF:
    ekf_.setNoise(noiseAX, noiseAY); 


    // Log summary of params that will be used:

    cout
        << "──────────────────────────────────────────────────────" << endl
        << endl
        << " STD DEV ACC X = " << setfill(' ') << setw(5) << noiseAX << " MET / S ^ 2" << endl
        << " STD DEV ACC Y = " << setfill(' ') << setw(5) << noiseAY << " MET / S ^ 2" << endl
        << endl
        << "──────────────────────────────────────────────────────" << endl;


    // Initialize previous_timestamp_:
    previous_timestamp_ = pack.timestamp_;
}


void EKFTracker::updateRMSE(VectorXd gt) {
    const VectorXd residual = getCurrentState() - gt;

    RMSE_sum_ += (residual.array().pow(2)).matrix();
    RMSE_ = (RMSE_sum_ / ++total_).array().sqrt();
}


vector<double> EKFTracker::updateNIS(
    double current_NIS,
    int total_sensor_measurements,
    vector<double> &NIS_table,
    vector<int> &NIS_results
) {
    vector<double> NIS_stats;

    NIS_stats.push_back(current_NIS);

    int i = -1;

    for (vector<double>::iterator it = NIS_table.begin(); it != NIS_table.end(); ++it) {
        if (current_NIS > *it) {                
            NIS_stats.push_back(100 * ++NIS_results[++i] / total_sensor_measurements);
        } else {
            NIS_stats.push_back(100 * NIS_results[++i] / total_sensor_measurements);
        }
    }

    return NIS_stats;
}


void EKFTracker::log(char sensor, vector<double> NIS) {

    if (total_ % 10 == 1) {
        cout
            << "                  │                                    │" << endl
            << "     #  S    TIME │  RMSE X   RMSE Y  RMSE VX  RMSE VY │     NIS   NIS 95   NIS 90   NIS 10    NIS 5" << endl;
    }

    const double RMSE_X = RMSE_(0);
    const double RMSE_Y = RMSE_(1);
    const double RMSE_VX = RMSE_(2);
    const double RMSE_VY = RMSE_(3);

    cout
        << "  " << setfill(' ') << setw(4) << total_
        << "  " << sensor
        << setprecision(0) << fixed
        << setfill(' ') << setw(5) << 1000000 * time_ / total_ << " us"
        << " │ "
        << setprecision(3) << fixed
        << "  " << (RMSE_X > RMSE_X_LIMIT ? C_RED : C_GREEN) << RMSE_X
        << "    " << (RMSE_Y > RMSE_Y_LIMIT ? C_RED : C_GREEN) << RMSE_Y
        << "    " << (RMSE_VX > RMSE_VX_LIMIT ? C_RED : C_GREEN) << RMSE_VX
        << "    " << (RMSE_VY > RMSE_VY_LIMIT ? C_RED : C_GREEN) << RMSE_VY
        << C_RST << " │ "
        << setprecision(3) << fixed
        << setfill(' ') << setw(7) << NIS[0] << "  "
        << setprecision(1) << fixed
        << setfill(' ') << setw(5) << NIS[1] << " %  "
        << setfill(' ') << setw(5) << NIS[2] << " %  "
        << setfill(' ') << setw(5) << NIS[3] << " %  "
        << setfill(' ') << setw(5) << NIS[4] << " %" << endl;
}


// PUBLIC:


EKFTracker::EKFTracker() {

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

    // Measurement covariance matrixes R_lidar and R_radar:
    MatrixXd R_laser(2, 2);
    MatrixXd R_radar(3, 3);

    R_laser <<
        0.0225, 0,
        0, 0.0225;

    R_radar <<
        0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    // Initialize EKF's matrixes:

    ekf_.setMatrixes(F, H, R_laser, R_radar);
}


EKFTracker::~EKFTracker() {}


void  EKFTracker::processMeasurement(const MeasurementPackage &pack) {

    // Compute the time elapsed between the current and previous measurements:
    const double dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

    // Update previous timestamp:
    previous_timestamp_ = pack.timestamp_;

    // INITIALIZATION:

    if (dt <= 0 || dt > MAX_DT) {
        initialize(pack);

        updateRMSE(pack.gt_);

        log(pack.sensor_type_ == MeasurementPackage::RADAR ? 'R' : 'L', { 0, 0, 0, 0, 0 });

        return; // Done initializing. No need to predict or update.
    }


    // PREDICTION:

    // Measure start time:
    clock_t begin = clock();

    // Makes the prediction:
    ekf_.predict(dt);


    // UPDATE:
    // Update UKF, NIS and RSME:

    // OUTPUT current state and state covariance:
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;

    char sensor;
    vector<double> NIS;

    if (pack.sensor_type_ == MeasurementPackage::RADAR) {
        const double NIS_radar = ekf_.updateRadar(pack.raw_measurements_);

        sensor = 'R';
        NIS = updateNIS(NIS_radar, ++total_radar_, NIS_3_table_, radar_NIS_results_);
    } else {
        const double NIS_lidar = ekf_.updateLidar(pack.raw_measurements_);

        sensor = 'L';
        NIS = updateNIS(NIS_lidar, ++total_lidar_, NIS_2_table_, lidar_NIS_results_);
    }

    updateRMSE(pack.gt_);

    // Measure end time:
    clock_t end = clock();

    // Update average time:
    time_ += (double)(end - begin) / CLOCKS_PER_SEC;

    // Log stuff. TODO: Check if log enabled, but always log all at the end!
    log(sensor, NIS);
}


VectorXd EKFTracker::getCurrentState() {
    return ekf_.getCurrentState();
}


VectorXd EKFTracker::getCurrentRMSE() {
    return RMSE_;
}
