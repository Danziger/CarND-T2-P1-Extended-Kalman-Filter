#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  R_laser_ <<
    0.0225, 0,
    0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;

  // EXTRA INITIALIZATION:
  
  // 4D state vector. We don't know yet the values of the x state.
  ekf_.x_ = VectorXd(4);

  // State covariance matrix P:
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;

  // The initial transition matrix F_:
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  // Measurement matrix:
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  // Process and measurement noises:
  ekf_.noiseAX_ = 9;
  ekf_.noiseAY_ = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &pack) {

  // INITIALIZATION:

  if (!is_initialized_) {
    initialize(pack);

    return;
  }

  // PREDICTION:

  // Compute the time elapsed between the current and previous measurements:

  double dt = (pack.timestamp_ - previous_timestamp_) / 1000000.0;	// In seconds.

  previous_timestamp_ = pack.timestamp_;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Updatethe process noise covariance matrix:
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * ekf_.noiseAX_, 0, dt_3 / 2 * ekf_.noiseAX_, 0,
    0, dt_4 / 4 * ekf_.noiseAY_, 0, dt_3 / 2 * ekf_.noiseAY_,
    dt_3 / 2 * ekf_.noiseAX_, 0, dt_2 * ekf_.noiseAX_, 0,
    0, dt_3 / 2 * ekf_.noiseAY_, 0, dt_2 * ekf_.noiseAY_;

  ekf_.Predict();

  // UPDATE:

   /**
    TODO:
      * Use the sensor type to perform the update step.
      * Update the state and covariance matrices.
    */



  if (pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.Update(pack.raw_measurements_);
  }

  // OUTPUT:

  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}


void FusionEKF::initialize(const MeasurementPackage &pack) {
  /**
  TODO:
  * Initialize the state ekf_.x_ with the first measurement.
  * Create the covariance matrix.
  * Remember: you'll need to convert radar from polar to cartesian coordinates.
  */

  // First measurement:

  // cout << "EKF: " << endl;

  ekf_.x_ = VectorXd(4);
  // ekf_.x_ << 1, 1, 1, 1;

  Eigen::VectorXd measurements = pack.raw_measurements_;
  MeasurementPackage::SensorType type = pack.sensor_type_;

  if (type == MeasurementPackage::RADAR) {
    // Convert radar from polar to cartesian coordinates and initialize state:

    float rho = measurements[0]; // Range
    float phi = measurements[1]; // Bearing

    // rho (range), phi (bearing), rho_dot (velocity)
    ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;
  }
  else if (type == MeasurementPackage::LASER) {
    // Set the state with the initial location and zero velocity:

    ekf_.x_ << measurements[0], measurements[1], 0, 0;
  }

  // OUTPUT initial value:
  cout << "INITIAL x = " << ekf_.x_.transpose() << endl;

  // Done initializing. No need to predict or update:

  previous_timestamp_ = pack.timestamp_;
  is_initialized_ = true;
}