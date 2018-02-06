#ifndef EKFTracker_H_
#define EKFTracker_H_


#include "MeasurementPackage.h"
#include "EKF.h"
#include "Eigen/Dense"

#include <vector>
#include <string>
#include <fstream>


using Eigen::MatrixXd;
using Eigen::VectorXd;


class EKFTracker {

    // The Extended Kalman Filter (EKF) instance:
    EKF ekf_;

    // Indicates if the EKF state has been initialized (first measurement):
    bool is_initialized_;

    // Previous timestamp:
    long long previous_timestamp_;


    /**
    * Initializes the state of the Kalman Filter using the first measurement.
    */
    void initialize(const MeasurementPackage &pack);


public:

    EKFTracker();

    virtual ~EKFTracker();

    /**
    * Run the whole flow of the KF/EKF from here.
    */
    void processMeasurement(const MeasurementPackage &pack);

    /**
    * Get the current filter state = px, py, vx, vy
    */
    VectorXd getCurrentState();
};


#endif /* EKFTracker_H_ */
