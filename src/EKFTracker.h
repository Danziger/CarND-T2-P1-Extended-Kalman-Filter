#ifndef EKFTracker_H_
#define EKFTracker_H_


#include "MeasurementPackage.h"
#include "EKF.h"
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


class EKFTracker {

    // The Extended Kalman Filter (EKF) instance:
    EKF ekf_;

    // Indicates if the EKF state has been initialized (first measurement):
    bool is_initialized_ = false;

    // Previous timestamp:
    long long previous_timestamp_ = 0;

    // NIS stuff:

    int total_lidar_ = 0;
    int total_radar_ = 0;

    vector<int> lidar_NIS_results_{ 0, 0, 0, 0 };
    vector<int> radar_NIS_results_{ 0, 0, 0, 0 };

    vector<double> NIS_2_table_{ 0.103, 0.211, 4.605, 5.991 };
    vector<double> NIS_3_table_{ 0.352, 0.584, 6.251, 7.815 };


    /**
    * Initializes the state of the Kalman Filter using the first measurement.
    */
    void initialize(const MeasurementPackage &pack);


public:

    EKFTracker();

    virtual ~EKFTracker();

    /**
    * Run the whole flow of the KF/EKF and updates NIS results.
    * @return A vector containing the NIS value for the current measurement,
    * either lidar or radar, plus NIS 95, NIS 90, NIS 10 and NIS 5.
    */
    vector<double> processMeasurement(const MeasurementPackage &pack);

    /**
    * Get the current filter state as [px, py, vx, vy]
    */
    VectorXd getCurrentState();

    /**
    * Updates NIS results.
    * @return A vector containing the NIS value for the current measurement,
    * either lidar or radar, plus NIS 95, NIS 90, NIS 10 and NIS 5.
    */
    vector<double> updateNIS(
        double current_NIS,
        int &total_measurements,
        vector<double> &NIS_table,
        vector<int> &NIS_results
    );
};


#endif /* EKFTracker_H_ */
