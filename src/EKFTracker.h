#ifndef EKFTracker_H_
#define EKFTracker_H_

#include "EKF.h"
#include "common/MeasurementPackage.h"
#include "common/Eigen-3.3/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


class EKFTracker {

    // The Extended Kalman Filter (EKF) instance:
    EKF ekf_;

    // Previous timestamp:
    long long previous_timestamp_ = 0;

    // Execution time stats:
    double time_;

    // RMSE stats:

    VectorXd RMSE_sum_;
    VectorXd RMSE_;

    // Measurements count:

    int total_;
    int total_lidar_;
    int total_radar_;

    // NIS stats:

    vector<int> lidar_NIS_results_;
    vector<int> radar_NIS_results_;

    vector<double> NIS_2_table_{ 0.103, 0.211, 4.605, 5.991 };
    vector<double> NIS_3_table_{ 0.352, 0.584, 6.251, 7.815 };


    /**
    * Initializes the state of the Kalman Filter using the first measurement.
    */
    void initialize(const MeasurementPackage &pack);

    /**
    * Update RMSE results.
    */
    void updateRMSE(VectorXd gt);

    /**
    * Updates NIS results.
    * @return A vector containing the NIS value for the current measurement,
    * either lidar or radar, plus NIS 95, NIS 90, NIS 10 and NIS 5.
    */
    vector<double> updateNIS(
        double current_NIS,
        int total_sensor_measurements,
        vector<double> &NIS_table,
        vector<int> &NIS_results
    );

    /**
    * Logs measurement index, sensor type, execution time, RMSE and NIS values to screen.
    */
    void log(char sensor, vector<double> NIS);


public:

    EKFTracker();

    virtual ~EKFTracker();

    /**
    * Runs the whole flow of the KF/EKF and updates RMSE and NIS.
    */
    void processMeasurement(const MeasurementPackage &pack);

    /**
    * Get the current filter state as [px, py, vx, vy]
    */
    VectorXd getCurrentState();

    /**
    * Get the current RMSE as [RMSE_X, RMSE_Y, RMSE_VX, RMSE_VY]
    */
    VectorXd getCurrentRMSE();
};


#endif /* EKFTracker_H_ */
