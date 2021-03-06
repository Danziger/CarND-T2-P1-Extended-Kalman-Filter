#ifndef EKF_H_
#define EKF_H_


#include "common/Eigen-3.3/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


class EKF {

    // State vector: [px, py, vx, vy] in SI units:
    VectorXd x_;

    // State covariance matrix:
    MatrixXd P_;

    // State transition matrix:
    MatrixXd F_;

    // Measurement matrix (laser only):
    MatrixXd H_;

    // Measurement covariance matrixes:
    MatrixXd R_laser_;
    MatrixXd R_radar_;

    // Process covariance matrix:
    MatrixXd Q_;

    // Acceleration noise components:
    float noiseAX_;
    float noiseAY_;


    /**
     * A helper method to map the predicted location from cartesian
     * to polar coordinates.
     */
    VectorXd h(const VectorXd &x);

    /**
     * A helper method to calculate Jacobians.
     */
    MatrixXd calculateJacobian();

    /**
     * Performs the Kalman Filter estimation calculations that are common 
     * for the Standard and the Extended Kalman Filter.
     */
    double estimate(const VectorXd &y, const MatrixXd &H, const MatrixXd &R);


public:

    EKF();

    virtual ~EKF();

    /**
     * Initializes KF/EKF's matrixes
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @R_laser_in_ R_in Measurement covariance matrix for laser
     * @R_radar_in_ R_in Measurement covariance matrix for radar
     * @param Q_in Process covariance matrix
     */
    void setMatrixes(
        const MatrixXd &F,
        const MatrixXd &H,
        const MatrixXd &R_laser,
        const MatrixXd &R_radar
    );

    /**
    * Initializes the state covariance matrix.
    * @param P_in State covariance matrix
    */
    void setP(const MatrixXd &P);

    /**
     * Initializes the current state. 
     */
    void setState(const float px, const float py, const float vx, const float vy);

    /**
     * Initializes the process and measurement noises.
     */
    void setNoise(const float nx, const float ny);

    /**
     * Get the current filter state as [px, py, vx, vy]
     */
    VectorXd getCurrentState();
    
    /**
     * Intagrates elapsed time in F and Q (recalculates Q) and predicts
     * the state and the state covariance using the process model.
     * @param dt Time between k and k+1 in s
     */
    void predict(const double dt);

    /**
     * Updates the state by using Standard Kalman Filter equations
     * @param z The measurement at k+1
     * @param R The measurement covariance matrix to use in the update
     */
    double update(const VectorXd &z, const MatrixXd &R);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     * @param R The measurement covariance matrix to use in the update
     */
    double updateEKF(const VectorXd &z, const MatrixXd &R);

    /**
     * Updates the state by using Standard Kalman Filter equations and 
     * R_laser_ as the measurement covariance matrix.
     * @param z The measurement at k+1
     * @return NIS value for current measurement.
     */
    double updateLidar(const VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations and
     * R_radar_ as the measurement covariance matrix.
     * @param z The measurement at k+1
     * @return NIS value for current measurement.
     */
    double updateRadar(const VectorXd &z);
};


#endif /* EKF_H_ */
