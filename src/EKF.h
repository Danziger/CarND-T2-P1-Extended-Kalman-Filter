#ifndef EKF_H_
#define EKF_H_


#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


class EKF {

    // State vector:
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

    // Identity matrix (same width and height than x_):
    MatrixXd I_;

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
    */
    void estimate(const VectorXd &y, const MatrixXd &H, const MatrixXd &R);

public:

    EKF();

    virtual ~EKF();

    /**
     * Initializes Kalman filter
     * @param P_in State covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @R_laser_in_ R_in Measurement covariance matrix for laser
     * @R_radar_in_ R_in Measurement covariance matrix for radar
     * @param Q_in Process covariance matrix
     */
    void initMatrixes(
        MatrixXd &P_in,
        MatrixXd &F_in,
        MatrixXd &H_in,
        MatrixXd &R_laser_in_,
        MatrixXd &R_radar_in_
    );

    /**
    * Initializes the current state. 
    */
    void initState(float px, float py, float vx, float vy);

    /**
    * Initializes the process and measurement noises.
    */
    void initNoise(float nx, float ny);

    /**
    * Get the current filter state = px, py, vx, vy
    */
    VectorXd getCurrentState();
    
    /**
     * Intagrates elapsed time in F and Q (recalculates Q) and predicts
     * the state and the state covariance using the process model.
     * @param dt Time between k and k+1 in s
     */
    void predict(float dt);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     * @param R The measurement covariance matrix to use in the update
     */
    void update(const VectorXd &z, const MatrixXd &R);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     * @param R The measurement covariance matrix to use in the update
     */
    void updateEKF(const VectorXd &z, const MatrixXd &R);

    /**
    * Updates the state by using standard Kalman Filter equations and 
    * R_laser_ as the measurement covariance matrix.
    * @param z The measurement at k+1
    */
    void updateLaser(const VectorXd &z);

    /**
    * Updates the state by using Extended Kalman Filter equations and
    * R_radar_ as the measurement covariance matrix.
    * @param z The measurement at k+1
    */
    void updateRadar(const VectorXd &z);
};


#endif /* EKF_H_ */
