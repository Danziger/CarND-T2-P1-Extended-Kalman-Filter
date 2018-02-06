#include "tools.h"

#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() {}


Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(
    const vector<VectorXd> &estimations,
    const vector<VectorXd> &groundTruth
) {

    unsigned int estimationsCount = estimations.size();

    VectorXd rmse(4);

    rmse << 0, 0, 0, 0;

    // Check the validity of the following inputs:
    // - The estimation vector size should not be zero.
    // - The estimation vector size should equal ground truth vector size.

    if (estimationsCount == 0 || estimationsCount != groundTruth.size()) {
        cout << "Invalid estimation or ground_truth data" << endl;

        return rmse;
    }

    // Accumulate squared residuals:

    for (unsigned int i = 0; i < estimationsCount; ++i) {

        VectorXd residual = estimations[i] - groundTruth[i];

        // Coefficient-wise multiplication:
        rmse += (residual.array() * residual.array()).matrix();
    }

    // Calculate the mean:
    rmse = rmse / estimationsCount;

    // Calculate the squared root and return final result:
    return rmse.array().sqrt();

}


MatrixXd Tools::CalculateJacobian(const VectorXd& stateX) {
    MatrixXd Hj(3, 4);

    // Recover state parameters:

    double px = stateX(0);
    double py = stateX(1);
    double vx = stateX(2);
    double vy = stateX(3);

    // Pre-compute a set of terms to avoid repeated calculation:

    double px2 = px * px;
    double py2 = py * py;
    double px2py2 = px2 + py2;
    double d1 = sqrt(px2py2);
    double d2 = px2py2 * d1;

    // Check division by zero:

    if (fabs(px2py2) < 0.0001) {
        cout << "ERROR: CalculateJacobian() - Division by 0." << endl;

        return Hj;
    }

    // Could also be Hj << px / d1, py / d1, 0, 0, -py...

    Hj(0, 0) = px / d1;
    Hj(0, 1) = py / d1;
    Hj(0, 2) = 0;
    Hj(0, 3) = 0;

    Hj(1, 0) = -py / px2py2;
    Hj(1, 1) = px / px2py2;
    Hj(1, 2) = 0;
    Hj(1, 3) = 0;

    Hj(2, 0) = py * (vx*py - vy * px) / d2;
    Hj(2, 1) = px * (vy*px - vx * py) / d2;
    Hj(2, 2) = px / d1;
    Hj(2, 3) = py / d1;

    return Hj;
}
