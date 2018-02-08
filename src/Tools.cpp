#include "Tools.h"

#include <iostream>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() {}
Tools::~Tools() {}


VectorXd Tools::calculateRMSE(
    const vector<VectorXd> &estimations,
    const vector<VectorXd> &groundTruth
) {

    const unsigned int estimationsCount = estimations.size();

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

        const VectorXd residual = estimations[i] - groundTruth[i];

        // Coefficient-wise multiplication:
        rmse += (residual.array().pow(2)).matrix();
    }

    // Calculate the mean:
    rmse = rmse / estimationsCount;

    // Calculate the squared root and return final result:
    return rmse.array().sqrt();

}
