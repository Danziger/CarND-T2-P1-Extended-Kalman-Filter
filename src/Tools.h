#ifndef TOOLS_H_
#define TOOLS_H_


#include "Eigen/Dense"

#include <vector>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


class Tools {

public:

    Tools();
    virtual ~Tools();

    /**
    * A helper method to calculate RMSE.
    */
    VectorXd calculateRMSE(
        const vector<VectorXd> &estimations,
        const vector<VectorXd> &groundTruth
    );

    /**
    * Helper method to normalize angles that will hopefully be inlined by the compiler.
    */
    static double normalizeAngle(double angle);
};


#endif /* TOOLS_H_ */
