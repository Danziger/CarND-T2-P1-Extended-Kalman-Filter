#ifndef TOOLS_H_
#define TOOLS_H_


#include "Eigen-3.3/Dense"

#include <vector>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


namespace tools {

    /**
    * Helper method to calculate RMSE.
    */
    VectorXd calculateRMSE(
        const vector<VectorXd> &estimations,
        const vector<VectorXd> &groundTruth
    );

    // PROMPTS:

    /**
    * Prompts the user for a double value and validates it is in the [min, max] interval.
    */
    double prompt(string message, string unit, double def, double min, double max);

    /**
    * Prompts the user for a char value and validates it is one of the options inside options.
    */
    // char prompt(string message, char def, vector<char> options);

    /**
    * Prompts the user for an int value and validates it is in the [min, max] interval.
    */
    // int prompt(string message, int def, int min, int max);
};

#endif /* TOOLS_H_ */
